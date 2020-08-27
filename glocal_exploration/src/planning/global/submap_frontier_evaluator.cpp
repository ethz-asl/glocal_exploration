#include "glocal_exploration/planning/global/submap_frontier_evaluator.h"

#include <chrono>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "glocal_exploration/state/communicator.h"

namespace glocal_exploration {

SubmapFrontierEvaluator::Config::Config() {
  setConfigName("SubmapFrontierEvaluator");
}

void SubmapFrontierEvaluator::Config::checkParams() const {}

void SubmapFrontierEvaluator::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam("min_frontier_size", &min_frontier_size);
  rosParam("submaps_are_frozen", &submaps_are_frozen);
}

void SubmapFrontierEvaluator::Config::printFields() const {
  printField("verbosity", verbosity);
  printField("min_frontier_size", min_frontier_size);
  printField("submaps_are_frozen", submaps_are_frozen);
}

std::vector<const Frontier*> SubmapFrontierEvaluator::getActiveFrontiers()
    const {
  std::vector<const Frontier*> result;
  for (const auto& frontiers : frontiers_) {
    for (const auto& frontier : frontiers.second) {
      if (frontier.isActive()) {
        result.push_back(&frontier);
      }
    }
  }
  return result;
}

SubmapFrontierEvaluator::SubmapFrontierEvaluator(
    const Config& config, std::shared_ptr<Communicator> communicator)
    : config_(config.checkValid()),
      GlobalPlannerBase(std::move(communicator)) {}

void SubmapFrontierEvaluator::computeFrontiersForSubmap(
    const MapBase::SubmapData& data, const Point& initial_point) {
  // Initialize all frontier candidates for the given layer and id.

  // Setup the frontier collection.
  auto it = frontiers_.find(data.id);
  if (it == frontiers_.end()) {
    it = frontiers_.insert(std::make_pair(data.id, FrontierCollection(data.id)))
             .first;
  } else if (config_.submaps_are_frozen) {
    // Found an existing frozen frontier.
    return;
  } else {
    // If they are not frozen frontiers will be recomputed and overwritten.
    it->second = FrontierCollection(data.id);
  }
  FrontierCollection& collection = it->second;

  // Compute all frontiers using the wavefront detector.
  auto t_start = std::chrono::high_resolution_clock::now();

  std::vector<std::vector<Point>> frontiers;
  wave_front_detector_.resetDetectorToLayer(data.tsdf_layer);
  wave_front_detector_.computeFrontiers(initial_point, &frontiers);

  // Parse the result into a frontier collection.
  unsigned int number_of_points = 0;
  unsigned int number_of_discarded_frontiers = 0;
  unsigned int number_of_discarded_points = 0;
  for (const std::vector<Point>& frontier : frontiers) {
    if (frontier.size() < config_.min_frontier_size) {
      number_of_discarded_frontiers++;
      number_of_discarded_points += frontier.size();
      continue;
    }
    Frontier& new_frontier = collection.addFrontier();
    new_frontier.setPoints(frontier);
    new_frontier.computeCentroid();
    number_of_points += frontier.size();
  }

  // Logging
  auto t_end = std::chrono::high_resolution_clock::now();
  std::stringstream info;
  info << "Found " << collection.size() << " frontiers, totaling "
       << number_of_points << " points, in submap " << data.id << " in "
       << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
              .count()
       << "ms.";
  if (config_.verbosity >= 3 && number_of_discarded_frontiers > 0) {
    info << " Discarded " << number_of_discarded_frontiers
         << " frontiers below minimum size, totaling "
         << number_of_discarded_points << " points.";
  }
  LOG_IF(INFO, config_.verbosity >= 2) << info.str();
}

void SubmapFrontierEvaluator::updateFrontiers(
    const std::unordered_map<int, Transformation>& T_M_S) {
  auto t_start = std::chrono::high_resolution_clock::now();
  unsigned int num_activated = 0;
  unsigned int num_deactivated = 0;
  unsigned int num_active = 0;
  unsigned int num_frontiers = 0;

  for (const auto& id_tf_pair : T_M_S) {
    // Check all submaps that need to be updated.
    auto it = frontiers_.find(id_tf_pair.first);
    if (it == frontiers_.end()) {
      LOG(WARNING) << "Tried to update non-existing frontiers for submap id "
                   << id_tf_pair.first << ".";
      continue;
    }
    FrontierCollection& collection = it->second;

    // Transform frontiers to the new frame.
    collection.updateFrontierFrame(id_tf_pair.second);

    // Update which candidates are active frontier points.
    for (Frontier& frontier : collection) {
      num_frontiers++;
      unsigned int number_of_active_points = 0;
      for (FrontierCandidate& candidate : frontier) {
        bool was_active = candidate.is_active;
        if (!comm_->regionOfInterest()->contains(candidate.position)) {
          candidate.is_active = false;
        } else {
          candidate.is_active =
              !comm_->map()->isObservedInGlobalMap(candidate.position);
        }
        if (candidate.is_active) {
          number_of_active_points++;
          num_active++;
          if (!was_active) {
            num_activated++;
          }
        } else if (was_active) {
          num_deactivated++;
        }
      }
      frontier.setIsActive(number_of_active_points >=
                           config_.min_frontier_size);
    }
  }

  // Logging
  auto t_end = std::chrono::high_resolution_clock::now();
  std::stringstream info;
  info << "Updated " << num_frontiers << " frontiers in " << T_M_S.size()
       << " submaps in "
       << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
              .count()
       << "ms.";
  if (config_.verbosity >= 3) {
    info << " Activated " << num_activated << ", deactivated "
         << num_deactivated << ", totalling " << num_active
         << " active frontier points.";
  }
  LOG_IF(INFO, config_.verbosity >= 2) << info.str();
}

}  // namespace glocal_exploration
