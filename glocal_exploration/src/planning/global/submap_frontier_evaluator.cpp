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
  rosParam("update_frontier_splits", &update_frontier_splits);
}

void SubmapFrontierEvaluator::Config::printFields() const {
  printField("verbosity", verbosity);
  printField("min_frontier_size", min_frontier_size);
  printField("submaps_are_frozen", submaps_are_frozen);
  printField("update_frontier_splits", update_frontier_splits);
}

SubmapFrontierEvaluator::SubmapFrontierEvaluator(
    const Config& config, std::shared_ptr<Communicator> communicator)
    : config_(config.checkValid()),
      GlobalPlannerBase(std::move(communicator)) {}

void SubmapFrontierEvaluator::computeFrontiersForSubmap(
    const MapBase::SubmapData& data, const Point& initial_point) {
  // Initialize all frontier candidates for the given layer and id.
  // Setup the frontier collection.
  auto it = submap_frontier_collections_.find(data.id);
  if (it == submap_frontier_collections_.end()) {
    it = submap_frontier_collections_
             .insert(std::make_pair(data.id, FrontierCollection(data.id)))
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
  std::vector<std::vector<Point>> frontiers =
      wave_front_detector_.computeFrontiers(*(data.tsdf_layer), initial_point);

  // Parse the result into a frontier collection.
  unsigned int number_of_points = 0;
  unsigned int number_of_discarded_frontiers = 0;
  unsigned int number_of_discarded_points = 0;
  for (const std::vector<Point>& frontier : frontiers) {
    if (frontier.size() < config_.min_frontier_size &&
        !config_.update_frontier_splits) {
      number_of_discarded_frontiers++;
      number_of_discarded_points += frontier.size();
      continue;
    }
    Frontier& new_frontier = collection.addFrontier();
    std::vector<FrontierCandidate> points;
    points.resize(frontier.size());
    for (const Point& pt : frontier) {
      points.emplace_back(pt, true);
    }
    new_frontier.setPoints(points);
    number_of_points += frontier.size();
  }

  // Logging
  auto t_end = std::chrono::high_resolution_clock::now();
  std::stringstream info;
  info << "Found " << collection.size() << " frontier candidates, totaling "
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
    auto it = submap_frontier_collections_.find(id_tf_pair.first);
    if (it == submap_frontier_collections_.end()) {
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
      frontier.computeCentroid();
    }
  }

  // Logging
  auto t_end = std::chrono::high_resolution_clock::now();
  std::stringstream info;
  info << "Updated " << num_frontiers << " frontier candidates in "
       << T_M_S.size() << " submaps in "
       << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
              .count()
       << "ms.";
  if (config_.verbosity >= 3) {
    info << " Activated " << num_activated << ", deactivated "
         << num_deactivated << ", totalling " << num_active
         << " active frontier points.";
  }
  LOG_IF(INFO, config_.verbosity >= 2) << info.str();

  // Continue updating the splits if required.
  if (config_.update_frontier_splits) {
    updateFrontierSplits();
  }
}

void SubmapFrontierEvaluator::updateFrontierSplits() {
  // For all candidate frontier recheck connectivity and apply qualifications.
  active_frontier_collections_.clear();
  auto t_start = std::chrono::high_resolution_clock::now();
  int num_new_frontiers = 0;
  int num_checked_frontiers = 0;
  int num_discarded_frontiers = 0;

  // Transform active candidate points into an index set for tracking track.
  WaveFrontDetector::IndexSet frontier_points;
  for (const auto& id_collection_pair : submap_frontier_collections_) {
    for (const Frontier& frontier : id_collection_pair.second) {
      for (const auto& candidate : frontier) {
        // Note(schmluk): This assumes that the wave_front_detector_ was used
        // before and that the voxel size is constant for all submaps.
        if (candidate.is_active) {
          frontier_points.insert(
              wave_front_detector_.indexFromPoint(candidate.position));
        }
      }
      num_checked_frontiers++;
    }
  }

  // Iterate through all points and find connected frontiers.
  while (!frontier_points.empty()) {
    WaveFrontDetector::IndexSet open_set;
    WaveFrontDetector::IndexSet result_set;
    open_set.insert(*frontier_points.begin());
    result_set.insert(*frontier_points.begin());
    frontier_points.erase(frontier_points.begin());

    // Find all points connected to this frontier.
    while (!open_set.empty()) {
      const WaveFrontDetector::Index& open_point = *open_set.begin();
      for (const auto& offset : wave_front_detector_.kNeighborOffsets) {
        const WaveFrontDetector::Index candidate = open_point + offset;
        // Check whether it is a valid frontier point.
        if (frontier_points.find(candidate) != frontier_points.end()) {
          open_set.insert(candidate);
          result_set.insert(candidate);
          frontier_points.erase(candidate);
        }
      }
      open_set.erase(open_set.begin());
    }

    // Check whether the final result matches the criteria and write result.
    if (result_set.size() >= config_.min_frontier_size) {
      // Find or create the frontier collection.
      auto it = active_frontier_collections_.find(0);
      if (it == active_frontier_collections_.end()) {
        it = active_frontier_collections_
                 .emplace(std::make_pair(0, FrontierCollection(0)))
                 .first;
      }

      // Add the found frontier.
      Frontier& new_frontier = it->second.addFrontier();
      std::vector<FrontierCandidate> points;
      points.reserve(result_set.size());
      for (const auto& index : result_set) {
        points.emplace_back(wave_front_detector_.centerPointFromIndex(index),
                            true);
      }
      new_frontier.setPoints(points);
      new_frontier.setIsActive(true);
      new_frontier.computeCentroid();
      num_new_frontiers++;
    } else {
      num_discarded_frontiers++;
    }
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  LOG_IF(INFO, config_.verbosity >= 3)
      << "Updated splits of " << num_checked_frontiers << " frontiers in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << "ms, found " << num_new_frontiers << " valid and discarded "
      << num_discarded_frontiers << " invalid frontiers.";
}

}  // namespace glocal_exploration
