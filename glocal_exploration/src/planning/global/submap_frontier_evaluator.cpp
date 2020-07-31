#include "glocal_exploration/planning/global/submap_frontier_evaluator.h"

#include <chrono>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace glocal_exploration {

bool SubmapFrontierEvaluator::Config::isValid() const { return true; }

SubmapFrontierEvaluator::Config SubmapFrontierEvaluator::Config::checkValid()
    const {
  CHECK(isValid());
  return Config(*this);
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
    it = frontiers_.insert(std::make_pair(data.id, FrontierCollection())).first;
    it->second.id = data.id;
  } else if (!config_.submaps_are_frozen) {
    // If they are not frozen frontiers will be recomputed and overwritten.
    it->second = FrontierCollection();
  }
  FrontierCollection& collection = it->second;

  // Compute all frontiers using the wavefront detector.
  auto t_start = std::chrono::high_resolution_clock::now();

  std::vector<std::vector<Point>> frontiers;
  wave_front_detector_.resetDetectorToLayer(*data.tsdf_layer);
  wave_front_detector_.computeFrontiers(initial_point, &frontiers);

  // Parse the result into a frontier collection.
  collection.T_M_S = data.T_M_S;
  unsigned int number_of_points = 0;
  unsigned int number_of_discarded_frontiers = 0;
  for (const std::vector<Point>& frontier : frontiers) {
    if (frontier.size() < config_.min_frontier_size) {
      number_of_discarded_frontiers++;
      continue;
    }
    Frontier& new_frontier = collection.frontiers.emplace_back(Frontier());
    new_frontier.setPoints(frontier);
    new_frontier.computeCentroid();
    number_of_points += frontier.size();
  }

  // Logging
  auto t_end = std::chrono::high_resolution_clock::now();
  LOG_IF(INFO, config_.verbosity >= 3)
      << "Found " << collection.frontiers.size() << "frontiers, totaling "
      << number_of_points << " points, in submap " << data.id << " in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << "ms";
  LOG_IF(INFO, config_.verbosity >= 4 && number_of_discarded_frontiers > 0)
      << "Discarded " << number_of_discarded_frontiers
      << " frontiers below minimum size.";
}

void SubmapFrontierEvaluator::updateFrontiers(
    const std::unordered_map<int, Transformation>& T_M_S) {}

}  // namespace glocal_exploration
