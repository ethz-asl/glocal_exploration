#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_EVALUATOR_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_EVALUATOR_H_

#include <memory>
#include <unordered_map>

#include "glocal_exploration/planning/global/global_planner_base.h"
#include "glocal_exploration/planning/global/submap_frontier.h"
#include "glocal_exploration/planning/global/wavefront_detector.h"

namespace glocal_exploration {
/**
 * A class that tracks and updates frontiers on tsdf-(sub)-maps for target
 * selection in a global planner.
 */
class SubmapFrontierEvaluator : public GlobalPlannerBase {
 public:
  struct Config {
    int verbosity = 2;
    int min_frontier_size = 1;
    bool submaps_are_frozen = true;  // false: submap frontiers will be
                                     // recomputed and overwritten.

    [[nodiscard]] bool isValid() const;
    [[nodiscard]] Config checkValid() const;
  };

  SubmapFrontierEvaluator(const Config& config,
                          std::shared_ptr<Communicator> communicator);
  ~SubmapFrontierEvaluator() override = default;

  void computeFrontiersForSubmap(const MapBase::SubmapData& data,
                                 const Point& initial_point) override;

  void updateFrontiers(
      const std::unordered_map<int, Transformation>& T_M_S) override;

  // access
  const std::unordered_map<int, FrontierCollection>& getFrontiers() const {
    return frontiers_;
  }

 private:
  const Config config_;
  WaveFrontDetector wave_front_detector_;

  std::unordered_map<int, FrontierCollection> frontiers_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_EVALUATOR_H_
