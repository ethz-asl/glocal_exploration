#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_EVALUATOR_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_EVALUATOR_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "glocal_exploration/3rd_party/config_utilities.hpp"
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
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    int min_frontier_size = 1;
    bool submaps_are_frozen = true;  // false: submap frontiers will be
                                     // recomputed and overwritten.
    bool update_frontier_splits = true;  // Recompute the frontier split based
    // on the active points, re-applies min size.

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  SubmapFrontierEvaluator(const Config& config,
                          std::shared_ptr<Communicator> communicator);
  ~SubmapFrontierEvaluator() override = default;

  void computeFrontiersForSubmap(const MapBase::SubmapData& data,
                                 const Point& initial_point) override;

  void updateFrontiers(
      const std::unordered_map<int, Transformation>& T_M_S) override;

  // access
  const std::unordered_map<int, FrontierCollection>& getCandidateCollections()
      const {
    return submap_frontier_collections_;
  }
  const std::unordered_map<int, FrontierCollection>& getUpdatedCollections()
      const {
    if (config_.update_frontier_splits) {
      return active_frontier_collections_;
    } else {
      return submap_frontier_collections_;
    }
  }

 private:
  void updateFrontierSplits();

 private:
  const Config config_;
  WaveFrontDetector wave_front_detector_;

  // Contains the maximal frontier set of each submap, i.e. all candidates.
  std::unordered_map<int, FrontierCollection> submap_frontier_collections_;

  // Contains the active frontiers, only used if update_frontier_splits=true.
  std::unordered_map<int, FrontierCollection> active_frontier_collections_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_EVALUATOR_H_
