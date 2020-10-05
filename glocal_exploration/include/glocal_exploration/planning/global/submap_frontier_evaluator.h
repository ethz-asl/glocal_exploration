#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_EVALUATOR_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_EVALUATOR_H_

#include <memory>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "glocal_exploration/3rd_party/config_utilities.hpp"
#include "glocal_exploration/planning/global/global_planner_base.h"

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
    int min_num_visible_frontier_points = 1;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  // Definitions.
  using Index = voxblox::GlobalIndex;
  using IndexSet = voxblox::LongIndexSet;

  // Construction.
  SubmapFrontierEvaluator(const Config& config,
                          std::shared_ptr<Communicator> communicator);
  ~SubmapFrontierEvaluator() override = default;

  // Methods.
  void computeFrontiersForSubmap(const MapBase::SubmapData& data,
                                 const Point& initial_point);

  void updateFrontiers(const std::vector<MapBase::SubmapData>& data);

  // Access.
  const std::unordered_map<int, std::vector<Point>>& getFrontierCandidates()
      const {
    return frontier_candidates_;
  }
  const std::vector<std::vector<Point>>& getActiveFrontiers() const {
    return active_frontiers_;
  }
  const std::vector<Point>& getInactiveFrontiers() const {
    return inactive_frontiers_;
  }

 protected:
  void computeFrontierCandidates(
      const voxblox::Layer<voxblox::TsdfVoxel>& layer,
      const Point& initial_point,
      std::pair<const int, std::vector<Point>>* output);

  Index indexFromPoint(const Point& point, FloatingPoint voxel_size_inv) const;
  Point centerPointFromIndex(const Index& index,
                             FloatingPoint voxel_size) const;
  MapBase::VoxelState voxelState(
      const Index& index,
      const voxblox::Layer<voxblox::TsdfVoxel>& layer) const;

 protected:
  const Config config_;

  // Aloow frontier computation in the background, although not in parallel.
  std::unique_ptr<std::thread> frontier_computation_thread_;

  // Store for each submap id (first) all candidates (second) in submap frame.
  std::unordered_map<int, std::vector<Point>> frontier_candidates_;

  // Active frontiers (set of connected active candidates) in mission frame.
  std::vector<std::vector<Point>> active_frontiers_;
  std::vector<Point> inactive_frontiers_;

  // Neighbor lookup.
  const Index kNeighborOffsets[26] = {
      Index(1, 0, 0),   Index(1, 1, 0),   Index(1, -1, 0),  Index(1, 0, 1),
      Index(1, 1, 1),   Index(1, -1, 1),  Index(1, 0, -1),  Index(1, 1, -1),
      Index(1, -1, -1), Index(0, 1, 0),   Index(0, -1, 0),  Index(0, 0, 1),
      Index(0, 1, 1),   Index(0, -1, 1),  Index(0, 0, -1),  Index(0, 1, -1),
      Index(0, -1, -1), Index(-1, 0, 0),  Index(-1, 1, 0),  Index(-1, -1, 0),
      Index(-1, 0, 1),  Index(-1, 1, 1),  Index(-1, -1, 1), Index(-1, 0, -1),
      Index(-1, 1, -1), Index(-1, -1, -1)};
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_EVALUATOR_H_
