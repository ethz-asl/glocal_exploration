#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_WAVEFRONT_DETECTOR_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_WAVEFRONT_DETECTOR_H_

#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include <voxblox/core/layer.h>

#include "glocal_exploration/common.h"
#include "glocal_exploration/mapping/map_base.h"

namespace glocal_exploration {

/**
 * Implements the wavefront frontier detection algorithm as described in these
 * papers: https://journals.sagepub.com/doi/full/10.1177/0278364913494911 /
 * https://arxiv.org/pdf/1806.03581.pdf.
 */
class WaveFrontDetector {
 public:
  using Index = voxblox::GlobalIndex;
  using IndexSet = voxblox::LongIndexSet;

  WaveFrontDetector() = default;
  ~WaveFrontDetector() = default;

  // The result contains vectors of points that belong to the same frontier.
  std::vector<std::vector<Point>> computeFrontiers(
      const voxblox::Layer<voxblox::TsdfVoxel>& layer,
      const Point& initial_point);

 private:
  std::vector<std::vector<Point>> mapBFS();
  std::vector<Point> frontierBFS();
  Index indexFromPoint(const Point& point) const;
  Point centerPointFromIndex(const Index& index) const;
  void enqueueMap(const Index& index);
  void enqueueFrontier(const Index& index);
  Index popMap();
  Index popFrontier();
  bool isInMapOpen(const Index& index) const;
  bool isInMapClosed(const Index& index) const;
  bool isInFrontierOpen(const Index& index) const;
  bool isInFrontierClosed(const Index& index) const;
  bool isFrontier(const Index& index) const;
  MapBase::VoxelState voxelState(const Index& index) const;

 private:
  friend class SubmapFrontierEvaluator;
  // Cached data.
  voxblox::Layer<voxblox::TsdfVoxel> const* layer_;
  double voxel_size_;
  double voxel_size_inv_;
  int voxels_per_side_;

  // Frontier detection tracking.
  std::queue<Index> map_queue_;
  std::queue<Index> frontier_queue_;
  IndexSet map_open_list_;
  IndexSet map_closed_list_;
  IndexSet frontier_open_list_;
  IndexSet frontier_closed_list_;

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

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_WAVEFRONT_DETECTOR_H_
