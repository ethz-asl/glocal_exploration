#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_WAVEFRONT_DETECTOR_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_WAVEFRONT_DETECTOR_H_

#include <memory>
#include <queue>
#include <vector>

#include <voxblox/core/layer.h>

#include "glocal_exploration/common.h"

namespace glocal_exploration {

/**
 * Implements the wavefront frontier detection algorithm as described in this
 * paper: https://journals.sagepub.com/doi/full/10.1177/0278364913494911 /
 * https://arxiv.org/pdf/1806.03581.pdf.
 */
class WaveFrontDetector {
 public:
  using Index = voxblox::GlobalIndex;
  using IndexSet = voxblox::LongIndexSet;

  WaveFrontDetector() = default;
  ~WaveFrontDetector() = default;

  void resetDetectorToLayer(const voxblox::Layer<voxblox::TsdfVoxel>& layer);

  // The result contains vectors of points that belong to the same frontier.
  void computeFrontiers(const Point& initial_point,
                        std::vector<std::vector<Point>>*);

 private:
  void mapBFS(std::vector<std::vector<Point>>* result);
  void frontierBFS(std::vector<std::vector<Point>>* result);
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
  bool indexIsObserved(const Index& index) const;
  bool indexIsFreeSpace(const Index& index) const;

 private:
  const Index kNeighborOffsets[26] = {
      Index(1, 0, 0),   Index(1, 1, 0),   Index(1, -1, 0),  Index(1, 0, 1),
      Index(1, 1, 1),   Index(1, -1, 1),  Index(1, 0, -1),  Index(1, 1, -1),
      Index(1, -1, -1), Index(0, 1, 0),   Index(0, -1, 0),  Index(0, 0, 1),
      Index(0, 1, 1),   Index(0, -1, 1),  Index(0, 0, -1),  Index(0, 1, -1),
      Index(0, -1, -1), Index(-1, 0, 0),  Index(-1, 1, 0),  Index(-1, -1, 0),
      Index(-1, 0, 1),  Index(-1, 1, 1),  Index(-1, -1, 1), Index(-1, 0, -1),
      Index(-1, 1, -1), Index(-1, -1, -1)};

  // cached data
  std::shared_ptr<const voxblox::Layer<voxblox::TsdfVoxel>> layer_;
  double voxel_size_;
  double voxel_size_inv_;
  int voxels_per_side_;

  // frontier detection tracking
  std::queue<Index> map_queue_;
  std::queue<Index> frontier_queue_;
  IndexSet map_open_list_;
  IndexSet map_closed_list_;
  IndexSet frontier_open_list_;
  IndexSet frontier_closed_list_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_WAVEFRONT_DETECTOR_H_
