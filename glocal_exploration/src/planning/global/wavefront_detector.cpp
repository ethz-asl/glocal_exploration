#include "glocal_exploration/planning/global/wavefront_detector.h"

#include <memory>
#include <queue>
#include <utility>
#include <vector>

namespace glocal_exploration {

void WaveFrontDetector::resetDetectorToLayer(
    std::shared_ptr<const voxblox::Layer<voxblox::TsdfVoxel>> layer) {
  layer_ = std::move(layer);
  voxel_size_ = layer_->voxel_size();
  voxel_size_inv_ = 1.0 / voxel_size_;
  voxels_per_side_ = layer_->voxels_per_side();
}

void WaveFrontDetector::computeFrontiers(
    const Point& initial_point, std::vector<std::vector<Point>>* result) {
  result->clear();

  // Reset map queue and lists.
  map_queue_ = std::queue<Index>();
  map_open_list_.clear();
  map_closed_list_.clear();

  // Initialize the map_queue.
  enqueueMap(indexFromPoint(initial_point));

  // Run the search through the map and add frontiers to the result.
  mapBFS(result);
}

void WaveFrontDetector::mapBFS(std::vector<std::vector<Point>>* result) {
  // First BFS through observed free space.
  while (!map_queue_.empty()) {
    Index candidate = popMap();

    if (isInMapClosed(candidate)) {
      continue;
    }

    if (isFrontier(candidate)) {
      // Frontier found: reset frontier queue and lists.
      frontier_queue_ = std::queue<Index>();
      frontier_open_list_.clear();
      frontier_closed_list_.clear();

      // Find all connected points of this frontier and add them to the result.
      enqueueFrontier(candidate);
      frontierBFS(result);
    }

    for (const Index& offset : kNeighborOffsets) {
      Index neighbor = candidate + offset;

      if (!isInMapOpen(neighbor) && !isInMapClosed(neighbor)) {
        // Check if the neighbor is adjacent to open space.
        for (const Index& offset_2 : kNeighborOffsets) {
          if (indexIsFreeSpace(neighbor + offset_2)) {
            enqueueMap(neighbor);
            break;
          }
        }
      }
    }
    map_closed_list_.insert(candidate);
  }
}

void WaveFrontDetector::frontierBFS(std::vector<std::vector<Point>>* result) {
  // Second BFS search for connected frontiers.
  std::vector<Point> frontier;
  while (!frontier_queue_.empty()) {
    Index frontier_candidate = popFrontier();

    if (isInMapClosed(frontier_candidate) ||
        isInFrontierClosed(frontier_candidate)) {
      continue;
    }

    if (isFrontier(frontier_candidate)) {
      // Add the point to the current frontier.
      frontier.emplace_back(centerPointFromIndex(frontier_candidate));

      for (const Index& offset : kNeighborOffsets) {
        Index neighbor = frontier_candidate + offset;

        if (!isInFrontierOpen(neighbor) || !isInFrontierClosed(neighbor) ||
            !isInMapClosed(neighbor)) {
          enqueueFrontier(neighbor);
        }
      }
    }
    frontier_closed_list_.insert(frontier_candidate);
  }
  // Append new frontier to result.
  result->push_back(frontier);

  // Mark the new points as traversed.
  for (const Index& point : frontier_closed_list_) {
    map_closed_list_.insert(point);
  }
}

WaveFrontDetector::Index WaveFrontDetector::indexFromPoint(
    const Point& point) const {
  return voxblox::getGridIndexFromPoint<Index>(
      point.cast<voxblox::FloatingPoint>(), voxel_size_inv_);
}

Point WaveFrontDetector::centerPointFromIndex(const Index& index) const {
  return voxblox::getCenterPointFromGridIndex(index, voxel_size_)
      .cast<FloatingPoint>();
}

void WaveFrontDetector::enqueueMap(const Index& index) {
  map_queue_.push(index);
  map_open_list_.insert(index);
}

void WaveFrontDetector::enqueueFrontier(const Index& index) {
  frontier_queue_.push(index);
  frontier_open_list_.insert(index);
}

WaveFrontDetector::Index WaveFrontDetector::popMap() {
  Index idx = map_queue_.front();
  map_queue_.pop();
  map_open_list_.erase(idx);
  return idx;
}

WaveFrontDetector::Index WaveFrontDetector::popFrontier() {
  Index idx = frontier_queue_.front();
  frontier_queue_.pop();
  frontier_open_list_.erase(idx);
  return idx;
}

bool WaveFrontDetector::isInMapOpen(const Index& index) const {
  return map_open_list_.find(index) != map_open_list_.end();
}

bool WaveFrontDetector::isInMapClosed(const Index& index) const {
  return map_closed_list_.find(index) != map_closed_list_.end();
}

bool WaveFrontDetector::isInFrontierOpen(const Index& index) const {
  return frontier_open_list_.find(index) != frontier_open_list_.end();
}

bool WaveFrontDetector::isInFrontierClosed(const Index& index) const {
  return frontier_closed_list_.find(index) != frontier_closed_list_.end();
}

bool WaveFrontDetector::isFrontier(const Index& index) const {
  // Here frontiers are defined as unknwon voxels that border free space voxels.
  if (indexIsObserved(index)) {
    return false;
  }

  // Check all neighboring voxels.
  for (const Index& offset : kNeighborOffsets) {
    if (indexIsObserved(index + offset)) {
      return true;
    }
  }
  return false;
}

bool WaveFrontDetector::indexIsObserved(const Index& index) const {
  voxblox::BlockIndex block_idx;
  voxblox::VoxelIndex voxel_idx;
  voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(index, voxels_per_side_,
                                                     &block_idx, &voxel_idx);
  auto block_ptr = layer_->getBlockPtrByIndex(block_idx);
  if (block_ptr) {
    return block_ptr->getVoxelByVoxelIndex(voxel_idx).weight > 1e-6;
  }
  return false;
}

bool WaveFrontDetector::indexIsFreeSpace(const Index& index) const {
  voxblox::BlockIndex block_idx;
  voxblox::VoxelIndex voxel_idx;
  voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(index, voxels_per_side_,
                                                     &block_idx, &voxel_idx);
  auto block_ptr = layer_->getBlockPtrByIndex(block_idx);
  if (block_ptr) {
    return block_ptr->getVoxelByVoxelIndex(voxel_idx).distance > 0.0;
  }
  return false;
}

}  // namespace glocal_exploration
