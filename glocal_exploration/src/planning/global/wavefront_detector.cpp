#include "glocal_exploration/planning/global/wavefront_detector.h"

#include <memory>
#include <queue>
#include <utility>
#include <vector>

namespace glocal_exploration {

std::vector<std::vector<Point>> WaveFrontDetector::computeFrontiers(
    const voxblox::Layer<voxblox::TsdfVoxel>& layer,
    const Point& initial_point) {
  // Cache layer data.
  layer_ = &layer;  // layer_ is only used while layer is in scope of this fn.
  voxel_size_ = layer_->voxel_size();
  voxel_size_inv_ = 1.0 / voxel_size_;
  voxels_per_side_ = layer_->voxels_per_side();
  // Reset map queue and lists.
  map_queue_ = std::queue<Index>();
  map_open_list_.clear();
  map_closed_list_.clear();

  // Initialize the map_queue.
  enqueueMap(indexFromPoint(initial_point));

  // Run the search through the map and add frontiers to the result.
  return mapBFS();
}

std::vector<std::vector<Point>> WaveFrontDetector::mapBFS() {
  // First BFS through observed free space.
  std::vector<std::vector<Point>> result;
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
      result.emplace_back(frontierBFS());
    }

    for (const Index& offset : kNeighborOffsets) {
      Index neighbor = candidate + offset;
      if (!isInMapOpen(neighbor) && !isInMapClosed(neighbor)) {
        // Check if the neighbor is adjacent to open space.
        for (const Index& offset_2 : kNeighborOffsets) {
          if (voxelState(neighbor + offset_2) == MapBase::VoxelState::kFree) {
            enqueueMap(neighbor);
            break;
          }
        }
      }
    }
    map_closed_list_.insert(candidate);
  }
  return result;
}

std::vector<Point> WaveFrontDetector::frontierBFS() {
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

      // Enqueue all neighbors to check for frontiers.
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

  // Mark the new points as traversed.
  for (const Index& point : frontier_closed_list_) {
    map_closed_list_.insert(point);
  }
  return frontier;
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
  // Here frontiers are defined as unknown voxels that border free space voxels.
  if (voxelState(index) != MapBase::VoxelState::kUnknown) {
    return false;
  }

  // Check all neighboring voxels.
  for (const Index& offset : kNeighborOffsets) {
    if (voxelState(index + offset) == MapBase::VoxelState::kFree) {
      return true;
    }
  }
  return false;
}

MapBase::VoxelState WaveFrontDetector::voxelState(const Index& index) const {
  voxblox::BlockIndex block_idx;
  voxblox::VoxelIndex voxel_idx;
  voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(index, voxels_per_side_,
                                                     &block_idx, &voxel_idx);
  auto block = layer_->getBlockPtrByIndex(block_idx);
  if (block) {
    const voxblox::TsdfVoxel& voxel = block->getVoxelByVoxelIndex(voxel_idx);
    if (voxel.weight <= 1e-6) {
      return MapBase::VoxelState::kUnknown;
    } else if (voxel.distance > voxel_size_) {
      // Note(schmluk): The surface is slightly inflated to make detection more
      // conservative and avoid frontiers out in the blue.
      return MapBase::VoxelState::kFree;
    } else {
      return MapBase::VoxelState::kOccupied;
    }
  }
  return MapBase::VoxelState::kUnknown;
}

}  // namespace glocal_exploration
