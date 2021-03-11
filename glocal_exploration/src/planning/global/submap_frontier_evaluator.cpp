#include "glocal_exploration/planning/global/submap_frontier_evaluator.h"

#include <chrono>
#include <memory>
#include <sstream>
#include <stack>
#include <unordered_map>
#include <utility>
#include <vector>

#include "glocal_exploration/state/communicator.h"

namespace glocal_exploration {

SubmapFrontierEvaluator::Config::Config() {
  setConfigName("SubmapFrontierEvaluator");
}

void SubmapFrontierEvaluator::Config::checkParams() const {
  checkParamGT(min_frontier_size, 0, "min_frontier_size");
  checkParamGT(min_num_visible_frontier_points, 0,
               "min_num_visible_frontier_points");
}

void SubmapFrontierEvaluator::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam("min_frontier_size", &min_frontier_size);
  rosParam("submaps_are_frozen", &submaps_are_frozen);
  rosParam("min_num_visible_frontier_points", &min_num_visible_frontier_points);
}

void SubmapFrontierEvaluator::Config::printFields() const {
  printField("verbosity", verbosity);
  printField("min_frontier_size", min_frontier_size);
  printField("submaps_are_frozen", submaps_are_frozen);
  printField("min_num_visible_frontier_points",
             min_num_visible_frontier_points);
}

SubmapFrontierEvaluator::SubmapFrontierEvaluator(
    const Config& config, std::shared_ptr<Communicator> communicator)
    : GlobalPlannerBase(std::move(communicator)),
      config_(config.checkValid()) {}

void SubmapFrontierEvaluator::computeFrontiersForSubmap(
    const MapBase::SubmapData& data, const Point& initial_point) {
  // Only one thread allowed at a time to run in background.
  if (frontier_computation_thread_) {
    if (frontier_computation_thread_->joinable()) {
      frontier_computation_thread_->join();
    }
  }
  // Initialize all frontier candidates for the given layer and id.
  auto it = frontier_candidates_.find(data.id);
  if (it == frontier_candidates_.end()) {
    // New id, setup candidates.
    it = frontier_candidates_
             .insert(std::make_pair(data.id, std::vector<Point>()))
             .first;
  } else if (config_.submaps_are_frozen) {
    // Found an existing frozen frontier.
    return;
  }
  // NOTE: If not frozen the frontiers will be recomputed and overwritten.

  // Compute all frontiers.
  frontier_computation_thread_ =
      std::make_unique<std::thread>([this, data, initial_point, it] {
        computeFrontierCandidates(*(data.tsdf_layer), initial_point, &(*it));
      });
}

void SubmapFrontierEvaluator::updateFrontiers(
    const std::vector<MapBase::SubmapData>& data) {
  // Make sure frontier computation finished.
  if (frontier_computation_thread_) {
    if (frontier_computation_thread_->joinable()) {
      frontier_computation_thread_->join();
    }
  }
  // Verify all frontiers are built. If they are frozen nothing happens.
  int num_candidate_points = 0;
  // NOTE: The submap origin is in free space since it corresponds
  //       to a robot pose by construction.
  Point initial_point(0.f, 0.f, 0.f);
  for (const auto& datum : data) {
    computeFrontiersForSubmap(datum, initial_point);
    num_candidate_points += frontier_candidates_.find(datum.id)->second.size();
  }

  // Transform and condense all submap frontier candidates to global frame and
  // check whether they are active. Then extract connected frontiers.
  auto t_start = std::chrono::high_resolution_clock::now();

  // Combine all candidates to a global set.
  // NOTE(schmluk): Reserving the maximum amount of points might waste a bit of
  //                memory but  prevents costly reallocation of bins and
  //                rehashing the index. This occurs multiple times downstream.
  IndexSet global_frontier_points;
  global_frontier_points.reserve(num_candidate_points);
  FloatingPoint voxel_size = comm_->map()->getVoxelSize();
  CHECK_GT(voxel_size, 0.f);
  FloatingPoint voxel_size_inv = 1.f / voxel_size;
  for (const auto& datum : data) {
    auto it = frontier_candidates_.find(datum.id);
    for (const Point& candidate_S : it->second) {
      Point candidate_M = datum.T_M_S * candidate_S;
      if (comm_->regionOfInterest()->contains(candidate_M)) {
        global_frontier_points.insert(
            indexFromPoint(candidate_M, voxel_size_inv));
      }
    }
  }

  // Verify the right number of transformations were supplied.
  for (const auto& id_submap_pair : frontier_candidates_) {
    if (std::find_if(data.begin(), data.end(),
                     [id_submap_pair](const MapBase::SubmapData& d) {
                       return d.id == id_submap_pair.first;
                     }) == data.end()) {
      LOG(WARNING) << "No update data for submap id " << id_submap_pair.first
                   << " was supplied, its frontier candidates will be ignored.";
    }
  }

  // Remove all inactive candidates.
  const int num_global_candidates = global_frontier_points.size();
  inactive_frontiers_.clear();
  inactive_frontiers_.reserve(num_global_candidates);
  auto it = global_frontier_points.begin();
  while (it != global_frontier_points.end()) {
    Point candidate = centerPointFromIndex(*it, voxel_size);
    if (comm_->map()->isObservedInGlobalMap(candidate)) {
      it = global_frontier_points.erase(it);
      inactive_frontiers_.push_back(candidate);
    } else {
      it++;
    }
  }

  // Cluster frontiers.
  const int num_active_points = global_frontier_points.size();
  int num_final_points = 0;
  int num_frontiers = 0;
  active_frontiers_.clear();
  while (!global_frontier_points.empty()) {
    // Setup search with first point left in candidates.
    IndexSet open_set;
    IndexSet result_set;
    open_set.reserve(num_active_points);
    result_set.reserve(num_active_points);
    open_set.insert(*global_frontier_points.begin());
    result_set.insert(*global_frontier_points.begin());
    global_frontier_points.erase(global_frontier_points.begin());

    // Find all points connected to this frontier.
    while (!open_set.empty()) {
      const Index& open_point = *open_set.begin();
      for (const Index& offset : kNeighborOffsets) {
        const Index candidate = open_point + offset;
        // Check whether it is a valid frontier point.
        if (global_frontier_points.find(candidate) !=
            global_frontier_points.end()) {
          open_set.insert(candidate);
          result_set.insert(candidate);
          global_frontier_points.erase(candidate);
        }
      }
      open_set.erase(open_set.begin());
    }

    // Check whether the final result matches the criteria and write result.
    if (result_set.size() >= config_.min_frontier_size) {
      std::vector<Point>& new_frontier = active_frontiers_.emplace_back();
      new_frontier.reserve(result_set.size());
      for (const Index& idx : result_set) {
        new_frontier.emplace_back(centerPointFromIndex(idx, voxel_size));
      }
      num_final_points += result_set.size();
      num_frontiers++;
    } else {
      for (const Index& idx : result_set) {
        inactive_frontiers_.emplace_back(centerPointFromIndex(idx, voxel_size));
      }
    }
  }

  // Logging.
  auto t_end = std::chrono::high_resolution_clock::now();
  std::stringstream info;
  info << "Updated global frontiers based on " << data.size() << " submaps in "
       << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
              .count()
       << "ms.";
  if (config_.verbosity >= 3) {
    info << " " << num_candidate_points << " candidates -> "
         << num_global_candidates << " global candidates -> "
         << num_active_points << " active points -> " << num_frontiers
         << " frontiers, totaling " << num_final_points << " points.";
  }
  LOG_IF(INFO, config_.verbosity >= 2) << info.str();
}

void SubmapFrontierEvaluator::computeFrontierCandidates(
    const voxblox::Layer<voxblox::TsdfVoxel>& layer, const Point& initial_point,
    std::pair<const int, std::vector<Point>>* output) {
  // Perform a full sweep over the submap's free space to identify frontier
  // candidates. Frontiers are unknown points that border observed free space
  // and are attributed to the submap that contains the free space. Use
  // depth-first search for better cache coherence.
  CHECK_NOTNULL(output);
  auto t_start = std::chrono::high_resolution_clock::now();

  // Cache submap data.
  FloatingPoint voxel_size = layer.voxel_size();
  CHECK_GT(voxel_size, 0.f);
  FloatingPoint voxel_size_inv = 1.f / voxel_size;

  // Setup search.
  IndexSet closed_list;
  std::stack<Index> open_stack;
  std::vector<Point> result;
  open_stack.push(indexFromPoint(initial_point, voxel_size_inv));

  // Search all frontiers.
  while (!open_stack.empty()) {
    // 'current', including the initial point, traverse observed free space.
    Index current = open_stack.top();
    open_stack.pop();

    // Check all neighbors for frontiers and free space.
    for (const Index& offset : kNeighborOffsets) {
      Index candidate = current + offset;
      if (closed_list.find(candidate) != closed_list.end()) {
        // Only consider voxels that were not yet checked.
        continue;
      }
      closed_list.insert(candidate);
      switch (voxelState(candidate, layer)) {
        case MapBase::VoxelState::kFree: {
          // Adjacent free space to continue the search.
          open_stack.push(candidate);
          break;
        }
        case MapBase::VoxelState::kUnknown: {
          // This is a frontier point.
          result.push_back(centerPointFromIndex(candidate, voxel_size));
          break;
        }
        case MapBase::VoxelState::kOccupied:
        default:
          // We hit an obstacle.
          break;
      }
    }
  }
  output->second = result;
  auto t_end = std::chrono::high_resolution_clock::now();

  // Logging
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Found " << result.size() << " frontier candidates in submap "
      << output->first << " in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << "ms.";
}

SubmapFrontierEvaluator::Index SubmapFrontierEvaluator::indexFromPoint(
    const Point& point, FloatingPoint voxel_size_inv) const {
  return voxblox::getGridIndexFromPoint<Index>(point, voxel_size_inv);
}

Point SubmapFrontierEvaluator::centerPointFromIndex(
    const Index& index, FloatingPoint voxel_size) const {
  return voxblox::getCenterPointFromGridIndex(index, voxel_size);
}

MapBase::VoxelState SubmapFrontierEvaluator::voxelState(
    const Index& index, const voxblox::Layer<voxblox::TsdfVoxel>& layer) const {
  voxblox::BlockIndex block_idx;
  voxblox::VoxelIndex voxel_idx;
  voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(
      index, layer.voxels_per_side(), &block_idx, &voxel_idx);
  const auto block = layer.getBlockPtrByIndex(block_idx);
  if (block) {
    const voxblox::TsdfVoxel& voxel = block->getVoxelByVoxelIndex(voxel_idx);
    if (voxel.weight > 1e-6) {
      if (voxel.distance > layer.voxel_size()) {
        // Note(schmluk): The surface is slightly inflated to make detection
        // more conservative and avoid frontiers out in the blue.
        return MapBase::VoxelState::kFree;
      } else {
        return MapBase::VoxelState::kOccupied;
      }
    }
  }
  return MapBase::VoxelState::kUnknown;
}

}  // namespace glocal_exploration
