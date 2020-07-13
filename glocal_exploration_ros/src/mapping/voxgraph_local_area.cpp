#include "glocal_exploration_ros/mapping/voxgraph_local_area.h"

#include <algorithm>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "voxblox_ros/ptcloud_vis.h"

namespace glocal_exploration {

void VoxgraphLocalArea::update(
    const voxgraph::VoxgraphSubmapCollection& submap_collection,
    const voxblox::EsdfMap& local_map) {
  // Get bounding box of local map
  voxgraph::BoundingBox local_map_aabb;
  voxblox::BlockIndexList local_map_blocks;
  local_map.getEsdfLayer().getAllAllocatedBlocks(&local_map_blocks);
  for (const voxblox::BlockIndex& block_index : local_map_blocks) {
    local_map_aabb.min = local_map_aabb.min.cwiseMin(
        local_area_layer_.block_size() * block_index.cast<FloatingPoint>());
    local_map_aabb.max = local_map_aabb.max.cwiseMax(
        local_area_layer_.block_size() *
        (block_index.cast<FloatingPoint>() + Point::Ones()));
  }

  // Find the submaps that currently overlap with the local map
  SubmapIdSet current_neighboring_submaps;
  for (const voxgraph::VoxgraphSubmap::ConstPtr& submap_in_global_map :
       submap_collection.getSubmapConstPtrs()) {
    const SubmapId submap_id = submap_in_global_map->getID();
    if (submap_id == submap_collection.getActiveSubmapID()) {
      // Exclude the active submap since its TSDF might not yet be finished
      continue;
    }
    if (local_map_aabb.overlapsWith(
            submap_in_global_map->getMissionFrameSubmapAabb())) {
      current_neighboring_submaps.emplace(submap_id);
    }
  }

  // Get the submaps that used to overlap with the local area
  SubmapIdSet old_neighboring_submaps;
  for (const auto& submap_kv : submaps_in_local_area_) {
    old_neighboring_submaps.emplace(submap_kv.first);
  }

  // Flag submaps that just started overlapping for integration
  SubmapIdSet submaps_to_integrate =
      setDifference(current_neighboring_submaps, old_neighboring_submaps);

  // Flag submaps that no longer overlap for deintegration
  SubmapIdSet submaps_to_deintegrate =
      setDifference(old_neighboring_submaps, current_neighboring_submaps);

  // Flag submaps that still overlap for reintegration if they moved
  {
    SubmapIdSet potential_submaps_to_reintegrate =
        setIntersection(old_neighboring_submaps, current_neighboring_submaps);
    for (const SubmapId& submap_id : potential_submaps_to_reintegrate) {
      Transformation new_submap_pose;
      CHECK(submap_collection.getSubmapPose(submap_id, &new_submap_pose));
      if (submapPoseChanged(submap_id, new_submap_pose)) {
        submaps_to_deintegrate.emplace(submap_id);
        submaps_to_integrate.emplace(submap_id);
      }
    }
  }

  // Deintegrate submaps (at old pose)
  for (const SubmapId& submap_id : submaps_to_deintegrate) {
    const voxblox::Layer<TsdfVoxel>& submap_tsdf =
        submap_collection.getSubmap(submap_id).getTsdfMap().getTsdfLayer();
    deintegrateSubmap(submap_id, submap_tsdf);
  }

  // Integrate submaps (at new pose)
  for (const SubmapId& submap_id : submaps_to_integrate) {
    const voxgraph::VoxgraphSubmap& submap =
        submap_collection.getSubmap(submap_id);
    const Transformation& submap_pose = submap.getPose();
    const voxblox::Layer<TsdfVoxel>& submap_tsdf =
        submap.getTsdfMap().getTsdfLayer();
    integrateSubmap(submap_id, submap_pose, submap_tsdf);
  }
}

void VoxgraphLocalArea::prune() {
  size_t num_pruned_blocks = 0u;
  voxblox::BlockIndexList blocks_indices;
  local_area_layer_.getAllAllocatedBlocks(&blocks_indices);
  for (const voxblox::BlockIndex& block_index : blocks_indices) {
    const voxblox::Block<TsdfVoxel>& block =
        local_area_layer_.getBlockByIndex(block_index);
    bool block_contains_observed_voxels = false;
    for (size_t linear_index = 0u; linear_index < block.num_voxels();
         ++linear_index) {
      const voxblox::TsdfVoxel& voxel =
          block.getVoxelByLinearIndex(linear_index);
      if (kTsdfObservedWeight < voxel.weight) {
        block_contains_observed_voxels = true;
        break;
      }
    }
    if (!block_contains_observed_voxels) {
      ++num_pruned_blocks;
      local_area_layer_.removeBlock(block_index);
    }
  }

  LOG(INFO) << "Pruned " << num_pruned_blocks << " local area blocks";
}

VoxgraphLocalArea::VoxelState VoxgraphLocalArea::getVoxelStateAtPosition(
    const Eigen::Vector3d& position) {
  TsdfVoxel* voxel_ptr = local_area_layer_.getVoxelPtrByCoordinates(
      position.cast<FloatingPoint>());
  if (voxel_ptr) {
    if (voxel_ptr->weight > kTsdfObservedWeight) {
      if (voxel_ptr->distance > local_area_layer_.voxel_size()) {
        return VoxelState::Free;
      } else {
        return VoxelState::Occupied;
      }
    }
  }
  return VoxelState::Unknown;
}

void VoxgraphLocalArea::publishLocalArea(ros::Publisher local_area_pub) {
  pcl::PointCloud<pcl::PointXYZI> local_area_pointcloud_msg;
  local_area_pointcloud_msg.header.stamp = ros::Time::now().toNSec() / 1000ull;
  local_area_pointcloud_msg.header.frame_id = "odom";

  voxblox::BlockIndexList block_indices;
  local_area_layer_.getAllAllocatedBlocks(&block_indices);
  for (const voxblox::BlockIndex& block_index : block_indices) {
    const voxblox::Block<TsdfVoxel>& block =
        local_area_layer_.getBlockByIndex(block_index);
    for (voxblox::IndexElement linear_voxel_index = 0;
         linear_voxel_index < block.num_voxels(); ++linear_voxel_index) {
      const TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_voxel_index);
      if (voxel.weight > kTsdfObservedWeight) {
        pcl::PointXYZI point_msg;
        const Point voxel_position =
            block.computeCoordinatesFromLinearIndex(linear_voxel_index);
        point_msg.x = voxel_position.x();
        point_msg.y = voxel_position.y();
        point_msg.z = voxel_position.z();
        point_msg.intensity = voxel.distance;
        local_area_pointcloud_msg.push_back(point_msg);
      }
    }
  }

  local_area_pub.publish(local_area_pointcloud_msg);
}

void VoxgraphLocalArea::deintegrateSubmap(
    const SubmapId submap_id, const voxblox::Layer<TsdfVoxel>& submap_tsdf) {
  const auto& submap_it = submaps_in_local_area_.find(submap_id);
  CHECK(submap_it != submaps_in_local_area_.end());
  const Transformation& submap_pose = submap_it->second;
  integrateSubmap(submap_id, submap_pose, submap_tsdf, true);
}

void VoxgraphLocalArea::integrateSubmap(
    const SubmapId submap_id,
    const VoxgraphLocalArea::Transformation& submap_pose,
    const voxblox::Layer<TsdfVoxel>& submap_tsdf, const bool deintegrate) {
  voxblox::Layer<TsdfVoxel> world_frame_submap_tsdf(
      submap_tsdf.voxel_size(), submap_tsdf.voxels_per_side());
  voxblox::transformLayer(submap_tsdf, submap_pose, &world_frame_submap_tsdf);

  voxblox::BlockIndexList submap_blocks;
  world_frame_submap_tsdf.getAllAllocatedBlocks(&submap_blocks);
  for (const voxblox::BlockIndex& submap_block_index : submap_blocks) {
    const voxblox::Block<TsdfVoxel>& submap_block =
        world_frame_submap_tsdf.getBlockByIndex(submap_block_index);
    if (!submap_block.has_data()) {
      LOG(INFO) << "Encountered submap block with no data. Skipping.";
      continue;
    }

    voxblox::Block<TsdfVoxel>::Ptr local_area_block =
        local_area_layer_.allocateBlockPtrByIndex(submap_block_index);
    CHECK(local_area_block) << "Local area block allocation failed";

    for (voxblox::IndexElement linear_voxel_index = 0;
         linear_voxel_index < submap_block.num_voxels(); ++linear_voxel_index) {
      TsdfVoxel& local_area_voxel =
          local_area_block->getVoxelByLinearIndex(linear_voxel_index);
      const TsdfVoxel& submap_voxel =
          submap_block.getVoxelByLinearIndex(linear_voxel_index);

      float signed_submap_voxel_weight;
      if (deintegrate) {
        signed_submap_voxel_weight = -submap_voxel.weight;
      } else {
        signed_submap_voxel_weight = submap_voxel.weight;
      }

      float combined_weight =
          local_area_voxel.weight + signed_submap_voxel_weight;
      if (combined_weight > kTsdfObservedWeight) {
        local_area_voxel.distance =
            (submap_voxel.distance * signed_submap_voxel_weight +
             local_area_voxel.distance * local_area_voxel.weight) /
            combined_weight;
        local_area_voxel.weight = combined_weight;
      } else {
        local_area_voxel.distance = 0.f;
        local_area_voxel.weight = 0.f;
      }
    }
  }

  // Update the record of what submaps currently are in the local area
  if (deintegrate) {
    submaps_in_local_area_.erase(submap_id);
  } else {
    submaps_in_local_area_.emplace(submap_id, submap_pose);
  }
}

bool VoxgraphLocalArea::submapPoseChanged(
    const VoxgraphLocalArea::SubmapId submap_id,
    const VoxgraphLocalArea::Transformation& new_submap_pose) {
  const auto& submap_old_it = submaps_in_local_area_.find(submap_id);
  if (submap_old_it == submaps_in_local_area_.end()) {
    LOG(WARNING) << "Requested whether a submap moved even though it has not "
                    "yet been integrated. This should never happen.";
    return false;
  }
  const Transformation& submap_pose_old = submap_old_it->second;

  const Transformation pose_delta = submap_pose_old.inverse() * new_submap_pose;
  const FloatingPoint angle_delta = pose_delta.log().tail<3>().norm();
  const FloatingPoint translation_delta = pose_delta.log().head<3>().norm();
  constexpr FloatingPoint kAngleThresholdRad = 0.0523599;  // 3 degrees
  const FloatingPoint translation_threshold = local_area_layer_.voxel_size();

  return (translation_threshold < translation_delta ||
          kAngleThresholdRad < angle_delta);
}

VoxgraphLocalArea::SubmapIdSet VoxgraphLocalArea::setDifference(
    const VoxgraphLocalArea::SubmapIdSet& positive_set,
    const VoxgraphLocalArea::SubmapIdSet& negative_set) {
  SubmapIdSet result_set;
  std::set_difference(positive_set.begin(), positive_set.end(),
                      negative_set.begin(), negative_set.end(),
                      std::inserter(result_set, result_set.end()));
  return result_set;
}

VoxgraphLocalArea::SubmapIdSet VoxgraphLocalArea::setIntersection(
    const VoxgraphLocalArea::SubmapIdSet& first_set,
    const VoxgraphLocalArea::SubmapIdSet& second_set) {
  SubmapIdSet result_set;
  std::set_intersection(first_set.begin(), first_set.end(), second_set.begin(),
                        second_set.end(),
                        std::inserter(result_set, result_set.end()));
  return result_set;
}

}  // namespace glocal_exploration
