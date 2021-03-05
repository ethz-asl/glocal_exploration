#include "glocal_exploration_ros/mapping/voxgraph_local_area.h"

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox/utils/evaluation_utils.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <glocal_exploration/utils/set_utils.h>

namespace glocal_exploration {

void VoxgraphLocalArea::update(
    const voxgraph::VoxgraphSubmapCollection& submap_collection,
    const VoxgraphSpatialHash& spatial_submap_id_hash,
    const voxblox::EsdfMap& local_map) {
  // Update the transform from the odom to a fixed (non-robocentric) frame
  if (submap_collection.empty()) {
    return;
  }
  fixed_frame_transformer_.update(
      submap_collection.getSubmap(submap_collection.getFirstSubmapId())
          .getPose());

  // Find the submaps that currently overlap with the local map
  SubmapIdSet current_neighboring_submaps;
  voxblox::BlockIndexList local_map_block_list;
  local_map.getEsdfLayer().getAllAllocatedBlocks(&local_map_block_list);
  for (const voxblox::BlockIndex& block_index : local_map_block_list) {
    const voxblox::Point t_O_block = voxblox::getCenterPointFromGridIndex(
        block_index, local_map.block_size());
    for (const voxgraph::SubmapID submap_id :
         spatial_submap_id_hash.getSubmapsAtPosition(t_O_block)) {
      current_neighboring_submaps.insert(submap_id);
    }
  }

  // Get the submaps that used to overlap with the local area
  SubmapIdSet old_neighboring_submaps;
  for (const auto& submap_kv : submaps_in_local_area_) {
    old_neighboring_submaps.emplace(submap_kv.first);
  }

  // Flag submaps that just started overlapping for integration
  SubmapIdSet submaps_to_integrate = set_utils::setDifference(
      current_neighboring_submaps, old_neighboring_submaps);

  // Flag submaps that no longer overlap for deintegration
  SubmapIdSet submaps_to_deintegrate = set_utils::setDifference(
      old_neighboring_submaps, current_neighboring_submaps);

  // Flag submaps that still overlap for reintegration if they moved
  {
    SubmapIdSet potential_submaps_to_reintegrate = set_utils::setIntersection(
        old_neighboring_submaps, current_neighboring_submaps);
    for (const SubmapId submap_id : potential_submaps_to_reintegrate) {
      Transformation T_O_submap_new;
      CHECK(submap_collection.getSubmapPose(submap_id, &T_O_submap_new));
      const Transformation T_F_submap_new =
          fixed_frame_transformer_.transformFromOdomToFixedFrame(
              T_O_submap_new);
      if (submapPoseChanged(submap_id, T_F_submap_new)) {
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
    const Transformation T_F_submap =
        fixed_frame_transformer_.transformFromOdomToFixedFrame(
            submap.getPose());
    const voxblox::Layer<TsdfVoxel>& submap_tsdf =
        submap.getTsdfMap().getTsdfLayer();
    integrateSubmap(submap_id, T_F_submap, submap_tsdf);
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

  VLOG(3) << "Pruned " << num_pruned_blocks << " local area blocks";
}

VoxgraphLocalArea::VoxelState VoxgraphLocalArea::getVoxelStateAtPosition(
    const Point& position) {
  const voxblox::Point t_F_position =
      fixed_frame_transformer_.transformFromOdomToFixedFrame(position);
  TsdfVoxel* voxel_ptr =
      local_area_layer_.getVoxelPtrByCoordinates(t_F_position);
  if (voxel_ptr) {
    if (voxel_ptr->weight > kTsdfObservedWeight) {
      if (voxel_ptr->distance > local_area_layer_.voxel_size()) {
        return VoxelState::kFree;
      } else {
        return VoxelState::kOccupied;
      }
    }
  }
  return VoxelState::kUnknown;
}

bool VoxgraphLocalArea::isObserved(const Point& position) {
  const voxblox::Point t_F_position =
      fixed_frame_transformer_.transformFromOdomToFixedFrame(position);
  TsdfVoxel* voxel_ptr =
      local_area_layer_.getVoxelPtrByCoordinates(t_F_position);
  return voxel_ptr && voxblox::utils::isObservedVoxel(*voxel_ptr);
}

void VoxgraphLocalArea::publishLocalArea(ros::Publisher local_area_pub) {
  pcl::PointCloud<pcl::PointXYZI> local_area_pointcloud_msg;
  local_area_pointcloud_msg.header.stamp = ros::Time::now().toNSec() / 1000ull;
  local_area_pointcloud_msg.header.frame_id =
      fixed_frame_transformer_.getFixedFrameId();

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
  CHECK(submap_it != submaps_in_local_area_.end())
      << "Could not find submap with ID: " << submap_id;
  const Transformation& T_F_submap_old = submap_it->second;
  integrateSubmap(submap_id, T_F_submap_old, submap_tsdf, true);
}

void VoxgraphLocalArea::integrateSubmap(
    const SubmapId submap_id,
    const VoxgraphLocalArea::Transformation& T_F_submap,
    const voxblox::Layer<TsdfVoxel>& submap_tsdf, const bool deintegrate) {
  LOG(INFO) << "Local area: " << (deintegrate ? "Deintegrating" : "Integrating")
            << " submap " << submap_id;

  voxblox::Layer<TsdfVoxel> world_frame_submap_tsdf(
      submap_tsdf.voxel_size(), submap_tsdf.voxels_per_side());
  voxblox::transformLayer(submap_tsdf, T_F_submap, &world_frame_submap_tsdf);

  voxblox::BlockIndexList submap_blocks;
  world_frame_submap_tsdf.getAllAllocatedBlocks(&submap_blocks);
  for (const voxblox::BlockIndex& submap_block_index : submap_blocks) {
    const voxblox::Block<TsdfVoxel>& submap_block =
        world_frame_submap_tsdf.getBlockByIndex(submap_block_index);
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
    submaps_in_local_area_.emplace(submap_id, T_F_submap);
  }
}

bool VoxgraphLocalArea::submapPoseChanged(
    const VoxgraphLocalArea::SubmapId submap_id,
    const VoxgraphLocalArea::Transformation& T_F_submap_new) {
  const auto& submap_old_it = submaps_in_local_area_.find(submap_id);
  if (submap_old_it == submaps_in_local_area_.end()) {
    LOG(WARNING) << "Requested whether a submap moved even though it has not "
                    "yet been integrated. This should never happen.";
    return false;
  }
  const Transformation& T_F_submap_old = submap_old_it->second;

  const Transformation pose_delta = T_F_submap_old.inverse() * T_F_submap_new;
  const FloatingPoint angle_delta = pose_delta.log().tail<3>().norm();
  const FloatingPoint translation_delta = pose_delta.log().head<3>().norm();
  constexpr FloatingPoint kAngleThresholdRad = 0.0523599f;  // 3 degrees
  const FloatingPoint translation_threshold = local_area_layer_.voxel_size();

  return (translation_threshold < translation_delta ||
          kAngleThresholdRad < angle_delta);
}

}  // namespace glocal_exploration
