#include "glocal_exploration_ros/mapping/voxgraph_local_area.h"

#include <unordered_map>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "voxblox_ros/ptcloud_vis.h"

namespace glocal_exploration {

void VoxgraphLocalArea::update(
    const voxgraph::VoxgraphSubmapCollection& submap_collection,
    const voxblox::EsdfMap& local_map) {
  std::unordered_map<SubmapId, Transformation> submaps_to_integrate;
  std::unordered_map<SubmapId, Transformation> submaps_to_deintegrate;

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

  // Flag new neighbours for integration
  for (const voxgraph::VoxgraphSubmap::ConstPtr& submap_in_global_map :
       submap_collection.getSubmapConstPtrs()) {
    const SubmapId global_submap_id = submap_in_global_map->getID();
    if (global_submap_id == submap_collection.getActiveSubmapID()) {
      // Avoid integrating the active submap since its TSDF might still change
      continue;
    }
    if (!submaps_in_local_area_.count(global_submap_id)) {
      if (local_map_aabb.overlapsWith(
              submap_in_global_map->getMissionFrameSubmapAabb())) {
        LOG(INFO) << "Flagging submap " << global_submap_id
                  << " for integration";
        submaps_to_integrate.emplace(global_submap_id,
                                     submap_in_global_map->getPose());
      }
    }
  }

  // Flag submaps that no longer overlap for deintegration
  for (const auto& submap_in_local_area : submaps_in_local_area_) {
    const SubmapId submap_id = submap_in_local_area.first;
    const Transformation& submap_pose = submap_in_local_area.second;
    if (!local_map_aabb.overlapsWith(submap_collection.getSubmap(submap_id)
                                         .getMissionFrameSubmapAabb())) {
      LOG(INFO) << "Flagging submap " << submap_id << " deintegration";
      submaps_to_deintegrate.emplace(submap_id, submap_pose);
    }
  }

  // Flag submaps that moved for reintegration
  for (const auto& pair : submaps_in_local_area_) {
    const SubmapId submap_id = pair.first;
    const Transformation& submap_pose_old = pair.second;
    Transformation submap_pose_new;
    submap_collection.getSubmapPose(submap_id, &submap_pose_new);

    const Transformation pose_delta =
        submap_pose_old.inverse() * submap_pose_new;
    const FloatingPoint angle_delta = pose_delta.log().tail<3>().norm();
    const FloatingPoint translation_delta = pose_delta.log().head<3>().norm();
    constexpr FloatingPoint kAngleThresholdRad = 0.0523599;  // 3 degrees
    const FloatingPoint translation_threshold = local_area_layer_.voxel_size();

    if (translation_threshold < translation_delta ||
        kAngleThresholdRad < angle_delta) {
      LOG(INFO) << "Flagging submap " << submap_id
                << " for move (reintegration)\n"
                << "- translation_delta: " << translation_delta << "\n"
                << "- angle_delta: " << angle_delta;
      submaps_to_deintegrate.emplace(submap_id, submap_pose_old);
      submaps_to_integrate.emplace(submap_id, submap_pose_new);
    }
  }

  // Deintegrate submaps (at old pose)
  for (const auto& submap_to_deintegrate : submaps_to_deintegrate) {
    const SubmapId submap_id = submap_to_deintegrate.first;
    const Transformation& submap_pose = submap_to_deintegrate.second;
    const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf =
        submap_collection.getSubmap(submap_id).getTsdfMap().getTsdfLayer();
    integrateSubmap(submap_id, submap_pose, submap_tsdf, true);
  }

  // Integrate submaps (at new pose)
  for (const auto& submap_to_integrate : submaps_to_integrate) {
    const SubmapId submap_id = submap_to_integrate.first;
    const Transformation& submap_pose = submap_to_integrate.second;
    const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf =
        submap_collection.getSubmap(submap_id).getTsdfMap().getTsdfLayer();
    integrateSubmap(submap_id, submap_pose, submap_tsdf, false);
  }
}

VoxgraphLocalArea::VoxelState VoxgraphLocalArea::getVoxelStateAtPosition(
    const Eigen::Vector3d& position) {
  ObservationCounterVoxel* voxel_ptr =
      local_area_layer_.getVoxelPtrByCoordinates(
          position.cast<FloatingPoint>());
  if (voxel_ptr) {
    const ObservationCounterElement observation_count =
        voxel_ptr->observation_count;
    if (observation_count > 0) {
      // TODO(victorr): Check with Lukas if we could use VoxelState::Observed
      return VoxelState::Free;
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
    const voxblox::Block<ObservationCounterVoxel>& block =
        local_area_layer_.getBlockByIndex(block_index);
    for (voxblox::IndexElement linear_voxel_index = 0;
         linear_voxel_index < block.num_voxels(); ++linear_voxel_index) {
      if (block.getVoxelByLinearIndex(linear_voxel_index).observation_count !=
          0u) {
        pcl::PointXYZI point_msg;
        const Point voxel_position =
            block.computeCoordinatesFromLinearIndex(linear_voxel_index);
        point_msg.x = voxel_position.x();
        point_msg.y = voxel_position.y();
        point_msg.z = voxel_position.z();
        point_msg.intensity = 1;
        local_area_pointcloud_msg.push_back(point_msg);
      }
    }
  }

  local_area_pub.publish(local_area_pointcloud_msg);
}

void VoxgraphLocalArea::integrateSubmap(
    const SubmapId submap_id,
    const VoxgraphLocalArea::Transformation& submap_pose,
    const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf,
    const bool deintegrate) {
  voxblox::Layer<voxblox::TsdfVoxel> world_frame_submap_tsdf(
      submap_tsdf.voxel_size(), submap_tsdf.voxels_per_side());
  voxblox::transformLayer(submap_tsdf, submap_pose, &world_frame_submap_tsdf);

  voxblox::BlockIndexList submap_blocks;
  world_frame_submap_tsdf.getAllAllocatedBlocks(&submap_blocks);
  for (const voxblox::BlockIndex& submap_block_index : submap_blocks) {
    const voxblox::Block<voxblox::TsdfVoxel>& submap_block =
        world_frame_submap_tsdf.getBlockByIndex(submap_block_index);
    voxblox::Block<ObservationCounterVoxel>::Ptr local_area_block =
        local_area_layer_.allocateBlockPtrByIndex(submap_block_index);
    CHECK(local_area_block) << "Local area block allocation failed";
    for (voxblox::IndexElement linear_voxel_index = 0;
         linear_voxel_index < submap_block.num_voxels(); ++linear_voxel_index) {
      if (voxblox::utils::isObservedVoxel(
              submap_block.getVoxelByLinearIndex(linear_voxel_index))) {
        if (deintegrate) {
          if (local_area_block->getVoxelByLinearIndex(linear_voxel_index)
                  .observation_count == 0u) {
            LOG(WARNING) << "Attempted to decrement observation count below "
                            "zero. This should never happen.";
          } else {
            local_area_block->getVoxelByLinearIndex(linear_voxel_index)
                .observation_count -= 1u;
          }
        } else {
          if (local_area_block->getVoxelByLinearIndex(linear_voxel_index)
                  .observation_count == kObservationCounterMax) {
            LOG(WARNING) << "Attempted to increment observation count beyond "
                            "max value of "
                         << kObservationCounterMax
                         << ". This should never happen.";
          } else {
            local_area_block->getVoxelByLinearIndex(linear_voxel_index)
                .observation_count += 1u;
          }
        }
      }
    }
  }

  if (deintegrate) {
    submaps_in_local_area_.erase(submap_id);
  } else {
    submaps_in_local_area_.emplace(submap_id, submap_pose);
  }
}

}  // namespace glocal_exploration
