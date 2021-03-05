#include "glocal_exploration_ros/mapping/voxgraph_spatial_hash.h"

#include <unordered_map>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/utils/color_maps.h>

namespace glocal_exploration {

void VoxgraphSpatialHash::update(
    const voxgraph::VoxgraphSubmapCollection& submap_collection) {
  // Update the transform from the odom to a fixed (non-robocentric) frame
  if (submap_collection.empty()) {
    return;
  }
  fixed_frame_transformer_.update(
      submap_collection.getSubmap(submap_collection.getFirstSubmapId())
          .getPose());

  // Get the submap IDs that used to be in the collection
  SubmapIdSet submap_ids_in_spatial_hash;
  for (const auto& submap_kv : submaps_in_spatial_hash_) {
    submap_ids_in_spatial_hash.emplace(submap_kv.first);
  }

  // Get the submap IDs that are currently in the collection
  SubmapIdSet submap_ids_in_submap_collection;
  for (const voxgraph::VoxgraphSubmap::ConstPtr& submap_ptr :
       submap_collection.getSubmapConstPtrs()) {
    submap_ids_in_submap_collection.emplace(submap_ptr->getID());
  }

  // Add the new submaps to the spatial hash
  SubmapIdSet submaps_to_add = set_utils::setDifference(
      submap_ids_in_submap_collection, submap_ids_in_spatial_hash);
  for (const voxgraph::SubmapID submap_id : submaps_to_add) {
    const voxgraph::VoxgraphSubmap& submap =
        submap_collection.getSubmap(submap_id);
    const voxgraph::Transformation T_F_submap =
        fixed_frame_transformer_.transformFromOdomToFixedFrame(
            submap.getPose());
    const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf =
        submap.getTsdfMap().getTsdfLayer();
    //    ROS_INFO_STREAM("Adding submap: " << submap_id);
    addSubmap(submap_id, T_F_submap, submap_tsdf);
  }

  // NOTE: Submaps are currently never deleted from the submap collection,
  //       we therefore don't check for this. If we wanted to add this in the
  //       future, we should store smart pointers to the submaps (or at least
  //       their TSDFs) s.t. they can properly be removed from the hash.

  // Update the submaps that moved by removing them at the old pose and
  // adding them again at their new pose
  for (const voxgraph::SubmapID submap_id : submap_ids_in_spatial_hash) {
    const voxgraph::VoxgraphSubmap& submap =
        submap_collection.getSubmap(submap_id);
    const voxgraph::Transformation T_F_submap_new =
        fixed_frame_transformer_.transformFromOdomToFixedFrame(
            submap.getPose());
    if (submapPoseChanged(submap_id, T_F_submap_new)) {
      const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf =
          submap.getTsdfMap().getTsdfLayer();
      //      ROS_INFO_STREAM("Moving submap: " << submap_id);
      removeSubmap(submap_id, submap_tsdf);
      addSubmap(submap_id, T_F_submap_new, submap_tsdf);
    }
  }
}

void VoxgraphSpatialHash::publishSpatialHash(ros::Publisher spatial_hash_pub) {
  ros::Time current_time = ros::Time::now();
  voxblox::ExponentialOffsetIdColorMap submap_id_color_map;
  std::unordered_map<voxgraph::SubmapID, visualization_msgs::Marker> marker_map;

  for (const auto& block_kv : spatial_submap_id_hash_) {
    const voxblox::BlockIndex& block_index = block_kv.first;
    geometry_msgs::Point position_msg;
    voxblox::Point block_center =
        voxblox::getCenterPointFromGridIndex(block_index, block_grid_size_);
    position_msg.x = block_center.x();
    position_msg.y = block_center.y();
    position_msg.z = block_center.z();

    for (const auto& submap_id : block_kv.second) {
      marker_map[submap_id].points.push_back(position_msg);
    }
  }

  visualization_msgs::MarkerArray marker_array;
  for (auto& marker_kv : marker_map) {
    marker_kv.second.header.frame_id =
        fixed_frame_transformer_.getFixedFrameId();
    marker_kv.second.header.stamp = current_time;
    marker_kv.second.action = visualization_msgs::Marker::ADD;
    marker_kv.second.pose.orientation.w = 1.0;
    marker_kv.second.type = visualization_msgs::Marker::CUBE_LIST;
    marker_kv.second.scale.x = block_grid_size_;
    marker_kv.second.scale.y = block_grid_size_;
    marker_kv.second.scale.z = block_grid_size_;
    marker_kv.second.color.a = 0.5;

    marker_kv.second.ns = "submap_" + std::to_string(marker_kv.first);
    voxblox::Color submap_id_color =
        submap_id_color_map.colorLookup(marker_kv.first);
    marker_kv.second.color.r = submap_id_color.r / 255.0;
    marker_kv.second.color.g = submap_id_color.g / 255.0;
    marker_kv.second.color.b = submap_id_color.b / 255.0;

    marker_array.markers.push_back(marker_kv.second);
  }

  spatial_hash_pub.publish(marker_array);
}

void VoxgraphSpatialHash::removeSubmap(
    const voxgraph::SubmapID submap_id,
    const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf) {
  const auto& submap_it = submaps_in_spatial_hash_.find(submap_id);
  CHECK(submap_it != submaps_in_spatial_hash_.end());
  const voxgraph::Transformation& T_F_submap_old = submap_it->second;
  addSubmap(submap_id, T_F_submap_old, submap_tsdf, true);
}

void VoxgraphSpatialHash::addSubmap(
    const voxgraph::SubmapID submap_id,
    const voxgraph::Transformation& T_F_submap,
    const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf, const bool remove) {
  LOG(INFO) << "Spatial hash: " << (remove ? "Removing" : "Adding")
            << " submap " << submap_id;
  if (remove) {
    if (!submaps_in_spatial_hash_.count(submap_id)) {
      LOG(ERROR) << "Spatial hash: Tried to remove submap that currently isn't "
                    "in the spatial hash. This should never happen.";
      return;
    }
  } else {
    if (submaps_in_spatial_hash_.count(submap_id)) {
      LOG(ERROR) << "Spatial hash: Tried to add submap that is already "
                    "in the spatial hash. This should never happen.";
      return;
    }
  }

  // Precompute the overlapping indices based on the AABB
  std::vector<voxblox::BlockIndex> colliding_index_offsets;
  {
    voxblox::Point aabb_min = voxblox::Point::Constant(INFINITY);
    voxblox::Point aabb_max = voxblox::Point::Constant(-INFINITY);
    for (int idx_x = 0; idx_x < 2; ++idx_x) {
      for (int idx_y = 0; idx_y < 2; ++idx_y) {
        for (int idx_z = 0; idx_z < 2; ++idx_z) {
          voxblox::Point unit_cube_vertex = voxblox::Point::Zero();
          if (idx_x) unit_cube_vertex.x() += 1.f;
          if (idx_y) unit_cube_vertex.y() += 1.f;
          if (idx_z) unit_cube_vertex.z() += 1.f;

          const voxblox::Point rotated_unit_cube_vertex =
              T_F_submap.getRotation().rotate(unit_cube_vertex);

          aabb_min = aabb_min.cwiseMin(rotated_unit_cube_vertex);
          aabb_max = aabb_max.cwiseMax(rotated_unit_cube_vertex);
        }
      }
    }
    const voxblox::BlockIndex aabb_min_idx =
        aabb_min.array().floor().cast<voxblox::IndexElement>();
    const voxblox::BlockIndex aabb_max_idx =
        aabb_max.array().floor().cast<voxblox::IndexElement>();

    // Check if the AABB is sensible
    CHECK((aabb_min_idx.array() <= 0).all() &&
          (0 <= aabb_max_idx.array()).all() &&
          ((aabb_max_idx.array() - aabb_min_idx.array()) <= 3).all())
        << "Faulty index offsets for submap " << submap_id << "\nAABB min "
        << aabb_min.x() << ", " << aabb_min.y() << ", " << aabb_min.z()
        << "; AABB max " << aabb_max.x() << ", " << aabb_max.y() << ", "
        << aabb_max.z() << "\nAABB idx min " << aabb_min_idx.x() << ", "
        << aabb_min_idx.y() << ", " << aabb_min_idx.z() << "; AABB idx max "
        << aabb_max_idx.x() << ", " << aabb_max_idx.y() << ", "
        << aabb_max_idx.z() << "\nat submap rotation rpy:\n"
        << T_F_submap.getRotation().log().x() << ", "
        << T_F_submap.getRotation().log().y() << ", "
        << T_F_submap.getRotation().log().z();

    for (int idx_x = aabb_min_idx.x(); idx_x <= aabb_max_idx.x(); ++idx_x) {
      for (int idx_y = aabb_min_idx.y(); idx_y <= aabb_max_idx.y(); ++idx_y) {
        for (int idx_z = aabb_min_idx.z(); idx_z <= aabb_max_idx.z(); ++idx_z) {
          colliding_index_offsets.emplace_back(idx_x, idx_y, idx_z);
        }
      }
    }
  }
  ROS_INFO_STREAM("Spatial hash block index offset count for submap "
                  << submap_id << ": " << colliding_index_offsets.size());

  voxblox::BlockIndexList submap_blocks;
  submap_tsdf.getAllAllocatedBlocks(&submap_blocks);
  std::lock_guard<std::mutex> spatial_hash_lock(spatial_hash_mutex_);
  for (const voxblox::BlockIndex& submap_block_index : submap_blocks) {
    const voxblox::Point t_submap_block_center =
        voxblox::getCenterPointFromGridIndex(submap_block_index,
                                             block_grid_size_);
    const voxblox::Point t_mission_block_center =
        T_F_submap * t_submap_block_center;
    const auto mission_block_index =
        voxblox::getGridIndexFromPoint<voxblox::BlockIndex>(
            t_mission_block_center, block_grid_size_inv_);

    for (const voxblox::BlockIndex& colliding_index_offset :
         colliding_index_offsets) {
      UnorderedSubmapIdSet& submap_id_set =
          spatial_submap_id_hash_[mission_block_index + colliding_index_offset];
      if (remove) {
        submap_id_set.erase(submap_id);
      } else {
        submap_id_set.insert(submap_id);
      }
    }
  }

  // Update the record of what submaps currently are in the spatial hash
  if (remove) {
    submaps_in_spatial_hash_.erase(submap_id);
  } else {
    submaps_in_spatial_hash_.emplace(submap_id, T_F_submap);
  }
}

bool VoxgraphSpatialHash::submapPoseChanged(
    const voxgraph::SubmapID submap_id,
    const voxgraph::Transformation& T_F_submap_new) {
  const auto& submap_old_it = submaps_in_spatial_hash_.find(submap_id);
  if (submap_old_it == submaps_in_spatial_hash_.end()) {
    LOG(WARNING) << "Requested whether a submap moved even though it has not "
                    "yet been integrated. This should never happen.";
    return false;
  }
  const voxgraph::Transformation& T_F_submap_old = submap_old_it->second;

  const voxgraph::Transformation pose_delta =
      T_F_submap_old.inverse() * T_F_submap_new;
  const FloatingPoint angle_delta = pose_delta.log().tail<3>().norm();
  const FloatingPoint translation_delta = pose_delta.log().head<3>().norm();
  constexpr FloatingPoint kAngleThresholdRad = 0.0872665f;  // 5 deg
  constexpr FloatingPoint kTranslationThresholdM = 1.f;     // 1 m

  return (kTranslationThresholdM < translation_delta ||
          kAngleThresholdRad < angle_delta);
}

}  // namespace glocal_exploration
