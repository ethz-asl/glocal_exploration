#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_SPATIAL_HASH_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_SPATIAL_HASH_H_

#include <set>
#include <unordered_map>
#include <unordered_set>

#include <glocal_exploration/common.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

#include "glocal_exploration_ros/mapping/set_utils.h"

namespace glocal_exploration {
class VoxgraphSpatialHash {
 public:
  using SubmapIdSet = std::set<voxgraph::SubmapID>;
  using UnorderedSubmapIdSet = std::unordered_set<voxgraph::SubmapID>;

  using SpatialSubmapIdHash =
      voxblox::AnyIndexHashMapType<UnorderedSubmapIdSet>::type;

  VoxgraphSpatialHash() = default;

  UnorderedSubmapIdSet* getSubmapsAtPosition(const Point& position) {
    const auto mission_block_index =
        voxblox::getGridIndexFromPoint<voxblox::BlockIndex>(
            position.cast<voxblox::FloatingPoint>(), block_grid_size_inv_);
    if (spatial_submap_id_hash_.count(mission_block_index)) {
      return &spatial_submap_id_hash_.at(mission_block_index);
    } else {
      return nullptr;
    }
  }

  void updateSpatialHash(
      const voxgraph::VoxgraphSubmapCollection& submap_collection);

  void publishSpatialHash(ros::Publisher spatial_hash_pub);

 private:
  SpatialSubmapIdHash spatial_submap_id_hash_;
  std::unordered_map<voxgraph::SubmapID, voxgraph::Transformation>
      submaps_in_spatial_hash_;
  const float block_grid_size_ = 3.2;
  const float block_grid_size_inv_ = 1.f / block_grid_size_;

  void removeSubmap(const voxgraph::SubmapID submap_id,
                    const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf);
  void addSubmap(const voxgraph::SubmapID submap_id,
                 const voxgraph::Transformation& T_odom_submap,
                 const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf,
                 const bool remove = false);

  bool submapPoseChanged(const voxgraph::SubmapID submap_id,
                         const voxgraph::Transformation& new_submap_pose);
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_SPATIAL_HASH_H_
