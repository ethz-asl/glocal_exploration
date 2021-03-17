#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_SPATIAL_HASH_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_SPATIAL_HASH_H_

#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

#include <glocal_exploration/common.h>
#include <glocal_exploration/utils/frame_transformer.h>
#include <glocal_exploration/utils/set_utils.h>

namespace glocal_exploration {
class VoxgraphSpatialHash {
 public:
  using SubmapIdSet = std::set<voxgraph::SubmapID>;
  using UnorderedSubmapIdSet = std::unordered_set<voxgraph::SubmapID>;

  using SpatialSubmapIdHash =
      voxblox::AnyIndexHashMapType<UnorderedSubmapIdSet>::type;

  VoxgraphSpatialHash() : fixed_frame_transformer_("submap_0") {}

  std::vector<voxgraph::SubmapID> getSubmapsAtPosition(
      const Point& position) const {
    const voxblox::Point t_F_block =
        fixed_frame_transformer_.transformFromOdomToFixedFrame(position);
    const auto mission_block_index =
        voxblox::getGridIndexFromPoint<voxblox::BlockIndex>(
            t_F_block, block_grid_size_inv_);
    if (spatial_submap_id_hash_.count(mission_block_index)) {
      std::lock_guard<std::mutex> spatial_hash_lock(spatial_hash_mutex_);
      const UnorderedSubmapIdSet& submap_id_set =
          spatial_submap_id_hash_.at(mission_block_index);
      std::vector<voxgraph::SubmapID> submap_id_vector(submap_id_set.begin(),
                                                       submap_id_set.end());
      return submap_id_vector;
    } else {
      return std::vector<voxgraph::SubmapID>();
    }
  }

  void update(const voxgraph::VoxgraphSubmapCollection& submap_collection);

  void publishSpatialHash(ros::Publisher spatial_hash_pub);

 private:
  SpatialSubmapIdHash spatial_submap_id_hash_;
  std::unordered_map<voxgraph::SubmapID, voxgraph::Transformation>
      submaps_in_spatial_hash_;
  mutable std::mutex spatial_hash_mutex_;

  FrameTransformer fixed_frame_transformer_;

  const float block_grid_size_ = 3.2;
  const float block_grid_size_inv_ = 1.f / block_grid_size_;

  void removeSubmap(const voxgraph::SubmapID submap_id,
                    const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf);
  void addSubmap(const voxgraph::SubmapID submap_id,
                 const voxgraph::Transformation& T_F_submap,
                 const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf,
                 const bool remove = false);

  bool submapPoseChanged(const voxgraph::SubmapID submap_id,
                         const voxgraph::Transformation& T_F_submap_new);
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_SPATIAL_HASH_H_
