#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_LOCAL_AREA_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_LOCAL_AREA_H_

#include <limits>
#include <set>
#include <unordered_map>

#include <glocal_exploration/mapping/map_base.h>
#include <voxgraph/common.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

namespace glocal_exploration {
class VoxgraphLocalArea {
 public:
  using SubmapId = voxgraph::SubmapID;
  using Transformation = voxblox::Transformation;
  using Point = voxblox::Point;
  using TsdfVoxel = voxblox::TsdfVoxel;
  using VoxelState = MapBase::VoxelState;
  using SubmapIdSet = std::set<SubmapId>;

  explicit VoxgraphLocalArea(const voxblox::TsdfMap::Config& config)
      : local_area_layer_(config.tsdf_voxel_size, config.tsdf_voxels_per_side) {
  }

  void update(const voxgraph::VoxgraphSubmapCollection& submap_collection,
              const voxblox::EsdfMap& local_map);
  void prune();

  VoxelState getVoxelStateAtPosition(const Eigen::Vector3d& position);
  bool isObserved(const Eigen::Vector3d& position);
  bool isValidAtPosition(const Eigen::Vector3d& position);

  void publishLocalArea(ros::Publisher local_area_pub);

 protected:
  static constexpr voxblox::FloatingPoint kTsdfObservedWeight = 1e-3;
  std::unordered_map<SubmapId, Transformation> submaps_in_local_area_;
  voxblox::Layer<TsdfVoxel> local_area_layer_;

  voxgraph::BoundingBox local_map_aabb_;
  void updateLocalMapAabb(const voxblox::EsdfMap& local_map);

  void deintegrateSubmap(const SubmapId submap_id,
                         const voxblox::Layer<TsdfVoxel>& submap_tsdf);
  void integrateSubmap(const SubmapId submap_id,
                       const Transformation& submap_pose,
                       const voxblox::Layer<TsdfVoxel>& submap_tsdf,
                       bool deintegrate = false);

  bool submapPoseChanged(const SubmapId submap_id,
                         const Transformation& new_submap_pose);
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_LOCAL_AREA_H_
