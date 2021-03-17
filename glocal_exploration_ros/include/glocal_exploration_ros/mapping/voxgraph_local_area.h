#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_LOCAL_AREA_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_LOCAL_AREA_H_

#include <limits>
#include <set>
#include <string>
#include <unordered_map>

#include <voxgraph/common.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

#include <glocal_exploration/mapping/map_base.h>
#include <glocal_exploration/utils/frame_transformer.h>

#include "glocal_exploration_ros/mapping/voxgraph_spatial_hash.h"

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
      : local_area_layer_(config.tsdf_voxel_size, config.tsdf_voxels_per_side),
        fixed_frame_transformer_("submap_0") {}

  void update(const voxgraph::VoxgraphSubmapCollection& submap_collection,
              const VoxgraphSpatialHash& spatial_submap_id_hash,
              const voxblox::EsdfMap& local_map);
  void prune();

  VoxelState getVoxelStateAtPosition(const Point& position);
  bool isObserved(const Point& position);
  bool isValidAtPosition(const Point& position);

  void publishLocalArea(ros::Publisher local_area_pub);

 protected:
  static constexpr FloatingPoint kTsdfObservedWeight = 1e-3;

  std::unordered_map<SubmapId, Transformation> submaps_in_local_area_;
  voxblox::Layer<TsdfVoxel> local_area_layer_;

  FrameTransformer fixed_frame_transformer_;

  void deintegrateSubmap(const SubmapId submap_id,
                         const voxblox::Layer<TsdfVoxel>& submap_tsdf);
  void integrateSubmap(const SubmapId submap_id,
                       const Transformation& T_F_submap,
                       const voxblox::Layer<TsdfVoxel>& submap_tsdf,
                       bool deintegrate = false);

  bool submapPoseChanged(const SubmapId submap_id,
                         const Transformation& T_F_submap_new);
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_LOCAL_AREA_H_
