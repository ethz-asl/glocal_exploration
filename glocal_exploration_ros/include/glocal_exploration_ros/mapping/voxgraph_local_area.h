#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_LOCAL_AREA_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_LOCAL_AREA_H_

#include <limits>
#include <unordered_map>

#include <glocal_exploration/mapping/map_base.h>
#include <voxgraph/common.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap_collection.h>

namespace glocal_exploration {
class VoxgraphLocalArea {
 public:
  using VoxelState = MapBase::VoxelState;
  using SubmapId = voxgraph::SubmapID;
  using Transformation = voxblox::Transformation;
  using Point = voxblox::Point;

  explicit VoxgraphLocalArea(const voxblox::TsdfMap::Config& config)
      : local_area_layer_(config.tsdf_voxel_size, config.tsdf_voxels_per_side) {
  }

  void update(const voxgraph::VoxgraphSubmapCollection& submap_collection,
              const voxblox::EsdfMap& local_map);

  VoxelState getVoxelStateAtPosition(const Eigen::Vector3d& position);

  void publishLocalArea(ros::Publisher local_area_pub);

 protected:
  std::unordered_map<SubmapId, Transformation> submaps_in_local_area_;

  // NOTE: Since we only need to know whether voxels are known/unknown, we use a
  //       layer of integers counting the number of submaps in which each voxel
  //       was observed.
  using ObservationCounterElement = uint8_t;
  static constexpr ObservationCounterElement kObservationCounterMax =
      std::numeric_limits<ObservationCounterElement>::max();
  struct ObservationCounterVoxel {
    ObservationCounterElement observation_count = 0u;
  };
  voxblox::Layer<ObservationCounterVoxel> local_area_layer_;

  void integrateSubmap(const SubmapId submap_id,
                       const Transformation& submap_pose,
                       const voxblox::Layer<voxblox::TsdfVoxel>& submap_tsdf,
                       bool deintegrate = false);
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_LOCAL_AREA_H_
