#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_

#include <memory>
#include <string>

#include <voxblox_ros/esdf_server.h>

#include "glocal_exploration/mapping/map_base.h"

namespace glocal_exploration {
/**
 * Map class that just uses voxblox as a monolithic map baseline.
 */
class VoxbloxMap : public MapBase {
 public:
  struct Config : MapBase::Config {
    // Since this is a ros-class anyways we make it easy and just get the nh.
    std::string nh_private_namespace = "~";
    double traversability_radius = 0.3;  // m
    double clearing_radius = 0.5;        // m
  };
  explicit VoxbloxMap(const std::shared_ptr<StateMachine>& state_machine);
  virtual ~VoxbloxMap() = default;

  bool setupFromConfig(MapBase::Config* config) override;
  double getVoxelSize() override;
  bool isTraversableInActiveSubmap(
      const Eigen::Vector3d& position,
      const Eigen::Quaterniond& orientation) override;
  VoxelState getVoxelStateInLocalArea(const Eigen::Vector3d& point) override;
  bool getVoxelCenterInLocalArea(Eigen::Vector3d* center,
                                 const Eigen::Vector3d& point) override;

 protected:
  Config config_;
  std::unique_ptr<voxblox::EsdfServer> server_;

  // cached constants
  double c_block_size_;
  double c_voxel_size_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_
