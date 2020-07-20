#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_MAP_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_MAP_H_

#include <memory>
#include <string>

#include <glocal_exploration/mapping/map_base.h>

#include "glocal_exploration_ros/mapping/threadsafe_wrappers/threadsafe_voxblox_server.h"
#include "glocal_exploration_ros/mapping/threadsafe_wrappers/threadsafe_voxgraph_server.h"

namespace glocal_exploration {
/**
 * Map class that just uses voxgraph as a monolithic map baseline.
 */
class VoxgraphMap : public MapBase {
 public:
  struct Config : MapBase::Config {
    // Since this is a ros-class anyways we make it easy and just get the nh.
    std::string nh_private_namespace = "~";
    double traversability_radius = 0.3;  // m
    double clearing_radius = 0.5;        // m
  };
  explicit VoxgraphMap(const std::shared_ptr<Communicator>& communicator);
  virtual ~VoxgraphMap() = default;

  bool setupFromConfig(MapBase::Config* config) override;

  bool isTraversableInActiveSubmap(
      const Eigen::Vector3d& position,
      const Eigen::Quaterniond& orientation) override;
  VoxelState getVoxelStateInLocalArea(const Eigen::Vector3d& point) override;

  double getVoxelSize() override { return c_voxel_size_; }
  Eigen::Vector3d getVoxelCenterInLocalArea(
      const Eigen::Vector3d& point) override;

 protected:
  Config config_;

  std::unique_ptr<ThreadsafeVoxbloxServer> voxblox_server_;
  std::unique_ptr<ThreadsafeVoxgraphServer> voxgraph_server_;

  // cached constants
  double c_block_size_;
  double c_voxel_size_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_MAP_H_
