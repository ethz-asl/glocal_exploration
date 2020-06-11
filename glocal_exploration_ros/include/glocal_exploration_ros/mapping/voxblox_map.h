#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_

#include <memory>
#include <string>

#include <voxblox_ros/esdf_server.h>

#include "glocal_exploration/mapping/map_interface.h"

namespace glocal_exploration {
/**
 * Map class that just uses voxblox as a monolithic map baseline.
 */
class VoxbloxMap : public MapInterface {
 public:
  struct Config : MapInterface::Config {
    // Since this is a ros-class anyways we make it easy and just get the nh.
    std::string nh_private_namespace = "~";
    double collision_radius = 0.5;  // m
  };
  VoxbloxMap() = default;
  virtual ~VoxbloxMap() = default;

  bool setupFromConfig(MapInterface::Config *config) override;
  double getVoxelSize() override;
  bool isTraversableInActiveSubmap(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) override;
  bool getVoxelCenterInLocalArea(Eigen::Vector3d *center, const Eigen::Vector3d &point) override;
  bool isObservedInLocalArea(const Eigen::Vector3d &point) override;

 protected:
  Config config_;
  std::unique_ptr<voxblox::EsdfServer> server_;

  // cached constants
  double c_block_size_;
  double c_voxel_size_;
};

} // namespace glocal_exploration {


#endif //GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_
