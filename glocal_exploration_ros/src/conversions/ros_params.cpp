#include "glocal_exploration_ros/conversions/ros_params.h"

namespace glocal_exploration {

VoxbloxMap::Config getVoxbloxMapConfigFromRos(const ros::NodeHandle &nh) {
  VoxbloxMap::Config config;
  config.nh_private_namespace = nh.getNamespace() + "/voxblox";
  nh.param("collision_radius", config.collision_radius, config.collision_radius);
  return config;
}

} // namespace glocal_exploration
