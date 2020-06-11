#ifndef GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_PARAMS_H_
#define GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_PARAMS_H_

#include "glocal_exploration_ros/mapping/voxblox_map.h"

namespace glocal_exploration {

VoxbloxMap::Config getVoxbloxMapConfigFromRos(const ros::NodeHandle &nh);

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_PARAMS_H_
