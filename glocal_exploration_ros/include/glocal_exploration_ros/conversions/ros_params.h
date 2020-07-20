#ifndef GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_PARAMS_H_
#define GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_PARAMS_H_

#include "glocal_exploration/planning/local/lidar_model.h"
#include "glocal_exploration/planning/local/rh_rrt_star.h"
#include "glocal_exploration/state/region_of_interest.h"
#include "glocal_exploration_ros/mapping/voxblox_map.h"
#include "glocal_exploration_ros/mapping/voxgraph_map.h"
#include "glocal_exploration_ros/visualization/rh_rrt_star_visualizer.h"

namespace glocal_exploration {

VoxbloxMap::Config getVoxbloxMapConfigFromRos(const ros::NodeHandle& nh);

VoxgraphMap::Config getVoxgraphMapConfigFromRos(const ros::NodeHandle& nh);

RHRRTStar::Config getRHRRTStarConfigFromRos(const ros::NodeHandle& nh);

LidarModel::Config getLidarModelConfigFromRos(const ros::NodeHandle& nh);

BoundingBox::Config getBoundingBoxConfigFromRos(const ros::NodeHandle& nh);

RHRRTStarVisualizer::Config getRHRRTStarVisualizerConfigFromRos(
    const ros::NodeHandle& nh);

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_PARAMS_H_
