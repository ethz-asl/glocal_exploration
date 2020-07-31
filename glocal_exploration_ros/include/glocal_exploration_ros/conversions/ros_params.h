#ifndef GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_PARAMS_H_
#define GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_PARAMS_H_

#include <glocal_exploration/planning/global/submap_frontiers.h>
#include <glocal_exploration/planning/local/lidar_model.h>
#include <glocal_exploration/planning/local/rh_rrt_star.h>
#include <glocal_exploration/state/region_of_interest.h>

#include "glocal_exploration_ros/glocal_system.h"
#include "glocal_exploration_ros/mapping/voxblox_map.h"
#include "glocal_exploration_ros/mapping/voxgraph_map.h"
#include "glocal_exploration_ros/planning/global/skeleton_planner.h"
#include "glocal_exploration_ros/visualization/rh_rrt_star_visualizer.h"
#include "glocal_exploration_ros/visualization/skeleton_visualizer.h"

namespace glocal_exploration {

GlocalSystem::Config getGlocalSystemConfigFromRos(const ros::NodeHandle& nh);

VoxbloxMap::Config getVoxbloxMapConfigFromRos(const ros::NodeHandle& nh);

VoxgraphMap::Config getVoxgraphMapConfigFromRos(const ros::NodeHandle& nh);

RHRRTStar::Config getRHRRTStarConfigFromRos(const ros::NodeHandle& nh);

LidarModel::Config getLidarModelConfigFromRos(const ros::NodeHandle& nh);

BoundingBox::Config getBoundingBoxConfigFromRos(const ros::NodeHandle& nh);

RHRRTStarVisualizer::Config getRHRRTStarVisualizerConfigFromRos(
    const ros::NodeHandle& nh);

SubmapFrontiers::Config getSubmapFrontiersConfigFromRos(
    const ros::NodeHandle& nh);

SkeletonPlanner::Config getSkeletonPlannerConfigFromRos(
    const ros::NodeHandle& nh);

SkeletonVisualizer::Config getSkeletonVisualizerConfigFromRos(
    const ros::NodeHandle& nh);

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_PARAMS_H_
