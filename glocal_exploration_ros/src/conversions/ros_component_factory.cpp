#include "glocal_exploration_ros/conversions/ros_component_factory.h"

#include <memory>
#include <string>

#include <glocal_exploration/3rd_party/config_utilities.hpp>
#include <glocal_exploration/planning/local/rh_rrt_star.h>
#include <glocal_exploration/state/region_of_interest.h>

#include "glocal_exploration_ros/mapping/voxblox_map.h"
#include "glocal_exploration_ros/mapping/voxgraph_map.h"
#include "glocal_exploration_ros/planning/global/skeleton_planner.h"
#include "glocal_exploration_ros/visualization/rh_rrt_star_visualizer.h"
#include "glocal_exploration_ros/visualization/skeleton_visualizer.h"

namespace glocal_exploration {

std::string ComponentFactoryROS::getType(const ros::NodeHandle& nh) {
  std::string type;
  nh.param("type", type, std::string("type param is not set"));
  return type;
}

std::shared_ptr<MapBase> ComponentFactoryROS::createMap(
    const ros::NodeHandle& nh,
    const std::shared_ptr<Communicator>& communicator) {
  std::string type = getType(nh);
  if (type == "voxblox") {
    return std::make_shared<VoxbloxMap>(
        config_utilities::getConfigFromRos<VoxbloxMap::Config>(nh),
        communicator);
  } else if (type == "voxgraph") {
    return std::make_shared<VoxgraphMap>(
        config_utilities::getConfigFromRos<VoxgraphMap::Config>(nh),
        communicator);
  } else {
    LOG(ERROR) << "Unknown map type '" << type << "'.";
    return nullptr;
  }
}

std::shared_ptr<RegionOfInterest> ComponentFactoryROS::createRegionOfInterest(
    const ros::NodeHandle& nh) {
  std::string type = getType(nh);
  if (type == "bounding_box") {
    return std::make_shared<BoundingBox>(
        config_utilities::getConfigFromRos<BoundingBox::Config>(nh));
  } else {
    LOG(ERROR) << "Unknown region of interest type '" << type << "'.";
    return nullptr;
  }
}

std::shared_ptr<LocalPlannerBase> ComponentFactoryROS::createLocalPlanner(
    const ros::NodeHandle& nh,
    const std::shared_ptr<Communicator>& communicator) {
  std::string type = getType(nh);
  if (type == "rh_rrt_star") {
    return std::make_shared<RHRRTStar>(
        config_utilities::getConfigFromRos<RHRRTStar::Config>(nh),
        communicator);
  } else {
    LOG(ERROR) << "Unknown local planner type '" << type << "'.";
    return nullptr;
  }
}

std::shared_ptr<LocalPlannerVisualizerBase>
ComponentFactoryROS::createLocalPlannerVisualizer(
    const ros::NodeHandle& nh,
    const std::shared_ptr<Communicator>& communicator) {
  std::string type = getType(nh);
  if (type == "rh_rrt_star") {
    return std::make_shared<RHRRTStarVisualizer>(
        config_utilities::getConfigFromRos<RHRRTStarVisualizer::Config>(nh),
        communicator);
  } else {
    LOG(WARNING) << "Could not find a visualizer for local planner '" << type
                 << "'.";
    return std::make_shared<LocalPlannerVisualizerBase>(communicator);
  }
}

std::shared_ptr<GlobalPlannerBase> ComponentFactoryROS::createGlobalPlanner(
    const ros::NodeHandle& nh,
    const std::shared_ptr<Communicator>& communicator) {
  std::string type = getType(nh);
  if (type == "skeleton") {
    ros::NodeHandle nh_skeleton_a_star(nh, "skeleton_a_star");
    return std::make_shared<SkeletonPlanner>(
        config_utilities::getConfigFromRos<SkeletonPlanner::Config>(nh),
        config_utilities::getConfigFromRos<SkeletonAStar::Config>(
            nh_skeleton_a_star),
        communicator);
  } else {
    LOG(ERROR) << "Unknown global planner type '" << type << "'.";
    return nullptr;
  }
}

std::shared_ptr<GlobalPlannerVisualizerBase>
ComponentFactoryROS::createGlobalPlannerVisualizer(
    const ros::NodeHandle& nh,
    const std::shared_ptr<Communicator>& communicator) {
  std::string type = getType(nh);
  if (type == "skeleton") {
    return std::make_shared<SkeletonVisualizer>(
        config_utilities::getConfigFromRos<SkeletonVisualizer::Config>(nh),
        communicator);
  } else {
    LOG(WARNING) << "Could not find a visualizer for global planner '" << type
                 << "'.";
    return std::make_shared<GlobalPlannerVisualizerBase>(communicator);
  }
}

}  // namespace glocal_exploration
