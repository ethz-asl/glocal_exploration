#include "glocal_exploration_ros/conversions/ros_component_factory.h"

#include <memory>
#include <string>

#include "glocal_exploration_ros/conversions/ros_params.h"

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
    return std::make_shared<VoxbloxMap>(getVoxbloxMapConfigFromRos(nh),
                                        communicator);
  } else if (type == "voxgraph") {
    return std::make_shared<VoxgraphMap>(getVoxgraphMapConfigFromRos(nh),
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
    return std::make_shared<BoundingBox>(getBoundingBoxConfigFromRos(nh));
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
    return std::make_shared<RHRRTStar>(getRHRRTStarConfigFromRos(nh),
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
        getRHRRTStarVisualizerConfigFromRos(nh), communicator);
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
    return std::make_shared<SkeletonPlanner>(
        getSkeletonPlannerConfigFromRos(nh), communicator);
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
        getSkeletonVisualizerConfigFromRos(nh), communicator);
  } else {
    LOG(WARNING) << "Could not find a visualizer for global planner '" << type
                 << "'.";
    return std::make_shared<GlobalPlannerVisualizerBase>(communicator);
  }
}

}  // namespace glocal_exploration
