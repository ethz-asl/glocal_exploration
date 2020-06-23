#include "glocal_exploration_ros/conversions/ros_component_factory.h"

#include "glocal_exploration_ros/conversions/ros_params.h"

namespace glocal_exploration {

std::string ComponentFactoryROS::getType(const ros::NodeHandle &nh) {
  std::string type;
  nh.param("type", type, std::string("type param is not set"));
  return type;
}

std::shared_ptr<MapBase> ComponentFactoryROS::createMap(const ros::NodeHandle &nh,
                                                             std::shared_ptr<StateMachine> state_machine) {
  std::string type = getType(nh);
  if (type == "voxblox") {
    auto map = std::make_shared<VoxbloxMap>(state_machine);
    VoxbloxMap::Config cfg = getVoxbloxMapConfigFromRos(nh);
    map->setupFromConfig(&cfg);
    return map;
  } else {
    LOG(ERROR) << "Unknown map type '" << type << "'.";
    return nullptr;
  }
}

std::unique_ptr<LocalPlannerBase> ComponentFactoryROS::createLocalPlanner(const ros::NodeHandle &nh,
                                                                          std::shared_ptr<MapBase> map,
                                                                          std::shared_ptr<StateMachine> state_machine) {
  std::string type = getType(nh);
  if (type == "rh_rrt_star") {
    auto planner = std::make_unique<RHRRTStar>(map, state_machine);
    RHRRTStar::Config cfg = getRHRRTStarConfigFromRos(nh);
    planner->setupFromConfig(&cfg);
    return planner;
  } else {
    LOG(ERROR) << "Unknown local planner type '" << type << "'.";
    return nullptr;
  }
}

} // namespace glocal_exploration
