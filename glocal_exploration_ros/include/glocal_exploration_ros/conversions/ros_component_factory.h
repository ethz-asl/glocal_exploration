#ifndef GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_COMPONENT_FACTORY_H_
#define GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_COMPONENT_FACTORY_H_

#include <memory>
#include <string>

#include <ros/node_handle.h>

#include "glocal_exploration/mapping/map_interface.h"
#include "glocal_exploration/planning/local_planner/rh_rrt_star.h"

namespace glocal_exploration {

class ComponentFactoryROS {
 public:
  virtual ~ComponentFactoryROS() = default;

  static std::shared_ptr<MapInterface> createMap(const ros::NodeHandle &nh);
  static std::unique_ptr<LocalPlannerBase> createLocalPlanner(const ros::NodeHandle &nh,
                                                              const std::shared_ptr<MapInterface> &map);

 private:
  ComponentFactoryROS() = default;
  static std::string getType(const ros::NodeHandle &nh);
};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_COMPONENT_FACTORY_H_
