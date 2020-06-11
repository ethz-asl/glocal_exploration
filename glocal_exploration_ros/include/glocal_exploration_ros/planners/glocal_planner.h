#ifndef GLOCAL_EXPLORATION_PLANNERS_GLOCAL_PLANNER_H_
#define GLOCAL_EXPLORATION_PLANNERS_GLOCAL_PLANNER_H_

#include <memory>

#include <ros/ros.h>

#include "glocal_exploration/mapping/map_interface.h"

namespace glocal_exploration {

/**
 * Currently just a placeholder class
 */
class GlocalPlanner {
 public:
  GlocalPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~GlocalPlanner() = default;

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  /* components */
  std::unique_ptr<MapInterface> map_;
};

} // namespace glocal_exploration

#endif //GLOCAL_EXPLORATION_PLANNERS_GLOCAL_PLANNER_H_
