#ifndef GLOCAL_EXPLORATION_VISUALIZATION_LOCAL_PLANNER_VISUALIZER_
#define GLOCAL_EXPLORATION_VISUALIZATION_LOCAL_PLANNER_VISUALIZER_

#include <memory>

#include <ros/ros.h>

#include "glocal_exploration/planning/local_planner/local_planner_base.h"

namespace glocal_exploration {

class LocalPlannerVisualizerBase {
 public:
  LocalPlannerVisualizerBase(const ros::NodeHandle &nh, const std::shared_ptr<LocalPlannerBase> &planner) : nh_(nh) {}
  virtual ~LocalPlannerVisualizerBase() = default;

  // Interface
  virtual void visualize() {}

 protected:
  ros::NodeHandle nh_;
};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_VISUALIZATION_LOCAL_PLANNER_VISUALIZER_
