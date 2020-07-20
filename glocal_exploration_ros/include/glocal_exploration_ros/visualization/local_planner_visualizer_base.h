#ifndef GLOCAL_EXPLORATION_ROS_VISUALIZATION_LOCAL_PLANNER_VISUALIZER_BASE_H_
#define GLOCAL_EXPLORATION_ROS_VISUALIZATION_LOCAL_PLANNER_VISUALIZER_BASE_H_

#include <memory>

#include <ros/ros.h>

#include "glocal_exploration/planning/local/local_planner_base.h"

namespace glocal_exploration {

class LocalPlannerVisualizerBase {
 public:
  explicit LocalPlannerVisualizerBase(
      const std::shared_ptr<LocalPlannerBase>& planner) {}
  virtual ~LocalPlannerVisualizerBase() = default;

  // Interface
  virtual void visualize() {}
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_VISUALIZATION_LOCAL_PLANNER_VISUALIZER_BASE_H_
