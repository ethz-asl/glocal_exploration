#ifndef GLOCAL_EXPLORATION_ROS_VISUALIZATION_GLOBAL_PLANNER_VISUALIZER_BASE_H_
#define GLOCAL_EXPLORATION_ROS_VISUALIZATION_GLOBAL_PLANNER_VISUALIZER_BASE_H_

#include <memory>
#include <utility>

#include <glocal_exploration/planning/local/local_planner_base.h>
#include <glocal_exploration/state/communicator.h>

namespace glocal_exploration {

class GlobalPlannerVisualizerBase {
 public:
  explicit GlobalPlannerVisualizerBase(
      std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~GlobalPlannerVisualizerBase() = default;

  // Interface (the default implementation does not visualize anything).
  virtual void visualize() {}

 protected:
  std::shared_ptr<Communicator> comm_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_VISUALIZATION_GLOBAL_PLANNER_VISUALIZER_BASE_H_
