#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_LOCAL_PLANNER_BASE_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_LOCAL_PLANNER_BASE_H_

#include <memory>
#include <utility>

#include "glocal_exploration/common.h"
#include "glocal_exploration/state/waypoint.h"

namespace glocal_exploration {
class Communicator;

/**
 * Defines the interface of a local planner.
 */
class LocalPlannerBase {
 public:
  explicit LocalPlannerBase(std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~LocalPlannerBase() = default;

  // interface
  virtual void executePlanningIteration() = 0;
  virtual void resetPlanner(const WayPoint& new_origin) = 0;

 protected:
  const std::shared_ptr<Communicator> comm_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_LOCAL_PLANNER_BASE_H_
