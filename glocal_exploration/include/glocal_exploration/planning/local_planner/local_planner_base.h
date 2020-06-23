#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_LOCAL_PLANNER_BASE_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_LOCAL_PLANNER_BASE_H_

#include <memory>

#include "glocal_exploration/common.h"
#include "glocal_exploration/mapping/map_base.h"
#include "glocal_exploration/planning/state_machine.h"

namespace glocal_exploration {
/**
 * Defines the interface of a local planner.
 */
class LocalPlannerBase {
 public:
  // Defines a baseclass for map configurations
  struct Config {
    virtual ~Config() = default;
  };
  LocalPlannerBase(std::shared_ptr<MapBase> map, std::shared_ptr<StateMachine> state_machine)
      : map_(std::move(map)), state_machine_(std::move(state_machine)) {};
  virtual ~LocalPlannerBase() = default;

  /* Setup */
  // Can pass derived configs here by base pointer to setup the map.
  virtual bool setupFromConfig(Config *config) = 0;

  /* General and Accessors */
  virtual void planningIteration() = 0;

 protected:
  const std::shared_ptr<MapBase> map_;
  const std::shared_ptr<StateMachine> state_machine_;
};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_LOCAL_PLANNER_BASE_H_
