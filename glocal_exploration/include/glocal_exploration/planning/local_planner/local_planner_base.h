#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_LOCAL_PLANNER_BASE_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_LOCAL_PLANNER_BASE_H_

#include <memory>

#include "glocal_exploration/common.h"
#include "glocal_exploration/mapping/map_interface.h"
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
  explicit LocalPlannerBase(std::shared_ptr<MapInterface> map) : map_(std::move(map)) {};
  virtual ~LocalPlannerBase() = default;

  /* Setup */
  // Can pass derived configs here by base pointer to setup the map.
  virtual bool setupFromConfig(Config *config) = 0;

  /* General and Accessors */
  virtual void planningIteration(const StateMachine &state_machine) = 0;

 protected:
  std::shared_ptr<MapInterface> map_;
};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_LOCAL_PLANNER_BASE_H_
