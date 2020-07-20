#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_LOCAL_PLANNER_BASE_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_LOCAL_PLANNER_BASE_H_

#include <memory>
#include <utility>

#include "glocal_exploration/common.h"

namespace glocal_exploration {
class Communicator;

/**
 * Defines the interface of a local planner.
 */

class LocalPlannerBase {
 public:
  // Defines a baseclass for map configurations
  struct Config {
    virtual ~Config() = default;
  };
  explicit LocalPlannerBase(std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~LocalPlannerBase() = default;

  /* Setup */
  // Can pass derived configs here by base pointer to setup the map.
  virtual bool setupFromConfig(Config* config) = 0;

  /* General and Accessors */
  virtual void planningIteration() = 0;

 protected:
  const std::shared_ptr<Communicator> comm_;
};

}  // namespace glocal_exploration

#include "glocal_exploration/state/communicator.h"

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_LOCAL_PLANNER_BASE_H_
