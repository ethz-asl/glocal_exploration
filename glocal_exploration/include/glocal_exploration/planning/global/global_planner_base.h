#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_GLOBAL_PLANNER_BASE_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_GLOBAL_PLANNER_BASE_H_

#include <memory>
#include <unordered_map>
#include <utility>

#include <voxblox/core/layer.h>

#include "glocal_exploration/common.h"
#include "glocal_exploration/mapping/map_base.h"

namespace glocal_exploration {
class Communicator;

/**
 * Defines the interface of a global planner.
 */
class GlobalPlannerBase {
 public:
  explicit GlobalPlannerBase(std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~GlobalPlannerBase() = default;

  virtual void executePlanningIteration() = 0;

  // NOTE(schmluk): these are curently exposed in the base class for simplicity.
  virtual void computeFrontiersForSubmap(const MapBase::SubmapData& data,
                                         const Point& initial_point) = 0;

  // transformations mission to submap for each id.
  virtual void updateFrontiers(
      const std::unordered_map<int, Transformation>& T_M_S) = 0;

 protected:
  const std::shared_ptr<Communicator> comm_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_GLOBAL_PLANNER_BASE_H_
