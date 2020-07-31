#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_GLOBAL_PLANNER_BASE_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_GLOBAL_PLANNER_BASE_H_

#include <memory>
#include <utility>

#include <voxblox/core/layer.h>

#include "glocal_exploration/common.h"
#include "glocal_exploration/state/communicator.h"

namespace glocal_exploration {
/**
 * Defines the interface of a global planner.
 */
class GlobalPlannerBase {
 public:
  explicit GlobalPlannerBase(std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~GlobalPlannerBase() = default;

  /* General and Accessors */
  virtual void planningIteration() = 0;

  // NOTE(schmluk): these are curently exposed in the base class for simplicity.
  virtual void computeFrontiersForSubmap(
      const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer, int submap_id) = 0;
  virtual void updateFrontiers() = 0;

 protected:
  const std::shared_ptr<Comm> comm_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_GLOBAL_PLANNER_BASE_H_
