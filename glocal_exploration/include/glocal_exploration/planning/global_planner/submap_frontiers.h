#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_PLANNER_FRONTIER_PLANNER_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_PLANNER_FRONTIER_PLANNER_H_

#include <queue>

#include <ros/ros.h>

#include "glocal_exploration/planning/global_planner/global_planner_base.h"


namespace glocal_exploration {
/**
 * A class that tracks and updates frontiers on submaps for use in a global planner.
 */
class SubmapFrontiers : public GlobalPlannerBase {
 public:
  struct Config : GlobalPlannerBase::Config {

  };
  SubmapFrontiers(std::shared_ptr<MapBase> map, std::shared_ptr<StateMachine> state_machine);
  virtual ~SubmapFrontiers() = default;

  virtual bool setupFromConfig(GlobalPlannerBase::Config *config) override;

  // interface for global planners that use the frontiers
  void updateFrontiers();
  void computeFrontierCandidates(int submap_id);

 private:
  Config config_;
  //std::shared_ptr<Voxgraph

};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_PLANNING_GLOBAL_PLANNER_FRONTIER_PLANNER_H_
