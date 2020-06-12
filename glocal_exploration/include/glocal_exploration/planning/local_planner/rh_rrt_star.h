#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_RH_RRT_STAR_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_RH_RRT_STAR_H_

#include "glocal_exploration/common.h"
#include "glocal_exploration/planning/local_planner/local_planner_base.h"

namespace glocal_exploration {

class RHRRTStar : public LocalPlannerBase {
 public:
  // Defines a baseclass for map configurations
  struct Config : LocalPlannerBase::Config{
  };
  RHRRTStar(std::shared_ptr<MapInterface> map);
  virtual ~RHRRTStar() = default;

  // Can pass derived configs here by base pointer to setup the map.
  bool setupFromConfig(LocalPlannerBase::Config *config) override;

  void planningIteration(const StateMachine& state_machine) override;

 protected:
  Config config_;
};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_RH_RRT_STAR_H_
