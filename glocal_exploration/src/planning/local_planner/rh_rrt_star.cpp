#include "glocal_exploration/planning/local_planner/rh_rrt_star.h"

namespace glocal_exploration {

RHRRTStar::RHRRTStar(std::shared_ptr<MapInterface> map) : LocalPlannerBase(std::move(map)) {}

bool RHRRTStar::setupFromConfig(LocalPlannerBase::Config *config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config *>(config);
  if (!cfg) {
    LOG(ERROR) << "Failed to setup: config is not of type 'VoxbloxMap::Config'.";
    return false;
  }
  config_ = *cfg;
  return true;
}

void RHRRTStar::planningIteration(const StateMachine &state_machine) {

}

} // namespace glocal_exploration
