#include "glocal_exploration/planning/global_planner/submap_frontiers.h"

namespace glocal_exploration {

SubmapFrontiers::SubmapFrontiers(std::shared_ptr<MapBase> map, std::shared_ptr<StateMachine> state_machine)
    : GlobalPlannerBase(std::move(map), std::move(state_machine)) {}

bool SubmapFrontiers::setupFromConfig(GlobalPlannerBase::Config *config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config *>(config);
  if (!cfg) {
    LOG(ERROR) << "Failed to setup: config is not of type 'SubmapFrontiers::Config'.";
    return false;
  }
  config_ = *cfg;

  return true;
}

void SubmapFrontiers::updateFrontiers() {

}

void SubmapFrontiers::computeFrontierCandidates(int submap_id) {

}

} // namespace glocal_exploration
