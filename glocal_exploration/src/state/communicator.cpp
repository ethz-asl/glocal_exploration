#include "glocal_exploration/state/communicator.h"

#include <memory>
#include <utility>

namespace glocal_exploration {
Communicator::Communicator()
    : target_reached_(false), new_waypoint_requested_(false) {}

void Communicator::setupStateMachine(
    std::shared_ptr<StateMachine> state_machine) {
  state_machine_ = std::move(state_machine);
}

void Communicator::setupMap(std::shared_ptr<MapBase> map) {
  map_ = std::move(map);
}

void Communicator::setupLocalPlanner(
    std::shared_ptr<LocalPlannerBase> local_planner) {
  local_planner_ = std::move(local_planner);
}

void Communicator::setupGlobalPlanner(
    std::shared_ptr<GlobalPlannerBase> global_planner) {
  global_planner_ = std::move(global_planner);
}

void Communicator::setupRegionOfInterest(
    std::shared_ptr<RegionOfInterest> roi) {
  roi_ = std::move(roi);
}

void Communicator::requestWayPoint(const WayPoint& way_point) {
  new_waypoint_requested_ = true;
  previous_target_way_point_ = target_way_point_;
  target_way_point_ = way_point;
}

}  // namespace glocal_exploration
