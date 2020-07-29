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

void Communicator::setupRegionOfInterest(
    std::shared_ptr<RegionOfInterest> roi) {
  roi_ = std::move(roi);
}

void Communicator::requestWayPoint(const WayPoint& way_point) {
  new_waypoint_requested_ = true;
  target_way_point_ = way_point;
}

bool Communicator::getNewWayPointIfRequested(WayPoint* way_point) {
  if (!new_waypoint_requested_) {
    return false;
  }
  CHECK_NOTNULL(way_point);
  *way_point = target_way_point_;
  new_waypoint_requested_ = false;
  return true;
}

}  // namespace glocal_exploration
