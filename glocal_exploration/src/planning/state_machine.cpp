#include "glocal_exploration/planning/state_machine.h"

namespace glocal_exploration {

StateMachine::StateMachine() : state_(SettingUp), previous_state_(SettingUp), target_reached_(false) {}

void StateMachine::signalReady() {
  if (state_ != State::SettingUp) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_) << " to 'Ready'.";
  } else {
    setState(Ready);
  }
}

void StateMachine::signalLocalPlanning() {
  if (state_ == State::SettingUp) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_) << " to 'LocalPlanning'.";
  } else {
    setState(LocalPlanning);
  }
}

void StateMachine::signalGlobalPlanning() {
  if (state_ == State::SettingUp) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_) << " to 'GlobalPlanning'.";
  } else {
    setState(GlobalPlanning);
  }
}

void StateMachine::requestWayPoint(const WayPoint &way_point) {
  new_waypoint_requested_ = true;
  target_way_point_ = way_point;
}

bool StateMachine::getNewWayPointIfRequested(WayPoint *way_point) {
  if (!new_waypoint_requested_) {
    return false;
  }
  CHECK_NOTNULL(way_point);
  *way_point = target_way_point_;
  new_waypoint_requested_ = false;
  return true;
}

void StateMachine::setState(State state) {
  previous_state_ = state_;
  state_ = state;
}

std::string StateMachine::stateToString(State state) {
  switch (state) {
    case SettingUp  : return "SettingUp";
    case Ready  : return "Ready";
    case LocalPlanning  : return "LocalPlanning";
    case GlobalPlanning  : return "GlobalPlanning";
    default: return "UnknownState";
  }
}

} // namespace glocal_exploration

