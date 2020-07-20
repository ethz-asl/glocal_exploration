#include "glocal_exploration/state/state_machine.h"

#include<string>

namespace glocal_exploration {

StateMachine::StateMachine()
    : state_(kSettingUp), previous_state_(kSettingUp) {}

void StateMachine::signalState(const State& state) {
  switch (state) {
    case kSettingUp:
      LOG(WARNING) << "Can not signal transition to kSettingUp";
    case kReady:
      return signalReady();
    case kLocalPlanning:
      return signalLocalPlanning();
    case kGlobalPlanning:
      return signalGlobalPlanning();
    case kFinished:
      return signalFinished();
  }
}

void StateMachine::signalFinished() {
  if (state_ == State::kSettingUp) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_)
                 << " to 'kFinished'.";
  } else {
    setState(kFinished);
  }
}

void StateMachine::signalReady() {
  if (state_ != State::kSettingUp) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_)
                 << " to 'kReady'.";
  } else {
    setState(kReady);
  }
}

void StateMachine::signalLocalPlanning() {
  if (state_ == State::kSettingUp || state_ == kFinished) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_)
                 << " to 'LocalPlanning'.";
  } else {
    setState(kLocalPlanning);
  }
}

void StateMachine::signalGlobalPlanning() {
  if (state_ == State::kSettingUp || state_ == kFinished) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_)
                 << " to 'GlobalPlanning'.";
  } else {
    setState(kGlobalPlanning);
  }
}

void StateMachine::setState(State state) {
  previous_state_ = state_;
  state_ = state;
}

std::string StateMachine::stateToString(State state) {
  switch (state) {
    case kSettingUp:
      return "SettingUp";
    case kReady:
      return "Ready";
    case kLocalPlanning:
      return "LocalPlanning";
    case kGlobalPlanning:
      return "GlobalPlanning";
    case kFinished:
      return "Finished";
    default:
      return "UnknownState";
  }
}

}  // namespace glocal_exploration
