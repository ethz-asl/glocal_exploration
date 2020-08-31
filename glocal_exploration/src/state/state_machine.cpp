#include "glocal_exploration/state/state_machine.h"

#include <string>

namespace glocal_exploration {

StateMachine::StateMachine()
    : state_(State::kSettingUp), previous_state_(State::kSettingUp) {}

void StateMachine::signalState(const State& state) {
  switch (state) {
    case State::kSettingUp:
      LOG(WARNING) << "Can not signal transition to kSettingUp";
    case State::kReady:
      return signalReady();
    case State::kLocalPlanning:
      return signalLocalPlanning();
    case State::kGlobalPlanning:
      return signalGlobalPlanning();
    case State::kFinished:
      return signalFinished();
  }
}

void StateMachine::signalFinished() {
  if (state_ == State::kSettingUp || state_ == State::kReady) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_)
                 << " to 'kFinished'.";
  } else {
    setState(State::kFinished);
  }
}

void StateMachine::signalReady() {
  if (state_ != State::kSettingUp) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_)
                 << " to 'kReady'.";
  } else {
    setState(State::kReady);
  }
}

void StateMachine::signalLocalPlanning() {
  if (state_ == State::kSettingUp || state_ == State::kFinished) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_)
                 << " to 'LocalPlanning'.";
  } else {
    setState(State::kLocalPlanning);
  }
}

void StateMachine::signalGlobalPlanning() {
  if (state_ == State::kSettingUp || state_ == State::kFinished) {
    LOG(WARNING) << "Can not transition from '" << stateToString(state_)
                 << " to 'GlobalPlanning'.";
  } else {
    setState(State::kGlobalPlanning);
  }
}

void StateMachine::setState(State state) {
  previous_state_ = state_;
  state_ = state;
}

std::string StateMachine::stateToString(State state) {
  switch (state) {
    case State::kSettingUp:
      return "SettingUp";
    case State::kReady:
      return "Ready";
    case State::kLocalPlanning:
      return "LocalPlanning";
    case State::kGlobalPlanning:
      return "GlobalPlanning";
    case State::kFinished:
      return "Finished";
    default:
      return "UnknownState";
  }
}

}  // namespace glocal_exploration
