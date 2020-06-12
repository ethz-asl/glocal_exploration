#ifndef GLOCAL_EXPLORATION_PLANNING_STATE_MACHINE_H_
#define GLOCAL_EXPLORATION_PLANNING_STATE_MACHINE_H_

#include <string>

#include "glocal_exploration/common.h"

namespace glocal_exploration {
/**
 * The state-machine is maintained by the planner node and passed to other modules, s.t. they can trigger state transitions.
 */
class StateMachine {
 public:
  enum State { SettingUp, Ready, LocalPlanning, GlobalPlanning };

  StateMachine();
  virtual ~StateMachine() = default;

  // access
  const State &currentState() const { return state_; }
  const State &previousState() const { return previous_state_; }
  bool targetIsReached() const { return target_reached_; }
  void setTargetReached(bool target_reached) { target_reached_ = target_reached; }

  // interactions
  void signalReady();
  void signalLocalPlanning();
  void signalGlobalPlanning();

  // utilities
  static std::string stateToString(State state);

 protected:
  State state_;
  State previous_state_;
  bool target_reached_;  // this information is provided by the node and usable for the planners

  void setState(State state);
};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_PLANNING_STATE_MACHINE_H_
