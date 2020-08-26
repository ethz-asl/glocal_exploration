#ifndef GLOCAL_EXPLORATION_STATE_STATE_MACHINE_H_
#define GLOCAL_EXPLORATION_STATE_STATE_MACHINE_H_

#include <memory>
#include <string>

#include <boost/shared_ptr.hpp>

#include "glocal_exploration/common.h"
#include "glocal_exploration/state/waypoint.h"

namespace glocal_exploration {
/**
 * The state-machine is maintained by the planner node and passed to other
 * modules, s.t. they can trigger state transitions.
 */
class StateMachine {
 public:
  enum class State {
    kSettingUp,
    kReady,
    kLocalPlanning,
    kGlobalPlanning,
    kFinished
  };

  StateMachine();
  virtual ~StateMachine() = default;

  // access
  const State& currentState() const { return state_; }
  const State& previousState() const { return previous_state_; }

  // interactions
  void signalReady();
  void signalLocalPlanning();
  void signalGlobalPlanning();
  void signalFinished();
  void signalState(const State& state);

  // utilities
  static std::string stateToString(State state);

 protected:
  State state_;
  State previous_state_;

  // methods
  void setState(State state);
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_STATE_STATE_MACHINE_H_
