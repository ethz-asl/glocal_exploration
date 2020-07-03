#ifndef GLOCAL_EXPLORATION_PLANNING_STATE_MACHINE_H_
#define GLOCAL_EXPLORATION_PLANNING_STATE_MACHINE_H_

#include <string>

#include "glocal_exploration/common.h"
#include "glocal_exploration/planning/waypoint.h"

namespace glocal_exploration {
/**
 * The state-machine is maintained by the planner node and passed to other modules, s.t. they can trigger state transitions.
 */
class StateMachine {
 public:
  enum State { SettingUp, Ready, LocalPlanning, GlobalPlanning, Finished };

  StateMachine();
  virtual ~StateMachine() = default;

  // access
  const State &currentState() const { return state_; }
  const State &previousState() const { return previous_state_; }
  bool targetIsReached() const { return target_reached_; }
  const WayPoint& currentPose() const { return current_pose_; }

  // interactions
  void signalReady();
  void signalLocalPlanning();
  void signalGlobalPlanning();
  void requestWayPoint(const WayPoint& way_point);

  // interface for the main node to manage the State machine
  void setTargetReached(bool target_reached) { target_reached_ = target_reached; }
  void setCurrentPose(const WayPoint &pose) { current_pose_ = pose; }
  bool getNewWayPointIfRequested(WayPoint * way_point);

  // utilities
  static std::string stateToString(State state);

 protected:
  State state_;
  State previous_state_;
  bool target_reached_;  // this information is provided by the node and usable for the planners
  WayPoint current_pose_;
  WayPoint target_way_point_;
  bool new_waypoint_requested_;

  // methods
  void setState(State state);
};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_PLANNING_STATE_MACHINE_H_
