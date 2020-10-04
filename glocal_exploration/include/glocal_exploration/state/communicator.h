#ifndef GLOCAL_EXPLORATION_STATE_COMMUNICATOR_H_
#define GLOCAL_EXPLORATION_STATE_COMMUNICATOR_H_

#include <memory>

#include "glocal_exploration/common.h"
#include "glocal_exploration/mapping/map_base.h"
#include "glocal_exploration/planning/global/global_planner_base.h"
#include "glocal_exploration/planning/local/local_planner_base.h"
#include "glocal_exploration/state/region_of_interest.h"
#include "glocal_exploration/state/state_machine.h"
#include "glocal_exploration/state/waypoint.h"

namespace glocal_exploration {

/**
 * The communicator is the high-level instance that owns all components, s.t.
 * they can interact where necessary.
 */
class Communicator {
 public:
  Communicator();
  virtual ~Communicator() = default;

  // general information accessors
  bool targetIsReached() const { return target_reached_; }
  const WayPoint& currentPose() const { return current_pose_; }
  bool newWayPointIsRequested() const { return new_waypoint_requested_; }

  // componet accessors
  const std::shared_ptr<StateMachine>& stateMachine() const {
    return state_machine_;
  }
  const std::shared_ptr<RegionOfInterest>& regionOfInterest() const {
    return roi_;
  }
  const std::shared_ptr<MapBase>& map() const { return map_; }
  const std::shared_ptr<LocalPlannerBase>& localPlanner() const {
    return local_planner_;
  }
  void resetLocalPlanner(const WayPoint& new_origin) {
    local_planner_->resetPlanner(new_origin);
  }
  const std::shared_ptr<GlobalPlannerBase>& globalPlanner() const {
    return global_planner_;
  }

  // requests
  void requestWayPoint(const WayPoint& way_point);
  const WayPoint& getRequestedWayPoint() { return target_way_point_; }
  const WayPoint& getPreviousWayPoint() { return previous_target_way_point_; }

  // interface for the main node to manage the state
  void setTargetReached(bool target_reached) {
    target_reached_ = target_reached;
  }
  void setCurrentPose(const WayPoint& pose) { current_pose_ = pose; }
  void setRequestedWayPointRead() { new_waypoint_requested_ = false; }

  // setup tools for the main node
  void setupStateMachine(std::shared_ptr<StateMachine> state_machine);
  void setupMap(std::shared_ptr<MapBase> map);
  void setupLocalPlanner(std::shared_ptr<LocalPlannerBase> local_planner);
  void setupGlobalPlanner(std::shared_ptr<GlobalPlannerBase> global_planner);
  void setupRegionOfInterest(std::shared_ptr<RegionOfInterest> roi);

 protected:
  // components
  std::shared_ptr<StateMachine> state_machine_;
  std::shared_ptr<MapBase> map_;
  std::shared_ptr<LocalPlannerBase> local_planner_;
  std::shared_ptr<GlobalPlannerBase> global_planner_;
  std::shared_ptr<RegionOfInterest> roi_;

  // General information is provided by the node and usable by the planners
  bool target_reached_;
  WayPoint current_pose_;
  WayPoint target_way_point_;
  WayPoint previous_target_way_point_;
  bool new_waypoint_requested_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_STATE_COMMUNICATOR_H_
