#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_PLANNER_SKELETON_PLANNER_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_PLANNER_SKELETON_PLANNER_H_

#include <queue>

#include <ros/ros.h>

#include "glocal_exploration/planning/global_planner/global_planner_base.h"


namespace glocal_exploration {
/**
 * Defines the interface of a global planner.
 */
class SkeletonPlanner : public GlobalPlannerBase {
 public:
  // Defines a baseclass for map configurations
  struct Config : GlobalPlannerBase::Config {
    std::string nh_namespace = "skeleton_global_planner";
    std::string servce_name;
  };
  SkeletonPlanner(std::shared_ptr<MapBase> map, std::shared_ptr<StateMachine> state_machine);
  virtual ~SkeletonPlanner() = default;

  /* Setup */
  // Can pass derived configs here by base pointer to setup the map.
  virtual bool setupFromConfig(GlobalPlannerBase::Config *config) override;

  /* General and Accessors */
  void planningIteration() override;

 protected:
  Config config_;

  // Interface to planner node
  ros::NodeHandle nh_;
  ros::ServiceClient skeleton_planner_srv_;
  ros::ServiceClient publish_global_trajectory_cln_;
  ros::ServiceServer publish_global_trajectory_srv_;
  ros::Subscriber trajectory_sub_;

  // methods
  void resetPlanner();
  bool computeGoalPoint();
  bool computePathToGoal();

  // variables
  std::queue<Eigen::Vector3d> way_points_;    // in mission frame
  int stage_;   // at which part of global planning the system is right now
  Eigen::Vector3d goal_point_;


};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_PLANNING_GLOBAL_PLANNER_SKELETON_PLANNER_H_
