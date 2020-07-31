#ifndef GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
#define GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_

#include <queue>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <glocal_exploration/state/communicator.h>

#include "glocal_exploration/planning/global/submap_frontiers.h"

namespace glocal_exploration {
/**
 * Uses the submap skeleton planner to ind paths to frontiers.
 */
class SkeletonPlanner : public SubmapFrontiers {
 public:
  // Defines a baseclass for map configurations
  struct Config {
    std::string nh_namespace = "skeleton_global_planner";
    std::string service_name;

    SubmapFrontiers::Config submap_frontier_config;

    [[nodiscard]] bool isValid() const;
    [[nodiscard]] Config checkValid() const;
  };
  SkeletonPlanner(const Config& config,
                  std::shared_ptr<Communicator> communicator);
  ~SkeletonPlanner() override = default;

  /* General and Accessors */
  void planningIteration() override;

 protected:
  const Config config_;

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

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
