#ifndef GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
#define GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_

#include <memory>
#include <queue>
#include <string>

#include <cblox_planning_global/linked_planning/skeleton/linked_skeleton_planner_ros.h>
#include <ros/ros.h>
#include <3rd_party/config_utilities.hpp>

#include <glocal_exploration/state/communicator.h>

#include "glocal_exploration/planning/global/submap_frontier_evaluator.h"

namespace glocal_exploration {
/**
 * Uses the submap skeleton planner to ind paths to frontiers.
 */
class SkeletonPlanner : public SubmapFrontierEvaluator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    std::string nh_private_namespace = "~";

    // Frontier evaluator.
    SubmapFrontierEvaluator::Config submap_frontier_config;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  SkeletonPlanner(const Config& config,
                  std::shared_ptr<Communicator> communicator);
  ~SkeletonPlanner() override = default;

  void planningIteration() override;

 private:
  const Config config_;

  // Skeleton planner.
  std::unique_ptr<mav_planning::CbloxSkeletonGlobalPlanner> skeleton_planner_;

  // methods
  void resetPlanner();
  bool computeFrontiers();
  bool computeGoalPoint();
  bool computePathToGoal(const Point& goal);

  // variables
  std::queue<Eigen::Vector3d> way_points_;  // in mission frame
  Eigen::Vector3d goal_point_;
  ros::Time tmp_;

  // Stages of global planning.
  enum class Stage {
    k1ComputeFrontiers,
    k2ComputeGoalPoint,
    k3ComputePath,
    k4ExecutePath
  };
  Stage stage_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
