#ifndef GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
#define GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_

#include <memory>
#include <queue>
#include <string>

#include <cblox_planning_global/linked_planning/skeleton/linked_skeleton_planner.h>
#include <cblox_planning_global/map_interface/cblox_planner.h>
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

    // Skeleton Planner.
    double collision_radius = 1.0;
    bool use_path_shortening = true;
    bool verbose_skeleton_planner = false;
    std::string frame_id = "world";

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
  using SkeletonMap = mav_planning::CbloxPlanner<cblox::SkeletonSubmap>;
  std::unique_ptr<SkeletonMap> skeleton_map_;
  std::unique_ptr<mav_planning::CbloxSkeletonPlanner> skeleton_planner_;

  // methods
  void resetPlanner();
  bool computeGoalPoint();
  bool computePathToGoal();
  void setupSkeletonPlanner();

  // variables
  std::queue<Eigen::Vector3d> way_points_;  // in mission frame
  Eigen::Vector3d goal_point_;
  ros::Time tmp_;

  // Stages of global planning.
  enum class Stage { k1ComputePoint, k2ComputePath, k3ExecutePath };
  Stage stage_ = Stage::k1ComputePoint;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
