#ifndef GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
#define GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cblox_planning_global/linked_planning/skeleton/linked_skeleton_planner_ros.h>
#include <ros/ros.h>

#include <glocal_exploration/state/communicator.h>
#include <glocal_exploration/3rd_party/config_utilities.hpp>

#include "glocal_exploration/planning/global/submap_frontier_evaluator.h"

namespace glocal_exploration {
/**
 * Uses the submap skeleton planner to find paths to frontiers.
 */
class SkeletonPlanner : public SubmapFrontierEvaluator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    std::string nh_private_namespace = "~/SkeletonPlanner";
    bool use_centroid_clustering = false;
    double centroid_clustering_radius = 1.0;  // m
    bool use_path_verification = true;  // Check traversability in temporal map.
    double path_verification_min_distance = 1.0;  // m
    int goal_search_steps = 5;  // number of grid elements per side of cube.
    double goal_search_step_size = 1.0;  // m, grid element length.

    // Frontier evaluator.
    SubmapFrontierEvaluator::Config submap_frontier_config;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  struct VisualizationInfo {
    bool frontiers_changed = false;
    enum Reachability { kReachable, kUnreachable, kUnchecked, kInvalidGoal };
    std::vector<std::pair<Reachability, Point>> goal_points;
    bool goals_changed = false;
  };

  SkeletonPlanner(const Config& config,
                  std::shared_ptr<Communicator> communicator);
  ~SkeletonPlanner() override = default;

  void executePlanningIteration() override;

  // Visualization access.
  VisualizationInfo& visualizationInfo() { return visualization_info_; }
  const std::vector<WayPoint>& getWayPoints() const { return way_points_; }

 private:
  // Frontier search data collection.
  struct FrontierSearchData {
    Point centroid;
    double euclidean_distance = 0;
    double path_distance = 0;
    int num_points = 0;
    std::vector<WayPoint> way_points;
  };

  // Planning iteration methods.
  void resetPlanner();
  bool computeFrontiers();
  bool computeGoalPoint();
  void executeWayPoint();

  // Helper methods.
  bool computePath(const Point& goal, std::vector<WayPoint>* way_points);
  bool findValidGoalPoint(Point* goal);  // Changes goal to the new point.
  void clusterFrontiers(std::vector<FrontierSearchData>* frontiers) const;
  bool lineIsIntraversableInSlidingWindowAt(Point* goal_point);
  bool verifyNextWayPoints();

 private:
  const Config config_;

  // Skeleton planner.
  std::unique_ptr<mav_planning::CbloxSkeletonGlobalPlanner> skeleton_planner_;

  // Variables.
  std::vector<WayPoint> way_points_;  // in mission frame
  Eigen::Vector3d goal_point_;

  // Stages of global planning.
  enum class Stage { k1ComputeFrontiers, k2ComputeGoalAndPath, k3ExecutePath };
  Stage stage_;

  // Visualization
  VisualizationInfo visualization_info_;

  // Cached data for feasible goal point lookup. Cube of side length
  // goal_search_step_size * goal_search_steps ordered by distance to center.
  std::vector<Point> goal_search_offsets_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
