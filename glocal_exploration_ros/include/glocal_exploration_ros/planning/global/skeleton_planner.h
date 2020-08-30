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
    bool use_frontier_clustering = false;
    double frontier_clustering_radius = 1.0;  // m

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
  const Config config_;

  // Skeleton planner.
  std::unique_ptr<mav_planning::CbloxSkeletonGlobalPlanner> skeleton_planner_;

  // Planning iteration methods.
  void resetPlanner();
  bool computeFrontiers();
  bool computeGoalPoint();
  void executeWayPoint();

  // Helper methods.
  bool computePath(const Point& goal, std::vector<WayPoint>* way_points);
  bool findValidGoalPoint(Point* goal);  // Changes goal to the new point.

  // Variables.
  std::vector<WayPoint> way_points_;  // in mission frame
  Eigen::Vector3d goal_point_;

  // Stages of global planning.
  enum class Stage { k1ComputeFrontiers, k2ComputeGoalAndPath, k3ExecutePath };
  Stage stage_;

  // Frontier search
  struct FrontierSearchData {
    Point centroid;
    double euclidean_distance = 0;
    double path_distance = 0;
    int num_points = 0;
    std::vector<WayPoint> way_points;
  };

  // Visualization
  VisualizationInfo visualization_info_;

  // Cached data for feasible goal point lookup. Cube of side lenth 4 ordered by
  // distance to center.
  const double kNeighborStepSize_ = 1.0;  // m
  const std::vector<Point> kNeighborOffsets_{
      Point(0, 0, 0),   Point(-1, 0, 0),   Point(0, -1, 0),  Point(0, 0, -1),
      Point(0, 0, 1),   Point(0, 1, 0),    Point(1, 0, 0),   Point(-1, -1, 0),
      Point(-1, 0, -1), Point(-1, 0, 1),   Point(-1, 1, 0),  Point(0, -1, -1),
      Point(0, -1, 1),  Point(0, 1, -1),   Point(0, 1, 1),   Point(1, -1, 0),
      Point(1, 0, -1),  Point(1, 0, 1),    Point(1, 1, 0),   Point(-1, -1, -1),
      Point(-1, -1, 1), Point(-1, 1, -1),  Point(-1, 1, 1),  Point(1, -1, -1),
      Point(1, -1, 1),  Point(1, 1, -1),   Point(1, 1, 1),   Point(-2, 0, 0),
      Point(0, -2, 0),  Point(0, 0, -2),   Point(0, 0, 2),   Point(0, 2, 0),
      Point(2, 0, 0),   Point(-2, -1, 0),  Point(-2, 0, -1), Point(-2, 0, 1),
      Point(-2, 1, 0),  Point(-1, -2, 0),  Point(-1, 0, -2), Point(-1, 0, 2),
      Point(-1, 2, 0),  Point(0, -2, -1),  Point(0, -2, 1),  Point(0, -1, -2),
      Point(0, -1, 2),  Point(0, 1, -2),   Point(0, 1, 2),   Point(0, 2, -1),
      Point(0, 2, 1),   Point(1, -2, 0),   Point(1, 0, -2),  Point(1, 0, 2),
      Point(1, 2, 0),   Point(2, -1, 0),   Point(2, 0, -1),  Point(2, 0, 1),
      Point(2, 1, 0),   Point(-2, -1, -1), Point(-2, -1, 1), Point(-2, 1, -1),
      Point(-2, 1, 1),  Point(-1, -2, -1), Point(-1, -2, 1), Point(-1, -1, -2),
      Point(-1, -1, 2), Point(-1, 1, -2),  Point(-1, 1, 2),  Point(-1, 2, -1),
      Point(-1, 2, 1),  Point(1, -2, -1),  Point(1, -2, 1),  Point(1, -1, -2),
      Point(1, -1, 2),  Point(1, 1, -2),   Point(1, 1, 2),   Point(1, 2, -1),
      Point(1, 2, 1),   Point(2, -1, -1),  Point(2, -1, 1),  Point(2, 1, -1),
      Point(2, 1, 1),   Point(-2, -2, 0),  Point(-2, 0, -2), Point(-2, 0, 2),
      Point(-2, 2, 0),  Point(0, -2, -2),  Point(0, -2, 2),  Point(0, 2, -2),
      Point(0, 2, 2),   Point(2, -2, 0),   Point(2, 0, -2),  Point(2, 0, 2),
      Point(2, 2, 0),   Point(-2, -2, -1), Point(-2, -2, 1), Point(-2, -1, -2),
      Point(-2, -1, 2), Point(-2, 1, -2),  Point(-2, 1, 2),  Point(-2, 2, -1),
      Point(-2, 2, 1),  Point(-1, -2, -2), Point(-1, -2, 2), Point(-1, 2, -2),
      Point(-1, 2, 2),  Point(1, -2, -2),  Point(1, -2, 2),  Point(1, 2, -2),
      Point(1, 2, 2),   Point(2, -2, -1),  Point(2, -2, 1),  Point(2, -1, -2),
      Point(2, -1, 2),  Point(2, 1, -2),   Point(2, 1, 2),   Point(2, 2, -1),
      Point(2, 2, 1),   Point(-2, -2, -2), Point(-2, -2, 2), Point(-2, 2, -2),
      Point(-2, 2, 2),  Point(2, -2, -2),  Point(2, -2, 2),  Point(2, 2, -2),
      Point(2, 2, 2),
  };
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
