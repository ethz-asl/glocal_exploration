#ifndef GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
#define GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_

#include <memory>
#include <string>
#include <vector>

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

  // Planning iteration methods.
  void resetPlanner();
  bool computeFrontiers();
  bool computeGoalPoint();
  void executeWayPoint();

  // Helper methods
  bool computePath(const Point& goal, std::vector<WayPoint>* way_points);
  bool findValidGoalPoint(Point* goal);  // Changes goal to the new point.

  // variables
  std::vector<WayPoint> way_points_;  // in mission frame
  Eigen::Vector3d goal_point_;

  // cached data for feasible goal point lookup. Cube of side lenth 4 ordered by
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

  // Stages of global planning.
  enum class Stage { k1ComputeFrontiers, k2ComputeGoalAndPath, k3ExecutePath };
  Stage stage_;

  // Frontier search
  struct FrontierSearchData {
    Point centroid;
    double euclidian_distance;
    double path_distance;
    std::vector<WayPoint> way_points;
  };
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
