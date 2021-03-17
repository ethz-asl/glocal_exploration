#ifndef GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
#define GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <glocal_exploration/3rd_party/config_utilities.hpp>
#include <glocal_exploration/planning/global/skeleton/relative_waypoint.h>
#include <glocal_exploration/planning/global/skeleton/skeleton_a_star.h>
#include <glocal_exploration/planning/global/skeleton/skeleton_submap_collection.h>
#include <glocal_exploration/planning/global/submap_frontier_evaluator.h>
#include <glocal_exploration/state/communicator.h>

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
    FloatingPoint centroid_clustering_radius = 1.f;  // m
    bool use_path_verification = true;  // Check traversability in temporal map.
    FloatingPoint path_verification_min_distance = 1.f;  // m
    int goal_search_steps = 5;  // number of grid elements per side of cube.
    FloatingPoint goal_search_step_size = 1.f;  // m, grid element length.
    FloatingPoint safety_distance = 0.f;
    // Maximum number of times the system can replan to the currently chosen
    // frontier before returning to local planning (or choosing a new frontier).
    int max_replan_attempts_to_chosen_frontier = 3;
    FloatingPoint sensor_vertical_fov_rad = 0.5;

    FloatingPoint backtracking_distance_m = 10.f;  // Set to -1 to disable.

    // Frontier evaluator.
    SubmapFrontierEvaluator::Config submap_frontier_config;
    int max_closest_frontier_search_time_sec = 25;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  // Frontier search data collection.
  struct FrontierSearchData {
    Point centroid = Point(0.f, 0.f, 0.f);
    FloatingPoint euclidean_distance = 0.f;
    FloatingPoint path_distance = 0.f;
    int num_points = 0;
    std::vector<Point> frontier_points;
    int clusters = 1;
    std::vector<RelativeWayPoint> way_points;
    enum Reachability {
      kReachable,
      kUnreachable,
      kUnchecked,
      kInvalidGoal
    } reachability;
  };

  // Visualization communication interface
  struct VisualizationData {
    bool frontiers_have_changed = false;
    bool execution_finished = false;
  };

  SkeletonPlanner(const Config& config,
                  const SkeletonAStar::Config& skeleton_a_star_config,
                  std::shared_ptr<Communicator> communicator);
  ~SkeletonPlanner() override = default;

  void executePlanningIteration() override;

  // Visualization access.
  const std::vector<FrontierSearchData>& getFrontierSearchData() const {
    return frontier_data_;
  }
  std::vector<WayPoint> getWayPoints() const;
  VisualizationData& visualizationData() { return vis_data_; }  // mutable

  void addSubmap(cblox::TsdfEsdfSubmap::ConstPtr submap_ptr,
                 const float traversability_radius) {
    skeleton_a_star_.addSubmap(std::move(submap_ptr), traversability_radius);
  }
  const SkeletonSubmapCollection& getSkeletonSubmapCollection() {
    return skeleton_a_star_.getSkeletonSubmapCollection();
  }

  SkeletonAStar::VisualizationEdges getVisualizationEdges() const {
    return skeleton_a_star_.getVisualizationEdges();
  }

 private:
  // Planning iteration methods.
  void resetPlanner();
  bool computeFrontiers();
  bool computeGoalPoint();
  void executeWayPoint();

  // Helper methods.
  bool computePath(const Point& goal,
                   std::vector<RelativeWayPoint>* way_points);
  int num_replan_attempts_to_chosen_frontier_;
  bool is_backtracking_;
  bool searchSkeletonStartVertices(
      Point* start_point, std::vector<GlobalVertexId>* start_vertex_candidates);
  bool computePathToFrontier(
      const Point& start_point,
      const std::vector<GlobalVertexId>& start_vertex_candidates,
      const Point& frontier_centroid, const std::vector<Point>& frontier_points,
      std::vector<RelativeWayPoint>* way_points,
      bool* frontier_is_observable = nullptr);
  bool isFrontierPointObservableFromPosition(
      const Point& frontier_point, const Point& skeleton_vertex_point);
  void clusterFrontiers();
  bool verifyNextWayPoints();

 private:
  const Config config_;

  // Skeleton planner.
  SkeletonAStar skeleton_a_star_;

  // Variables.
  std::vector<RelativeWayPoint> way_points_;  // in mission frame.
  std::vector<FrontierSearchData> frontier_data_;
  VisualizationData vis_data_;

  // Stages of global planning.
  enum class Stage { k1ComputeFrontiers, k2ComputeGoalAndPath, k3ExecutePath };
  Stage stage_;

  // Cached data for feasible goal point lookup. Cube of side length
  // goal_search_step_size * goal_search_steps ordered by distance to center.
  std::vector<Point> goal_search_offsets_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_PLANNER_H_
