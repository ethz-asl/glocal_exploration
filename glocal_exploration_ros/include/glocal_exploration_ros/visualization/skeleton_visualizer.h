#ifndef GLOCAL_EXPLORATION_ROS_VISUALIZATION_SKELETON_VISUALIZER_H_
#define GLOCAL_EXPLORATION_ROS_VISUALIZATION_SKELETON_VISUALIZER_H_

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/common.h>

#include <glocal_exploration/3rd_party/config_utilities.hpp>

#include "glocal_exploration_ros/planning/global/skeleton_planner.h"
#include "glocal_exploration_ros/visualization/global_planner_visualizer_base.h"

namespace glocal_exploration {

class SkeletonVisualizer : public GlobalPlannerVisualizerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    std::string nh_namespace = "skeleton_planner_visualizer";
    bool visualize_frontiers = true;
    bool visualize_frontier_text = true;
    bool visualize_executed_path = true;
    bool visualize_candidate_goals = true;
    bool visualize_planned_path = true;
    bool visualize_path_search = true;
    bool visualize_inactive_frontiers = true;
    bool visualize_skeleton_submaps = true;
    bool keep_visualizations = false;  // Keep even after finishing paths.

    Config();
    void checkParams() const override;
    void fromRosParam() override;
  };

  SkeletonVisualizer(const Config& config,
                     const std::shared_ptr<Communicator>& communicator);
  ~SkeletonVisualizer() override = default;

  void visualize() override;

  // Visualization tasks.
  void visualizeExecutedPath();
  void visualizeFrontiers();
  void visualizeInactiveFrontiers();
  void visualizeFrontierText();
  void visualizePlannedPath();
  void visualizePathSearch();
  void visualizeGoalPoints();
  void visualizeSkeletonSubmaps();

 private:
  std::string frontierTextFormat(FloatingPoint value) const;
  bool getVertexPositionMsg(const GlobalVertexId& global_vertex_id,
                            geometry_msgs::Point* position_msg);

 private:
  const Config config_;
  std::shared_ptr<SkeletonPlanner> planner_;
  ros::NodeHandle nh_;
  ros::Publisher frontier_pub_;
  ros::Publisher executed_path_pub_;
  ros::Publisher planned_path_pub_;
  ros::Publisher path_search_pub_;
  ros::Publisher goals_pub_;
  ros::Publisher frontier_text_pub_;
  ros::Publisher inactive_frontiers_pub_;
  ros::Publisher skeleton_submaps_pub_;

  // Tracking.
  int executed_path_id_ = 0;

  // Settings.
  const std::string frame_id_ = "mission";
  const int queue_size_ = 100;
  const ros::Duration failed_timeout_ = ros::Duration(10.0);  // s
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_VISUALIZATION_SKELETON_VISUALIZER_H_
