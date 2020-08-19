#ifndef GLOCAL_EXPLORATION_ROS_VISUALIZATION_SKELETON_VISUALIZER_H_
#define GLOCAL_EXPLORATION_ROS_VISUALIZATION_SKELETON_VISUALIZER_H_

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/core/common.h>
#include <3rd_party/config_utilities.hpp>

#include "glocal_exploration_ros/planning/global/skeleton_planner.h"
#include "glocal_exploration_ros/visualization/global_planner_visualizer_base.h"

namespace glocal_exploration {

class SkeletonVisualizer : public GlobalPlannerVisualizerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    std::string nh_namespace = "skeleton_planner_visualizer";
    bool visualize_frontiers = true;
    bool visualize_inactive_frontiers = false;
    int n_frontier_colors = 20;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
  };

  SkeletonVisualizer(const Config& config,
                     const std::shared_ptr<Communicator>& communicator);
  ~SkeletonVisualizer() override = default;

  void visualize() override;

 private:
  visualization_msgs::MarkerArray visualizeFrontier(const Frontier& frontier,
                                                    unsigned int* id);

 private:
  const Config config_;
  std::shared_ptr<SkeletonPlanner> planner_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  // tracking
  int frontier_msg_id_;

  // Settings
  std::vector<voxblox::Color> color_list_;
  const std::string frontier_ns = "frontiers";
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_VISUALIZATION_SKELETON_VISUALIZER_H_
