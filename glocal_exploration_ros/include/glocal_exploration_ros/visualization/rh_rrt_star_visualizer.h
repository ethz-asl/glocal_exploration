#ifndef GLOCAL_EXPLORATION_ROS_VISUALIZATION_RH_RRT_STAR_VISUALIZER_H_
#define GLOCAL_EXPLORATION_ROS_VISUALIZATION_RH_RRT_STAR_VISUALIZER_H_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <glocal_exploration/3rd_party/config_utilities.hpp>
#include <glocal_exploration/planning/local/rh_rrt_star.h>

#include "glocal_exploration_ros/visualization/local_planner_visualizer_base.h"

namespace glocal_exploration {

class RHRRTStarVisualizer : public LocalPlannerVisualizerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    std::string nh_namespace = "rh_rrt_star_visualizer";
    bool visualize_tree = true;
    bool visualize_gain = true;
    bool visualize_text = true;
    bool visualize_visible_voxels = true;
    bool visualize_executed_path = true;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
  };

  RHRRTStarVisualizer(const Config& config,
                      const std::shared_ptr<Communicator>& communicator);

  void visualize() override;

 private:
  void visualizeValue(const RHRRTStar::ViewPoint& point,
                      FloatingPoint min_value, FloatingPoint max_value, int id);
  void visualizeGain(const RHRRTStar::ViewPoint& point, FloatingPoint min_gain,
                     FloatingPoint max_gain, int id);
  void visualizeText(const RHRRTStar::ViewPoint& point, int id);
  void visualizeVisibleVoxels(const RHRRTStar::ViewPoint& point);

 private:
  const Config config_;
  std::shared_ptr<RHRRTStar> planner_;
  ros::NodeHandle nh_;
  ros::Publisher gain_pub_;
  ros::Publisher value_pub_;
  ros::Publisher text_pub_;
  ros::Publisher voxel_pub_;
  ros::Publisher path_pub_;

  // Variables.
  const std::string frame_id_ = "mission";
  ros::Time timestamp_;
  int executed_path_id_ = 0;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_VISUALIZATION_RH_RRT_STAR_VISUALIZER_H_
