#ifndef GLOCAL_EXPLORATION_ROS_VISUALIZATION_RH_RRT_STAR_VISUALIZER_H_
#define GLOCAL_EXPLORATION_ROS_VISUALIZATION_RH_RRT_STAR_VISUALIZER_H_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "glocal_exploration/planning/local/rh_rrt_star.h"
#include "glocal_exploration_ros/visualization/local_planner_visualizer_base.h"

namespace glocal_exploration {

class RHRRTStarVisualizer : public LocalPlannerVisualizerBase {
 public:
  struct Config {
    std::string nh_namespace = "rh_rrt_star_visualizer";
    bool visualize_gain = true;
    bool visualize_text = true;
    bool visualize_visible_voxels = true;
    bool visualize_value = true;

    [[nodiscard]] bool isValid() const { return true; }[[nodiscard]] Config
        checkValid() const;
  };

  RHRRTStarVisualizer(const Config& config,
                      const std::shared_ptr<Communicator>& communicator);

  void visualize() override;

 private:
  visualization_msgs::Marker visualizeValue(const RHRRTStar::ViewPoint& point,
                                            double min_value, double max_value,
                                            int id);
  visualization_msgs::Marker visualizeGain(const RHRRTStar::ViewPoint& point,
                                           double min_gain, double max_gain,
                                           int id);
  visualization_msgs::Marker visualizeText(const RHRRTStar::ViewPoint& point,
                                           int id);
  visualization_msgs::MarkerArray visualizeVisibleVoxels(
      const RHRRTStar::ViewPoint& point);

 private:
  const Config config_;
  std::shared_ptr<RHRRTStar> planner_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  // variables
  std::string frame_id_;
  ros::Time timestamp_;

  // visualization namespace definitions
  const std::string value_ns_ = "candidate_trajectories";
  const std::string gain_ns_ = "candidate_gains";
  const std::string text_ns_ = "candidate_text";
  const std::string voxel_ns_ = "next_expected_gain";
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_VISUALIZATION_RH_RRT_STAR_VISUALIZER_H_
