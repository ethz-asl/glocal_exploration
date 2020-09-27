#ifndef GLOCAL_EXPLORATION_ROS_GLOCAL_SYSTEM_H_
#define GLOCAL_EXPLORATION_ROS_GLOCAL_SYSTEM_H_

#include <memory>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <glocal_exploration/state/communicator.h>
#include <glocal_exploration/3rd_party/config_utilities.hpp>

#include "glocal_exploration_ros/visualization/global_planner_visualizer_base.h"
#include "glocal_exploration_ros/visualization/local_planner_visualizer_base.h"

namespace glocal_exploration {

class GlocalSystem {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    FloatingPoint replan_position_threshold = 0.2f;  // m
    FloatingPoint replan_yaw_threshold = 10.f;       // deg
    FloatingPoint replan_timeout_constant = 0.f;     // s, wait this long always
    // add timeout time per distance
    FloatingPoint replan_timeout_velocity = 0.f;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  // Constructors.
  GlocalSystem(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  GlocalSystem(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
               const Config& config);
  virtual ~GlocalSystem() = default;

  // ROS callbacks.
  void odomCallback(const nav_msgs::Odometry& msg);
  bool runSrvCallback(std_srvs::SetBool::Request& req,    // NOLINT
                      std_srvs::SetBool::Response& res);  // NOLINT

  // Spinning is managed explicitly, run this to start the planner.
  void mainLoop();

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers and publishers.
  ros::Subscriber odom_sub_;
  ros::Publisher target_pub_;
  ros::ServiceServer run_srv_;

  // Components.
  const Config config_;
  std::shared_ptr<Communicator> comm_;
  std::shared_ptr<LocalPlannerVisualizerBase> local_planner_visualizer_;
  std::shared_ptr<GlobalPlannerVisualizerBase> global_planner_visualizer_;

  // Methods.
  void buildComponents(const ros::NodeHandle& nh);
  bool startExploration();
  void loopIteration();
  void publishTargetPose();

  // Variables.
  Point current_position_;  // current/goal poses are in odom frame.
  Eigen::Quaterniond current_orientation_;
  Point target_position_;
  FloatingPoint target_yaw_;             // rad
  FloatingPoint last_waypoint_timeout_;  // s
  ros::Time last_waypoint_published_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_GLOCAL_SYSTEM_H_
