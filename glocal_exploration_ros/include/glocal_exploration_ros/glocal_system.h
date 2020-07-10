#ifndef GLOCAL_EXPLORATION_ROS_GLOCAL_SYSTEM_H_
#define GLOCAL_EXPLORATION_ROS_GLOCAL_SYSTEM_H_

#include <memory>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include "glocal_exploration/mapping/map_base.h"
#include "glocal_exploration/planning/local/local_planner_base.h"
#include "glocal_exploration/planning/state_machine.h"
#include "glocal_exploration_ros/visualization/local_planner_visualizer_base.h"

namespace glocal_exploration {

class GlocalSystem {
 public:
  struct Config {
    double replan_position_threshold = 0.2;  // m
    double replan_yaw_threshold = 10;        // deg
    bool republish_waypoints = false;
  };

  // Constructor
  GlocalSystem(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~GlocalSystem() = default;

  // ROS callbacks
  void odomCallback(const nav_msgs::Odometry& msg);
  bool runSrvCallback(std_srvs::SetBool::Request& req,    // NOLINT
                      std_srvs::SetBool::Response& res);  // NOLINT

  // spinning is managed explicitly, run this to start the planner
  void mainLoop();

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers and publishers
  ros::Subscriber odom_sub_;
  ros::Publisher target_pub_;
  ros::ServiceServer run_srv_;

  // Components
  Config config_;
  const std::shared_ptr<StateMachine> state_machine_;
  std::shared_ptr<MapBase> map_;
  std::shared_ptr<LocalPlannerBase> local_planner_;
  std::shared_ptr<LocalPlannerVisualizerBase> local_planner_visualizer_;

  // methods
  void loopIteration();
  void readParamsFromRos();
  void publishTargetPose();

  // variables
  Eigen::Vector3d
      current_position_;  // current and goal poses are in odom frame
  Eigen::Quaterniond current_orientation_;
  Eigen::Vector3d target_position_;
  double target_yaw_;                  // rad
  Eigen::Vector3d previous_position_;  // to track whether the drone is moving
  double previous_yaw_;
  ros::Time previous_time_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_GLOCAL_SYSTEM_H_
