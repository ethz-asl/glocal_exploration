#include "glocal_exploration_ros/planners/glocal_planner.h"

#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>

#include "glocal_exploration_ros/conversions/ros_component_factory.h"

namespace glocal_exploration {

GlocalPlanner::GlocalPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {

  // setup the map
  ros::NodeHandle nh_mapping(nh_private_, "mapping");
  map_ = ComponentFactoryROS::createMap(nh_mapping);

  // setup the local planner
  ros::NodeHandle nh_local_planner(nh_private_, "local_planner");
  local_planner_ = ComponentFactoryROS::createLocalPlanner(nh_local_planner, map_);

  // params
  nh_private_.param("replan_position_threshold", config_.replan_position_threshold, config_.replan_position_threshold);
  nh_private_.param("replan_yaw_threshold", config_.replan_yaw_threshold, config_.replan_yaw_threshold);
  CHECK_GE(config_.replan_position_threshold, 0) << "Param 'replan_position_threshold' expected >= 0";
  CHECK_GE(config_.replan_yaw_threshold, 0) << "Param 'replan_yaw_threshold' expected >= 0";

  // ROS
  target_pub_ = nh_.advertise<geometry_msgs::Pose>("command/pose", 10);
  odom_sub_ = nh_.subscribe("odometry", 1, &GlocalPlanner::odomCallback, this);
}

void GlocalPlanner::planningLoop() {
  // This is the main loop, spinning is managed explicitely for efficiency

  // starting the main loop means everything is setup
  VLOG(1) << "Glocal Exploration Planner set up successfully.";
  state_machine_.signalReady();
  run_srv_ = nh_private_.advertiseService("toggle_running", &GlocalPlanner::runSrvCallback, this);

  while (ros::ok()) {
    if (state_machine_.currentState() != StateMachine::Ready) {
      loopIteration();
    }
    ros::spinOnce();
  }
  VLOG(1) << "Glocal Exploration Planner finished planning.";
}

void GlocalPlanner::loopIteration() {
  switch (state_machine_.currentState()){
    case StateMachine::LocalPlanning: local_planner_->planningIteration(state_machine_); break;
  }
}

void GlocalPlanner::odomCallback(const nav_msgs::Odometry &msg) {
  // Track the current pose
  current_position_ =
      Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z);
  current_orientation_ = Eigen::Quaterniond(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

  // Check whether the goal pose is reached
  if (!state_machine_.targetIsReached()) {
    // check position
    if ((target_position_ - current_position_).norm() <= config_.replan_position_threshold) {
      // check yaw
      double yaw_diff = target_yaw_ - tf2::getYaw(msg.pose.pose.orientation);
      if (yaw_diff < 0){
        yaw_diff += 2.0 * M_PI;
      } if (yaw_diff > M_PI){
         yaw_diff = 2.0 * M_PI - yaw_diff;
       }
      if (yaw_diff <= config_.replan_yaw_threshold * M_PI / 180.0) {
        state_machine_.setTargetReached(true);
      }
    }
  }
}

bool GlocalPlanner::runSrvCallback(std_srvs::SetBool::Request &req,  std_srvs::SetBool::Response &res) {
  state_machine_.signalLocalPlanning();
  if (state_machine_.currentState() == StateMachine::LocalPlanning) {
    VLOG(1) << "Started Glocal Exploration.";
    res.success = true;
  } else {
    res.success = false;
  }
  return true;
}

} // namespace glocal_exploration