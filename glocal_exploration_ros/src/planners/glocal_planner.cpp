#include "glocal_exploration_ros/planners/glocal_planner.h"

#include <geometry_msgs/Pose.h>
#include <glocal_exploration_ros/conversions/ros_params.h>
#include <tf2/utils.h>

#include "glocal_exploration_ros/conversions/ros_component_factory.h"

namespace glocal_exploration {

GlocalPlanner::GlocalPlanner(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), state_machine_(new StateMachine()) {
  // params
  readParamsFromRos();

  // setup the region of interest
  ros::NodeHandle nh_roi(nh_private_, "region_of_interest");
  std::shared_ptr<RegionOfInterest> roi =
      ComponentFactoryROS::createRegionOfInterest(nh_roi);
  state_machine_->setROI(roi);

  // setup the map
  ros::NodeHandle nh_mapping(nh_private_, "mapping");
  map_ = ComponentFactoryROS::createMap(nh_mapping, state_machine_);

  // setup the local planner
  ros::NodeHandle nh_local_planner(nh_private_, "local_planner");
  local_planner_ = ComponentFactoryROS::createLocalPlanner(
      nh_local_planner, map_, state_machine_);
  local_planner_visualizer_ = ComponentFactoryROS::createLocalPlannerVisualizer(
      nh_local_planner, local_planner_);

  // ROS
  target_pub_ = nh_.advertise<geometry_msgs::Pose>("command/pose", 10);
  odom_sub_ = nh_.subscribe("odometry", 1, &GlocalPlanner::odomCallback, this);
}

void GlocalPlanner::readParamsFromRos() {
  nh_private_.param("replan_position_threshold",
                    config_.replan_position_threshold,
                    config_.replan_position_threshold);
  nh_private_.param("replan_yaw_threshold", config_.replan_yaw_threshold,
                    config_.replan_yaw_threshold);
  nh_private_.param("republish_waypoints", config_.republish_waypoints,
                    config_.republish_waypoints);
  CHECK_GT(config_.replan_position_threshold, 0)
      << "Param 'replan_position_threshold' expected > 0";
  CHECK_GT(config_.replan_yaw_threshold, 0)
      << "Param 'replan_yaw_threshold' expected > 0";
}

void GlocalPlanner::planningLoop() {
  // This is the main loop, spinning is managed explicitely for efficiency
  // starting the main loop means everything is setup
  VLOG(1) << "Glocal Exploration Planner set up successfully.";
  state_machine_->signalReady();
  run_srv_ = nh_private_.advertiseService("toggle_running",
                                          &GlocalPlanner::runSrvCallback, this);

  while (ros::ok() &&
         state_machine_->currentState() != StateMachine::Finished) {
    if (state_machine_->currentState() != StateMachine::Ready) {
      loopIteration();
    }
    ros::spinOnce();
  }
  VLOG(1) << "Glocal Exploration Planner finished planning.";
}

void GlocalPlanner::loopIteration() {
  // actions
  switch (state_machine_->currentState()) {
    case StateMachine::LocalPlanning: {
      local_planner_->planningIteration();
      break;
    }
  }

  // move requests
  WayPoint next_point;
  if (state_machine_->getNewWayPointIfRequested(&next_point)) {
    target_position_ = next_point.position();
    target_yaw_ = next_point.yaw;
    publishTargetPose();
    state_machine_->setTargetReached(false);

    // visualizations
    switch (state_machine_->currentState()) {
      case StateMachine::LocalPlanning: {
        local_planner_visualizer_->visualize();
        break;
      }
    }
  }
}

void GlocalPlanner::publishTargetPose() {
  geometry_msgs::Pose msg;
  tf2::Quaternion q;
  q.setRPY(0, 0, target_yaw_);
  msg.position.x = target_position_.x();
  msg.position.y = target_position_.y();
  msg.position.z = target_position_.z();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  target_pub_.publish(msg);
}

void GlocalPlanner::odomCallback(const nav_msgs::Odometry& msg) {
  // Track the current pose
  current_position_ =
      Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z);
  current_orientation_ = Eigen::Quaterniond(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

  // update the state machine with the current pose
  double yaw = tf2::getYaw(msg.pose.pose.orientation);
  WayPoint current_point;
  current_point.x = current_position_.x();
  current_point.y = current_position_.y();
  current_point.z = current_position_.z();
  current_point.yaw = yaw;
  state_machine_->setCurrentPose(current_point);

  // Check whether the goal pose is reached
  if (!state_machine_->targetIsReached()) {
    // check position
    if ((target_position_ - current_position_).norm() <=
        config_.replan_position_threshold) {
      // check yaw
      double yaw_diff = target_yaw_ - yaw;
      if (yaw_diff < 0) {
        yaw_diff += 2.0 * M_PI;
      }
      if (yaw_diff > M_PI) {
        yaw_diff = 2.0 * M_PI - yaw_diff;
      }
      if (yaw_diff <= config_.replan_yaw_threshold * M_PI / 180.0) {
        state_machine_->setTargetReached(true);
      }
    }
  }

  // check whether we're moving if we should be
  if (config_.republish_waypoints && !state_machine_->targetIsReached() &&
      (msg.header.stamp - previous_time_).toSec() > 1.0) {
    double distance = (current_position_ - previous_position_).norm();
    double yaw_diff = previous_yaw_ - yaw;
    if (yaw_diff < 0) {
      yaw_diff += 2.0 * M_PI;
    }
    if (yaw_diff > M_PI) {
      yaw_diff = 2.0 * M_PI - yaw_diff;
    }
    if (distance < 0.2 && yaw_diff < 15.0 * M_PI / 180.0) {
      publishTargetPose();
    }
    previous_time_ = msg.header.stamp;
    previous_yaw_ = yaw;
    previous_position_ = current_position_;
  }
}

bool GlocalPlanner::runSrvCallback(std_srvs::SetBool::Request& req,
                                   std_srvs::SetBool::Response& res) {
  state_machine_->signalLocalPlanning();
  if (state_machine_->currentState() == StateMachine::LocalPlanning) {
    VLOG(1) << "Started Glocal Exploration.";
    state_machine_->setTargetReached(true);
    previous_time_ = ros::Time::now();
    previous_position_ = current_position_;
    previous_yaw_ = state_machine_->currentPose().yaw;
    res.success = true;
  } else {
    res.success = false;
  }
  return true;
}

}  // namespace glocal_exploration