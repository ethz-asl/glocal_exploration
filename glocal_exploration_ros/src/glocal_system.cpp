#include "glocal_exploration_ros/glocal_system.h"

#include <memory>

#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>

#include "glocal_exploration_ros/conversions/ros_component_factory.h"

namespace glocal_exploration {

GlocalSystem::Config::Config() { setConfigName("GlocalSystem"); }

void GlocalSystem::Config::checkParams() const {
  checkParamGT(replan_position_threshold, 0.0, "replan_position_threshold");
  checkParamGT(replan_yaw_threshold, 0.0, "replan_yaw_threshold");
}

void GlocalSystem::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam("replan_position_threshold", &replan_position_threshold);
  rosParam("replan_yaw_threshold", &replan_yaw_threshold);
  rosParam("waypoint_timeout", &waypoint_timeout);
}

GlocalSystem::GlocalSystem(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private)
    : GlocalSystem(nh, nh_private,
                   config_utilities::getConfigFromRos<Config>(nh_private)) {}

GlocalSystem::GlocalSystem(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private,
                           const Config& config)
    : nh_(nh), nh_private_(nh_private), config_(config.checkValid()) {
  // build communicator and components
  buildComponents(nh_private_);

  // ROS
  target_pub_ = nh_.advertise<geometry_msgs::Pose>("command/pose", 10);
  odom_sub_ = nh_.subscribe("odometry", 1, &GlocalSystem::odomCallback, this);
}

void GlocalSystem::buildComponents(const ros::NodeHandle& nh) {
  // Initialize the communicator
  comm_ = std::make_shared<Communicator>();

  // setup the state machine
  comm_->setupStateMachine(std::make_shared<StateMachine>());

  // setup the region of interest
  ros::NodeHandle nh_roi(nh, "region_of_interest");
  comm_->setupRegionOfInterest(
      ComponentFactoryROS::createRegionOfInterest(nh_roi));

  // setup the map
  ros::NodeHandle nh_mapping(nh, "mapping");
  comm_->setupMap(ComponentFactoryROS::createMap(nh_mapping, comm_));

  // setup the local planner + visualizer
  ros::NodeHandle nh_local_planner(nh, "local_planner");
  comm_->setupLocalPlanner(
      ComponentFactoryROS::createLocalPlanner(nh_local_planner, comm_));
  local_planner_visualizer_ = ComponentFactoryROS::createLocalPlannerVisualizer(
      nh_local_planner, comm_);

  // setup the global planner + visualizer
  ros::NodeHandle nh_global_planner(nh, "global_planner");
  comm_->setupGlobalPlanner(
      ComponentFactoryROS::createGlobalPlanner(nh_global_planner, comm_));
  global_planner_visualizer_ =
      ComponentFactoryROS::createGlobalPlannerVisualizer(nh_global_planner,
                                                         comm_);
}

void GlocalSystem::mainLoop() {
  // This is the main loop, spinning is managed explicitly for efficiency
  // starting the main loop means everything is setup.
  LOG_IF(INFO, config_.verbosity >= 1)
      << "Glocal Exploration Planner set up successfully.";
  comm_->stateMachine()->signalReady();
  run_srv_ = nh_private_.advertiseService("toggle_running",
                                          &GlocalSystem::runSrvCallback, this);

  while (ros::ok() && comm_->stateMachine()->currentState() !=
                          StateMachine::State::kFinished) {
    loopIteration();
    ros::spinOnce();
  }
  LOG_IF(INFO, config_.verbosity >= 1)
      << "Glocal Exploration Planner finished planning.";
}

void GlocalSystem::loopIteration() {
  // Actions.
  switch (comm_->stateMachine()->currentState()) {
    case StateMachine::State::kLocalPlanning: {
      comm_->localPlanner()->planningIteration();
      local_planner_visualizer_->visualize();
      break;
    }
    case StateMachine::State::kGlobalPlanning: {
      comm_->globalPlanner()->planningIteration();
      global_planner_visualizer_->visualize();
      break;
    }
  }

  // Move requests.
  if (comm_->newWayPointIsRequested()) {
    const WayPoint& next_point = comm_->getRequestedWayPoint();
    target_position_ = next_point.position();
    target_yaw_ = next_point.yaw;
    publishTargetPose();
    comm_->setRequestedWayPointRead();
  }
}

void GlocalSystem::publishTargetPose() {
  // publish the target pose.
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

  // update the tracking state.
  comm_->setTargetReached(false);
  time_last_waypoint_started_ = ros::Time::now();
}

void GlocalSystem::odomCallback(const nav_msgs::Odometry& msg) {
  // Track the current pose
  current_position_ =
      Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z);
  current_orientation_ = Eigen::Quaterniond(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

  // update the state machine with the current pose.
  double yaw = tf2::getYaw(msg.pose.pose.orientation);
  WayPoint current_point;
  current_point.x = current_position_.x();
  current_point.y = current_position_.y();
  current_point.z = current_position_.z();
  current_point.yaw = yaw;
  comm_->setCurrentPose(current_point);

  // Check whether the goal pose is reached
  if (!comm_->targetIsReached()) {
    // check proximity.
    if ((target_position_ - current_position_).norm() <=
        config_.replan_position_threshold) {
      // check yaw.
      double yaw_diff = target_yaw_ - yaw;
      if (yaw_diff < 0) {
        yaw_diff += 2.0 * M_PI;
      }
      if (yaw_diff > M_PI) {
        yaw_diff = 2.0 * M_PI - yaw_diff;
      }
      if (yaw_diff <= config_.replan_yaw_threshold * M_PI / 180.0) {
        comm_->setTargetReached(true);
      }
    }

    // check whether the last move command is timing out.
    if (config_.waypoint_timeout > 0.0) {
      if ((msg.header.stamp - time_last_waypoint_started_).toSec() >
          config_.waypoint_timeout) {
        // NOTE(schmluk): This usually means the MAV is close to the target but
        // the controller did not exactly reach the threshold. Therefore we
        // assume the target is reached and continue planning.
        comm_->setTargetReached(true);
      }
    }
  }
}

bool GlocalSystem::startExploration() {
  if (comm_->stateMachine()->currentState() != StateMachine::State::kReady) {
    // Can not start from another state than ready.
    return false;
  }

  // setup initial state.
  comm_->stateMachine()->signalLocalPlanning();
  comm_->setTargetReached(true);
  LOG_IF(INFO, config_.verbosity >= 1) << "Started Glocal Exploration Planner.";
  return true;
}

bool GlocalSystem::runSrvCallback(std_srvs::SetBool::Request& req,
                                  std_srvs::SetBool::Response& res) {
  res.success = startExploration();
  return true;
}

}  // namespace glocal_exploration
