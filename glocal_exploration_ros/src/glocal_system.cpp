#include "glocal_exploration_ros/glocal_system.h"

#include <memory>

#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>

#include "glocal_exploration_ros/conversions/ros_component_factory.h"

namespace glocal_exploration {

GlocalSystem::Config::Config() { setConfigName("GlocalSystem"); }

void GlocalSystem::Config::checkParams() const {
  checkParamGT(replan_position_threshold, 0.f, "replan_position_threshold");
  checkParamGT(replan_yaw_threshold, 0.f, "replan_yaw_threshold");
  checkParamGT(collision_check_period_s, 0.f, "collision_check_period_s");
  checkParamGT(max_planner_update_frequency, 0.f,
               "max_planner_update_frequency");
}

void GlocalSystem::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam("replan_position_threshold", &replan_position_threshold);
  rosParam("replan_yaw_threshold", &replan_yaw_threshold);
  rosParam("replan_timeout_constant", &replan_timeout_constant);
  rosParam("replan_timeout_velocity", &replan_timeout_velocity);
  rosParam("collision_check_period_s", &collision_check_period_s);
  rosParam("max_planner_update_frequency", &max_planner_update_frequency);
}

void GlocalSystem::Config::printFields() const {
  printField("verbosity", verbosity);
  printField("replan_position_threshold", replan_position_threshold);
  printField("replan_yaw_threshold", replan_yaw_threshold);
  printField("replan_timeout_constant", replan_timeout_constant);
  printField("replan_timeout_velocity", replan_timeout_velocity);
  printField("collision_check_period_s", collision_check_period_s);
  printField("max_planner_update_frequency", max_planner_update_frequency);
}

GlocalSystem::GlocalSystem(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private)
    : GlocalSystem(nh, nh_private,
                   config_utilities::getConfigFromRos<Config>(nh_private)) {}

GlocalSystem::GlocalSystem(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private,
                           const Config& config)
    : nh_(nh),
      nh_private_(nh_private),
      config_(config.checkValid()),
      current_position_(Point::Zero()),
      current_orientation_(Eigen::Quaterniond::Identity()),
      target_position_(Point::Zero()),
      target_yaw_(0.f),
      total_planning_cpu_time_s_(0.0),
      collision_check_period_(config.collision_check_period_s),
      collision_check_last_timestamp_(0),
      signal_collision_avoidance_triggered_(false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" + config_.toString();
  // build communicator and components
  buildComponents(nh_private_);

  // ROS
  target_pub_ = nh_.advertise<geometry_msgs::Pose>("command/pose", 10);
  odom_sub_ = nh_.subscribe("odometry", 1, &GlocalSystem::odomCallback, this);
  collision_check_timer_ = nh_.createTimer(
      collision_check_period_,
      std::bind(&GlocalSystem::performCollisionAvoidance, this));
  total_planning_cpu_time_pub_ =
      nh_.advertise<std_msgs::Float32>("total_planning_cpu_time", 1);
  collision_avoidance_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>(
      "collision_avoidance", 1);
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

  // Limit the maximum update frequency.
  // NOTE: This is mainly intended to avoid spinning at unlimited rates in case
  //       a loopIteration() returns immediately. For example, when the global
  //       planner is idling while waiting for the next waypoint to be reached.
  ros::Rate max_rate(config_.max_planner_update_frequency);

  // Spin.
  while (ros::ok() && comm_->stateMachine()->currentState() !=
                          StateMachine::State::kFinished) {
    loopIteration();
    ros::spinOnce();

    // Limit the maximum planner update frequency
    // NOTE: The local planner is exempt from this rate limit since fast
    //       restarts are useful when paths become infeasible.
    //       Furthermore, collision avoidance is not affected since it runs
    //       in a separate thread.
    if (comm_->stateMachine()->currentState() !=
        StateMachine::State::kLocalPlanning) {
      // Sleep only if the maximum rate would otherwise be exceeded
      max_rate.sleep();
    }
  }
  LOG_IF(INFO, config_.verbosity >= 1)
      << "Glocal Exploration Planner finished planning.";
}

void GlocalSystem::loopIteration() {
  // Start tracking the planning CPU time
  // NOTE: This way of measuring the CPU usage of the planners assumes that they
  //       are single threaded, which is the case in our current implementation.
  struct timespec start_cpu_time;
  clockid_t current_thread_clock_id;
  pthread_getcpuclockid(pthread_self(), &current_thread_clock_id);
  clock_gettime(current_thread_clock_id, &start_cpu_time);

  // Actions.
  switch (comm_->stateMachine()->currentState()) {
    case StateMachine::State::kLocalPlanning: {
      comm_->localPlanner()->executePlanningIteration();
      local_planner_visualizer_->visualize();
      break;
    }
    case StateMachine::State::kGlobalPlanning: {
      comm_->globalPlanner()->executePlanningIteration();
      global_planner_visualizer_->visualize();
      break;
    }
    default:
      // No need to do anything.
      break;
  }

  // Move requests.
  if (comm_->newWayPointIsRequested() &&
      !signal_collision_avoidance_triggered_) {
    const WayPoint& next_point = comm_->getRequestedWayPoint();
    target_position_ = next_point.position;
    target_yaw_ = next_point.yaw;
    publishTargetPose();
    comm_->setRequestedWayPointRead();
  }

  // Reset the local planner in case collision avoidance was triggered.
  if (signal_collision_avoidance_triggered_) {
    if (comm_->stateMachine()->currentState() ==
        StateMachine::State::kLocalPlanning) {
      const WayPoint safe_waypoint(target_position_, target_yaw_);
      comm_->localPlanner()->resetPlanner(safe_waypoint);
    }
    signal_collision_avoidance_triggered_ = false;
  }

  // Stop tracking the planning CPU time
  struct timespec stop_cpu_time;
  clock_gettime(current_thread_clock_id, &stop_cpu_time);
  total_planning_cpu_time_s_ +=
      static_cast<double>(stop_cpu_time.tv_sec - start_cpu_time.tv_sec) +
      static_cast<double>(stop_cpu_time.tv_nsec - start_cpu_time.tv_nsec) *
          1e-9;
  if (0 < total_planning_cpu_time_pub_.getNumSubscribers()) {
    std_msgs::Float32 total_planning_cpu_time_msg;
    total_planning_cpu_time_msg.data = total_planning_cpu_time_s_;
    total_planning_cpu_time_pub_.publish(total_planning_cpu_time_msg);
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

  // Update the tracking state.
  comm_->setTargetReached(false);

  // Compute the timeout limit.
  last_waypoint_timeout_ = config_.replan_timeout_constant;
  if (config_.replan_timeout_velocity > 0) {
    last_waypoint_timeout_ +=
        (comm_->currentPose().position - target_position_).norm() /
        config_.replan_timeout_velocity;
  }
  last_waypoint_published_ = ros::Time::now();
}

void GlocalSystem::odomCallback(const nav_msgs::Odometry& msg) {
  // Track the current pose
  current_position_ = Point(msg.pose.pose.position.x, msg.pose.pose.position.y,
                            msg.pose.pose.position.z);
  current_orientation_ = Eigen::Quaterniond(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);

  // update the state machine with the current pose.
  FloatingPoint yaw = tf2::getYaw(msg.pose.pose.orientation);
  WayPoint current_point;
  current_point.position = current_position_;
  current_point.yaw = yaw;
  comm_->setCurrentPose(current_point);

  const StateMachine::State& state = comm_->stateMachine()->currentState();
  if (state == StateMachine::State::kReady ||
      state == StateMachine::State::kSettingUp ||
      state == StateMachine::State::kFinished) {
    return;
  }

  // Check whether the goal pose is reached
  if (!comm_->targetIsReached()) {
    // check proximity.
    if ((target_position_ - current_position_).norm() <=
        config_.replan_position_threshold) {
      // check yaw.
      FloatingPoint yaw_diff = target_yaw_ - yaw;
      if (yaw_diff < 0) {
        yaw_diff += 2.f * M_PI;
      }
      if (yaw_diff > M_PI) {
        yaw_diff = 2.f * M_PI - yaw_diff;
      }
      if (yaw_diff <= config_.replan_yaw_threshold * M_PI / 180.f) {
        comm_->setTargetReached(true);
      }
    }

    // Check whether the last move command is timing out.
    if (config_.replan_timeout_velocity > 0.f ||
        config_.replan_timeout_constant > 0.f) {
      if ((msg.header.stamp - last_waypoint_published_).toSec() >
          last_waypoint_timeout_) {
        // NOTE(schmluk): This usually means the MAV is close to the target but
        // the controller did not exactly reach the threshold. Therefore we
        // assume the target is reached and continue planning.
        comm_->setTargetReached(true);
        LOG_IF(INFO, config_.verbosity >= 3)
            << "Waypoint timed out after " << std::fixed << std::setprecision(2)
            << last_waypoint_timeout_ << "s, continuing with next goal.";
      }
    }
  }
}

bool GlocalSystem::startExploration() {
  if (comm_->stateMachine()->currentState() != StateMachine::State::kReady) {
    // Can not start from another state than ready.
    LOG(WARNING) << "Can only start exploration from state Ready (is '"
                 << StateMachine::stateToString(
                        comm_->stateMachine()->currentState())
                 << "').";
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

void GlocalSystem::performCollisionAvoidance() {
  ros::Time current_timestamp = ros::Time::now();
  if (1.2 * collision_check_period_.toSec() <
      std::abs((current_timestamp - collision_check_last_timestamp_).toSec())) {
    LOG(INFO)
        << "Collision checking is not running at its requested frequency. "
           "The requested period is "
        << collision_check_period_.toSec()
        << "s, but the time since last call is "
        << (current_timestamp - collision_check_last_timestamp_).toSec()
        << "s.";
  }
  collision_check_last_timestamp_ = current_timestamp;

  // Check if the line to the upcoming waypoint became intraversable.
  const FloatingPoint traversability_radius =
      comm_->map()->getTraversabilityRadius();
  const Point current_position = comm_->currentPose().position;
  constexpr bool kOptimistic = true;
  if (!comm_->map()->isLineTraversableInActiveSubmap(
          current_position, target_position_, kOptimistic)) {
    // Only intervene if we're actively flying toward a surface.
    FloatingPoint distance;
    Point gradient;
    if (comm_->map()->getDistanceAndGradientInActiveSubmap(
            current_position, &distance, &gradient)) {
      if (gradient.dot(target_position_ - current_position) < 0.f) {
        // Try to find a safe position near the current position.
        Point safe_position = current_position;
        if (comm_->map()->findSafestNearbyPoint(traversability_radius,
                                                &safe_position)) {
          LOG(WARNING)
              << "Attempting to fly to safety and continue from there.";
          // Rotate by 180deg to reduce unobserved space in the active
          // submap.
          const FloatingPoint new_yaw = comm_->currentPose().yaw + M_PI;
          const WayPoint safe_waypoint(safe_position, new_yaw);
          target_position_ = safe_waypoint.position;
          target_yaw_ = safe_waypoint.yaw;
          publishTargetPose();
          comm_->setRequestedWayPointRead();
          signal_collision_avoidance_triggered_ = true;

          // Visualize the goal point chosen by the collision avoidance
          geometry_msgs::PointStamped goal_point_msg;
          goal_point_msg.header.frame_id = "odom";
          goal_point_msg.header.stamp = current_timestamp;
          goal_point_msg.point.x = safe_waypoint.position.x();
          goal_point_msg.point.y = safe_waypoint.position.y();
          goal_point_msg.point.z = safe_waypoint.position.z();
          collision_avoidance_pub_.publish(goal_point_msg);
        }
      }
    }
  }
}

}  // namespace glocal_exploration
