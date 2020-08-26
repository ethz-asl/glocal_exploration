#include "glocal_exploration_ros/planning/global/skeleton_planner.h"

#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mav_planning_common/physical_constraints.h>

namespace glocal_exploration {

SkeletonPlanner::Config::Config() { setConfigName("SkeletonPlanner"); }

void SkeletonPlanner::Config::checkParams() const {}

void SkeletonPlanner::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam(&submap_frontier_config);
  nh_private_namespace = rosParamNameSpace() + "/skeleton";
}

void SkeletonPlanner::Config::printFields() const {
  printField("verbosity", verbosity);
  printField("nh_private_namespace", nh_private_namespace);
  printField("submap_frontier_config", submap_frontier_config);
}

SkeletonPlanner::SkeletonPlanner(const Config& config,
                                 std::shared_ptr<Communicator> communicator)
    : config_(config.checkValid()),
      SubmapFrontierEvaluator(config.submap_frontier_config,
                              std::move(communicator)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // NOTE(schmluk): The skeleton planner has an internal cblox server that
  //                subscribes to submaps and submap poses via ros. One needs to
  //                make sure these are remapped properly. The submaps are
  //                skeletonized upon reception.
  // Setup the skeleton planner.
  ros::NodeHandle nh_private(config_.nh_private_namespace);
  ros::NodeHandle nh(ros::names::parentNamespace(config_.nh_private_namespace));
  skeleton_planner_ =
      std::make_unique<mav_planning::CbloxSkeletonGlobalPlanner>(nh,
                                                                 nh_private);
  skeleton_planner_->setupPlannerAndSmoother();
}

void SkeletonPlanner::planningIteration() {
  // Newly started global planning.
  if (comm_->stateMachine()->previousState() !=
      StateMachine::State::kGlobalPlanning) {
    resetPlanner();
    comm_->stateMachine()->signalGlobalPlanning();
  }

  // TEST
  if ((ros::Time::now() - tmp_).toSec() <= 3.0) {
    return;
  }
  resetPlanner();
  tmp_ = ros::Time::now();
  // TODO(schmluk): check we have submaps.

  switch (stage_) {
    case Stage::k1ComputeFrontiers: {
      // Compute and update all frontiers to current state.
      if (computeFrontiers()) {
        stage_ = Stage::k2ComputeGoalPoint;
      }
    }
    case Stage::k2ComputeGoalPoint: {
      // Select a frontier to move towards.
      if (computeGoalPoint()) {
        stage_ = Stage::k3ComputePath;
      }
    }
    case Stage::k3ComputePath: {
      // let the skeleton planner compute a path to the target.
      if (computePathToGoal(goal_point_)) {
        stage_ = Stage::k4ExecutePath;
      }
    }
    case Stage::k4ExecutePath: {
      if (comm_->targetIsReached()) {
        if (way_points_.empty()) {
          // Finished execution.
          comm_->stateMachine()->signalLocalPlanning();
          LOG_IF(INFO, config_.verbosity >= 2)
              << "Finished global path execution.";
        } else {
          // Request next way point.
          // TODO(schmluk): collision checks, min length
          WayPoint way_point;
          way_point.x = way_points_.front().x();
          way_point.y = way_points_.front().y();
          way_point.z = way_points_.front().z();
          way_point.yaw =
              std::atan2((way_points_.front().y() - comm_->currentPose().y),
                         (way_points_.front().x() - comm_->currentPose().x));
          comm_->requestWayPoint(way_point);
          way_points_.pop();
        }
      }
    }
  }
}

void SkeletonPlanner::resetPlanner() { stage_ = Stage::k1ComputeFrontiers; }

bool SkeletonPlanner::computeFrontiers() {
  // Guarantee tha all frontiers are computed and update them to the current
  // state. If they are already pre-computed and frozen the computation step
  // will do nothing.
  std::vector<MapBase::SubmapData> data;
  comm_->map()->getAllSubmapData(&data);
  std::unordered_map<int, Transformation> update_list;
  for (const auto& datum : data) {
    computeFrontiersForSubmap(datum, Point(0, 0, 0));
    update_list[datum.id] = datum.T_M_S;
  }
  updateFrontiers(update_list);

  // Check there are still frontiers left.
  //  if (getActiveFrontiers().empty()) {
  //    // No more open frontiers, exploration is done.
  //    LOG_IF(INFO, config_.verbosity >= 1)
  //    << "No active frontiers remaining, exploration succeeded.";
  //    comm_->stateMachine()->signalFinished();
  //    return false;
  //  }
  return true;
}

bool SkeletonPlanner::computeGoalPoint() {
  // check there are enough submaps already

  // compute the best frontier

  // check it is not in the active submap

  // find a reachable point near the best frontier

  goal_point_ = Eigen::Vector3d(2, 0, 0);
  return true;
}

bool SkeletonPlanner::computePathToGoal(const Point& goal) {
  // Setup data.
  auto t_start = std::chrono::high_resolution_clock::now();
  geometry_msgs::PoseStamped start_pose;
  geometry_msgs::PoseStamped goal_pose;
  geometry_msgs::PoseArray way_points;
  start_pose.pose.position.x = comm_->currentPose().x;
  start_pose.pose.position.y = comm_->currentPose().y;
  start_pose.pose.position.z = comm_->currentPose().z;
  start_pose.pose.orientation.w = 1.0;
  goal_pose.pose.position.x = goal.x();
  goal_pose.pose.position.y = goal.y();
  goal_pose.pose.position.z = goal.z();
  goal_pose.pose.orientation.w = 1.0;

  // Compute path.
  if (!skeleton_planner_->planPath(start_pose, goal_pose, &way_points)) {
    LOG(WARNING) << "Global path computation failed.";
    // TODO(schmluk): what todo in this case?
    return false;
  }
  if (way_points.poses.empty()) {
    LOG(WARNING) << "Global path did not contain any waypoints.";
    // TODO(schmluk): what todo in this case?
    return false;
  }

  // Enqueue all way points.
  way_points_ = std::queue<Eigen::Vector3d>();
  for (const auto& point : way_points.poses) {
    Point temp_point;
    temp_point.x() = point.position.x;
    temp_point.y() = point.position.y;
    temp_point.z() = point.position.z;
    way_points_.push(temp_point);
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Found a path to goal in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << "ms.";
  return true;
}

}  // namespace glocal_exploration
