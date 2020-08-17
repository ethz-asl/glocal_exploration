#include "glocal_exploration_ros/planning/global/skeleton_planner.h"

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mav_planning_msgs/PlannerService.h>

namespace glocal_exploration {

bool SkeletonPlanner::Config::isValid() const { return true; }

SkeletonPlanner::Config SkeletonPlanner::Config::checkValid() const {
  CHECK(isValid());
  return Config(*this);
}

SkeletonPlanner::SkeletonPlanner(const Config& config,
                                 std::shared_ptr<Communicator> communicator)
    : config_(config.checkValid()),
      SubmapFrontierEvaluator(config.submap_frontier_config,
                              std::move(communicator)) {
  // setup servers
  nh_ = ros::NodeHandle(ros::names::parentNamespace(config_.nh_namespace));
  skeleton_planner_srv_ = nh_.serviceClient<mav_planning_msgs::PlannerService>(
      config_.service_name);

  // Setup planner
  skeleton_planner_ = std::make_unique<mav_planning::CbloxSkeletonPlanner>();
}

void SkeletonPlanner::planningIteration() {
  // TEST: compute all frontiers.
  std::vector<MapBase::SubmapData> data;
  comm_->map()->getAllSubmapData(&data);
  std::unordered_map<int, Transformation> update_list;
  for (const auto& datum : data) {
    computeFrontiersForSubmap(datum, Point(0, 0, 0));
    update_list[datum.id] = datum.T_M_S;
  }
  updateFrontiers(update_list);
  return;

  // Newly started global planning
  if (comm_->stateMachine()->previousState() !=
      StateMachine::State::kGlobalPlanning) {
    resetPlanner();
    comm_->stateMachine()->signalGlobalPlanning();
  }

  // stage1: compute the target point
  if (stage_ == 1) {
    if (computeGoalPoint()) {
      stage_ = 2;
    }
  }

  // stage2: let the skeleton planner a path to the target
  if (stage_ == 2) {
    if (computePathToGoal()) {
      stage_ = 3;
    }
  }

  // stage3: execute
  if (stage_ == 3) {
    if (comm_->targetIsReached()) {
      if (way_points_.empty()) {
        // finished execution
        comm_->stateMachine()->signalLocalPlanning();
      } else {
        // request next point
        WayPoint way_point;
        way_point.x = way_points_.front().x();
        way_point.y = way_points_.front().y();
        way_point.z = way_points_.front().z();
        way_point.yaw =
            std::atan2((way_points_.front().y() - comm_->currentPose().y),
                       (way_points_.front().x() - comm_->currentPose().x));
        comm_->requestWayPoint(way_point);
      }
    }
  }
}

void SkeletonPlanner::resetPlanner() {
  stage_ = 1;
}

bool SkeletonPlanner::computeGoalPoint() {
  // check there are enough submaps already

  // compute the best frontier

  // check it is not in the active submap

  // find a reachable point near the best frontier

  goal_point_ = Eigen::Vector3d(0, 0, 0);
  return true;
}

bool SkeletonPlanner::computePathToGoal() {
  mav_planning_msgs::PlannerService srv;

  // Set the current pose transformed in Mission frame
  //  Point current_position_M = voxgraph_map_ptr_->get_T_M_O() *
  //      voxblox::Point(current_position_.x(), current_position_.y(),
  //      current_position_.z());
  //  srv.request.start_pose.pose.position.x = current_position_M.x();
  //  srv.request.start_pose.pose.position.y = current_position_M.y();
  //  srv.request.start_pose.pose.position.z = current_position_M.z();

  // Set the goal pose transformed in Mission frame
  //  voxblox::Point goal_point_M = voxgraph_map_ptr_->get_T_M_O() *
  //      voxblox::Point(goal_point.pose.position.x, goal_point.pose.position.y,
  //      goal_point.pose.position.z);
  //  srv.request.goal_pose.pose.position.x = goal_point_M.x();
  //  srv.request.goal_pose.pose.position.y = goal_point_M.y();
  //  srv.request.goal_pose.pose.position.z = goal_point_M.z();
  //  srv.request.goal_pose.pose.position.x = goal_point_M.x();
  //  srv.request.goal_pose.pose.position.y = goal_point_M.y();
  //  srv.request.goal_pose.pose.position.z = goal_point_M.z();

  // Global planning
  skeleton_planner_srv_.call(srv);
  return srv.response.success;
}

}  // namespace glocal_exploration
