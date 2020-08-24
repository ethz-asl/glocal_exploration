#include "glocal_exploration_ros/planning/global/skeleton_planner.h"

#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>

#include <mav_planning_common/physical_constraints.h>

namespace glocal_exploration {

SkeletonPlanner::Config::Config() { setConfigName("SkeletonPlanner"); }

void SkeletonPlanner::Config::checkParams() const {}

void SkeletonPlanner::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam(&submap_frontier_config);
  nh_private_namespace = rosParamNameSpace();
  // Skeleton Planner.
  rosParam("collision_radius", &collision_radius);
  rosParam("use_path_shortening", &use_path_shortening);
  rosParam("verbose_skeleton_planner", &verbose_skeleton_planner);
  rosParam("frame_id", &frame_id);
}

void SkeletonPlanner::Config::printFields() const {
  printField("verbosity", verbosity);
  printField("nh_private_namespace", nh_private_namespace);
  printField("submap_frontier_config", submap_frontier_config);
  // Skeleton Planner.
  printField("collision_radius", collision_radius);
  printField("use_path_shortening", use_path_shortening);
  printField("verbose_skeleton_planner", verbose_skeleton_planner);
  printField("frame_id", frame_id);
}

SkeletonPlanner::SkeletonPlanner(const Config& config,
                                 std::shared_ptr<Communicator> communicator)
    : config_(config.checkValid()),
      SubmapFrontierEvaluator(config.submap_frontier_config,
                              std::move(communicator)) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // NOTE(schmluk): The skeleton map has an internal cblox server that
  //                subscribes to submaps and submap poses via ros. One needs to
  //                make sure these are remapped properly. The ubmaps are
  //                skeletonized upon reception.
  // Setup the skeleton planner.
  ros::NodeHandle nh_private(config_.nh_private_namespace);
  ros::NodeHandle nh(ros::names::parentNamespace(config_.nh_private_namespace));
  skeleton_map_ = std::make_unique<SkeletonMap>(nh, nh_private);
  skeleton_planner_ = std::make_unique<mav_planning::CbloxSkeletonPlanner>();
  // NOTE(schmluk): These planners are setup later during planning.
}

void SkeletonPlanner::planningIteration() {
  // TEST: compute all frontiers.
  //  std::vector<MapBase::SubmapData> data;
  //  comm_->map()->getAllSubmapData(&data);
  //  std::unordered_map<int, Transformation> update_list;
  //  for (const auto& datum : data) {
  //    computeFrontiersForSubmap(datum, Point(0, 0, 0));
  //    update_list[datum.id] = datum.T_M_S;
  //  }
  //  updateFrontiers(update_list);
  //  return;

  // Newly started global planning.
  //  if (comm_->stateMachine()->previousState() !=
  //      StateMachine::State::kGlobalPlanning) {
  //    resetPlanner();
  //    comm_->stateMachine()->signalGlobalPlanning();
  //  }

  if ((ros::Time::now() - tmp_).toSec() <= 3.0) {
    return;
  }
  resetPlanner();
  tmp_ = ros::Time::now();

  // Stage1: compute the target point.
  if (stage_ == Stage::k1ComputePoint) {
    if (computeGoalPoint()) {
      stage_ = Stage::k2ComputePath;
    }
  }

  // Stage2: let the skeleton planner compute a path to the target.
  if (stage_ == Stage::k2ComputePath) {
    if (computePathToGoal()) {
      stage_ = Stage::k3ExecutePath;
    }
  }

  // Stage3: execute the path segments.
  if (stage_ == Stage::k3ExecutePath) {
    if (comm_->targetIsReached()) {
      if (way_points_.empty()) {
        // Finished execution.
        comm_->stateMachine()->signalLocalPlanning();
        LOG(WARNING) << "Finished Global Path!";
      } else {
        // Request next way point.
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

void SkeletonPlanner::resetPlanner() { stage_ = Stage::k1ComputePoint; }

bool SkeletonPlanner::computeGoalPoint() {
  // check there are enough submaps already

  // compute the best frontier

  // check it is not in the active submap

  // find a reachable point near the best frontier

  goal_point_ = Eigen::Vector3d(2, 0, 0);
  return true;
}

bool SkeletonPlanner::computePathToGoal() {
  // Setup data.
  setupSkeletonPlanner();
  mav_planning::LocalTrajectoryPointVector path;
  mav_msgs::EigenTrajectoryPoint start;
  mav_msgs::EigenTrajectoryPoint goal;
  start.position_W = comm_->currentPose().position();
  goal.position_W = goal_point_;

  // Check the map.
  if (!skeleton_map_->isMapInitialized()) {
    LOG(WARNING) << "Skeleton map is not initialized.";
    return false;
  }

  // Compute the path.
  if (!skeleton_planner_->getPathBetweenWaypoints(start, goal, &path)) {
    LOG(WARNING) << "Global path extraction failed.";
    return false;
  }

  // Transform path to global frame.
  mav_msgs::EigenTrajectoryPointVector global_path =
      skeleton_map_->transformLocalTpvToEigenTpv(path);

  // Enqueue all way points.
  way_points_ = std::queue<Eigen::Vector3d>();
  for (const auto& point : global_path) {
    way_points_.push(point.position_W);
  }
  LOG(WARNING) << "Found global path.";
  return true;
}

void SkeletonPlanner::setupSkeletonPlanner() {
  mav_planning::PhysicalConstraints constraints_;
  constraints_.robot_radius = config_.collision_radius;
  skeleton_planner_->setConstraints(constraints_);
  skeleton_planner_->setVerbose(config_.verbose_skeleton_planner);
  skeleton_planner_->setPathShortening(config_.use_path_shortening);
  skeleton_planner_->setFrameID(config_.frame_id);

  // Setup planner-map links.
  skeleton_planner_->setMapObject(skeleton_map_.get());
  skeleton_planner_->setMapDistanceCallback(
      std::bind(&SkeletonMap::getMapDistance, skeleton_map_.get(),
                std::placeholders::_1));
  skeleton_planner_->setCheckCollisionCallback(std::bind(
      &SkeletonMap::checkCollision, skeleton_map_.get(), std::placeholders::_1,
      std::placeholders::_2, constraints_.robot_radius));
  skeleton_planner_->setTransformLocalPointCallback(
      std::bind(&SkeletonMap::transformLocalTrajectoryPointToGlobalPoint,
                skeleton_map_.get(), std::placeholders::_1));
  skeleton_planner_->setOptimisticCallback(std::bind(
      &SkeletonMap::setOptimistic, skeleton_map_.get(), std::placeholders::_1));
  skeleton_planner_->setVerboseCallback(std::bind(
      &SkeletonMap::setVerbose, skeleton_map_.get(), std::placeholders::_1));
  skeleton_planner_->setOptimistic(skeleton_map_->getOptimistic());
  skeleton_planner_->setupMap(skeleton_map_->getSubmapcollection());
  skeleton_planner_->updateSparseGraph();
}

}  // namespace glocal_exploration
