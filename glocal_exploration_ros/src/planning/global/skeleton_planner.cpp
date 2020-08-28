#include "glocal_exploration_ros/planning/global/skeleton_planner.h"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mav_planning_common/physical_constraints.h>

namespace glocal_exploration {

SkeletonPlanner::Config::Config() { setConfigName("SkeletonPlanner"); }

void SkeletonPlanner::Config::checkParams() const {}

void SkeletonPlanner::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam("use_frontier_clustering", &use_frontier_clustering);
  rosParam("frontier_clustering_radius", &frontier_clustering_radius);
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

  switch (stage_) {
    case Stage::k1ComputeFrontiers: {
      // Compute and update all frontiers to current state.
      if (computeFrontiers()) {
        stage_ = Stage::k2ComputeGoalAndPath;
      }
      break;
    }
    case Stage::k2ComputeGoalAndPath: {
      // Select a frontier to move towards, including path generation.
      if (computeGoalPoint()) {
        stage_ = Stage::k3ExecutePath;
      }
      break;
    }
    case Stage::k3ExecutePath: {
      // Execute way points until finished, then switch back to local.
      executeWayPoint();
      break;
    }
  }
}

void SkeletonPlanner::resetPlanner() { stage_ = Stage::k1ComputeFrontiers; }

bool SkeletonPlanner::computeFrontiers() {
  // Guarantee tha all frontiers are computed and update them to the current
  // state. If they are already pre-computed and frozen the computation step
  // will do nothing.
  visualization_info_.frontiers_changed = true;
  std::vector<MapBase::SubmapData> data;
  comm_->map()->getAllSubmapData(&data);

  // Check there are enough submaps already.
  if (data.empty()) {
    LOG_IF(INFO, config_.verbosity >= 2)
        << "No submaps finished yet for global planning, switching back local.";
    comm_->stateMachine()->signalLocalPlanning();
    return false;
  }

  // Compute and update frontiers.
  std::unordered_map<int, Transformation> update_list;
  for (const auto& datum : data) {
    computeFrontiersForSubmap(datum, Point(0, 0, 0));
    update_list[datum.id] = datum.T_M_S;
  }
  updateFrontiers(update_list);

  // Check there are still frontiers left.
  if (getActiveFrontiers().empty()) {
    // No more open frontiers, exploration is done.
    LOG_IF(INFO, config_.verbosity >= 1)
        << "No active frontiers remaining, exploration terminated succesfully.";
    comm_->stateMachine()->signalFinished();
    return false;
  }
  return true;
}

bool SkeletonPlanner::computeGoalPoint() {
  // Compute the frontier with the shortest path to it.
  visualization_info_.goals_changed = true;
  visualization_info_.goal_points.clear();

  // Get all frontiers.
  std::vector<FrontierSearchData> frontiers;
  for (const auto& frontier : getActiveFrontiers()) {
    // Compute active centroid.
    FrontierSearchData& data = frontiers.emplace_back();
    data.centroid = Point(0.0, 0.0, 0.0);
    for (const auto& point : *frontier) {
      if (point.is_active) {
        data.centroid += point.position;
        data.num_points++;
      }
    }
    if (data.num_points <= 0) {
      LOG(WARNING) << "Tried to compute the centroid for a frontier without "
                      "any active points.";
      continue;
    }
    data.centroid /= static_cast<FloatingPoint>(data.num_points);
    data.euclidean_distance =
        (comm_->currentPose().position() - frontier->centroid()).norm();
  }
  if (frontiers.empty()) {
    return false;
  }

  // Frontier clustering.
  if (config_.use_frontier_clustering) {
    const int num_frontiers = frontiers.size();
    for (auto it = frontiers.begin(); it !=frontiers.end(); ++it) {
      auto it2 = it;
      it2++;
      while (it2 != frontiers.end()) {
        if ((it2->centroid - it->centroid).norm() <= config_.frontier_clustering_radius) {
          // Nearby frontier centroids are weighted merged.
          it->centroid = it->centroid * static_cast<FloatingPoint>(it->num_points) + it2->centroid * static_cast<FloatingPoint>(it2->num_points);
          it->num_points += it2->num_points;
          it->centroid /= static_cast<FloatingPoint>(it->num_points);
          it2 = frontiers.erase(it2);
        } else {
          it2++;
        }
      }
    }
    // Logging.
    LOG_IF(INFO, config_.verbosity >= 3) << "Clustered " << num_frontiers -frontiers.size() << " frontiers (" << num_frontiers << "->" << frontiers.size() << ").";
  }


  // Compute paths to frontiers to determine the closest reachable one. Start
  // with closest and use euclidean distance as lower bound to prune candidates.
  std::sort(frontiers.begin(), frontiers.end(),
            [](const FrontierSearchData& lhs, const FrontierSearchData& rhs) {
              return lhs.euclidean_distance > rhs.euclidean_distance;
            });
  auto t_start = std::chrono::high_resolution_clock::now();
  int path_counter = 0;
  int unreachable_goal_counter = 0;
  const int total_frontiers = frontiers.size();
  std::vector<WayPoint> way_points;
  for (auto it = frontiers.begin(); it != frontiers.end();) {
    // Find feasible goal near frontier centroid.
    Point goal = it->centroid;
    if (!findValidGoalPoint(&goal)) {
      unreachable_goal_counter++;
      visualization_info_.goal_points.emplace_back(std::make_pair(3, goal));
      it = frontiers.erase(it);
      continue;
    }

    // Try to find a path via linked skeleton planning.
    path_counter++;
    if (computePath(goal, &way_points)) {
      it->way_points = way_points;
      it->path_distance =
          (way_points[0].position() - comm_->currentPose().position()).norm();
      for (size_t i = 1; i < way_points.size(); ++i) {
        it->path_distance +=
            (way_points[i].position() - way_points[i - 1].position()).norm();
      }
      visualization_info_.goal_points.emplace_back(std::make_pair(0, goal));

      // Prune paths that can not be shorter than this one.
      double dist = it->path_distance;
      auto it2 = ++it;
      while (it2 != frontiers.end()) {
        if (it2->euclidean_distance >= dist) {
          visualization_info_.goal_points.emplace_back(std::make_pair(2, goal));
          it2 = frontiers.erase(it2);
        } else {
          it2++;
        }
      }
    } else {
      // Remove inaccessible frontiers.
      visualization_info_.goal_points.emplace_back(std::make_pair(1, goal));
      it = frontiers.erase(it);
    }
  }

  // Logging.
  auto t_end = std::chrono::high_resolution_clock::now();
  std::stringstream ss;
  ss << "Evaluated " << path_counter << " of " << total_frontiers
     << " global paths in "
     << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
            .count()
     << "ms.";
  if (config_.verbosity >= 4) {
    ss << " Skipped " << unreachable_goal_counter
       << " unreachable goal points.";
  }
  LOG_IF(INFO, config_.verbosity >= 3) << ss.str();

  // Select result.
  if (frontiers.empty()) {
    LOG_IF(INFO, config_.verbosity >= 2)
        << "No reachable frontier found, returning to local planning.";
    comm_->stateMachine()->signalLocalPlanning();
    return false;
  }
  auto it = std::min_element(
      frontiers.begin(), frontiers.end(),
      [](const FrontierSearchData& lhs, const FrontierSearchData& rhs) {
        return lhs.path_distance < rhs.path_distance;
      });
  way_points_ = it->way_points;
  ss = std::stringstream();
  ss << "Found a path of " << way_points_.size()
     << " waypoints to closest frontier.";
  if (config_.verbosity >= 3) {
    ss << " (" << std::fixed << std::setprecision(2) << it->path_distance
       << "m path, " << std::fixed << std::setprecision(2)
       << it->euclidean_distance << "m euclidean distance).";
  }
  LOG_IF(INFO, config_.verbosity >= 2) << ss.str();
  return true;
}

void SkeletonPlanner::executeWayPoint() {
  if (comm_->targetIsReached()) {
    if (way_points_.empty()) {
      // Finished execution.
      comm_->stateMachine()->signalLocalPlanning();
      LOG_IF(INFO, config_.verbosity >= 2) << "Finished global path execution.";
    } else {
      // Request next way point.
      // TODO(schmluk): collision checks, min length
      WayPoint& way_point = way_points_[0];
      way_point.yaw = std::atan2((way_point.y - comm_->currentPose().y),
                                 (way_point.x - comm_->currentPose().x));
      comm_->requestWayPoint(way_point);
      way_points_.erase(way_points_.begin());
    }
  }
}

bool SkeletonPlanner::findValidGoalPoint(Point* goal) {
  if (comm_->map()->isTraversableInGlobalMap(*goal)) {
    return true;
  }
  // Sample from a cube to find candidates.
  for (const Point& offset : kNeighborOffsets_) {
    Point candidate = *goal + offset * kNeighborStepSize_;
    if (comm_->map()->isTraversableInGlobalMap(candidate)) {
      *goal = candidate;
      return true;
    }
  }
  return false;
}

bool SkeletonPlanner::computePath(const Point& goal,
                                  std::vector<WayPoint>* way_points) {
  CHECK_NOTNULL(way_points);
  // Setup data.
  geometry_msgs::PoseStamped start_pose;
  geometry_msgs::PoseStamped goal_pose;
  geometry_msgs::PoseArray path;
  start_pose.pose.position.x = comm_->currentPose().x;
  start_pose.pose.position.y = comm_->currentPose().y;
  start_pose.pose.position.z = comm_->currentPose().z;
  start_pose.pose.orientation.w = 1.0;
  goal_pose.pose.position.x = goal.x();
  goal_pose.pose.position.y = goal.y();
  goal_pose.pose.position.z = goal.z();
  goal_pose.pose.orientation.w = 1.0;

  // Compute path.
  if (!skeleton_planner_->planPath(start_pose, goal_pose, &path)) {
    return false;
  }
  if (path.poses.empty()) {
    return false;
  }

  // Write all way points to result.
  way_points->clear();
  way_points->reserve(path.poses.size());
  for (const auto& point : path.poses) {
    WayPoint temp_point;
    temp_point.x = point.position.x;
    temp_point.y = point.position.y;
    temp_point.z = point.position.z;
    way_points->emplace_back(temp_point);
  }
  return true;
}

}  // namespace glocal_exploration
