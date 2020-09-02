#include "glocal_exploration_ros/planning/global/skeleton_planner.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mav_planning_common/physical_constraints.h>

namespace glocal_exploration {

SkeletonPlanner::Config::Config() { setConfigName("SkeletonPlanner"); }

void SkeletonPlanner::Config::checkParams() const {
  checkParamGT(goal_search_steps, 0, "goal_search_steps");
}

void SkeletonPlanner::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam("use_centroid_clustering", &use_centroid_clustering);
  rosParam("centroid_clustering_radius", &centroid_clustering_radius);
  rosParam("use_path_verification", &use_path_verification);
  rosParam("path_verification_min_distance", &path_verification_min_distance);
  rosParam("goal_search_steps", &goal_search_steps);
  rosParam("goal_search_step_size", &goal_search_step_size);
  rosParam(&submap_frontier_config);
  nh_private_namespace = rosParamNameSpace() + "/skeleton";
}

void SkeletonPlanner::Config::printFields() const {
  printField("verbosity", verbosity);
  printField("use_centroid_clustering", use_centroid_clustering);
  printField("centroid_clustering_radius", centroid_clustering_radius);
  printField("use_path_verification", use_path_verification);
  printField("path_verification_min_distance", path_verification_min_distance);
  printField("goal_search_steps", goal_search_steps);
  printField("goal_search_step_size", goal_search_step_size);
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

  // Precompute goal search offsets (points on cube ordered by distance).
  goal_search_offsets_.reserve(std::pow(config_.goal_search_steps, 3));
  for (int i_x = 0; i_x < config_.goal_search_steps; ++i_x) {
    FloatingPoint x =
        config_.goal_search_step_size *
        (static_cast<FloatingPoint>(i_x) -
         static_cast<FloatingPoint>(config_.goal_search_steps - 1) / 2.0);
    for (int i_y = 0; i_y < config_.goal_search_steps; ++i_y) {
      FloatingPoint y =
          config_.goal_search_step_size *
          (static_cast<FloatingPoint>(i_y) -
           static_cast<FloatingPoint>(config_.goal_search_steps - 1) / 2.0);
      for (int i_z = 0; i_z < config_.goal_search_steps; ++i_z) {
        FloatingPoint z =
            config_.goal_search_step_size *
            (static_cast<FloatingPoint>(i_z) -
             static_cast<FloatingPoint>(config_.goal_search_steps - 1) / 2.0);
        goal_search_offsets_.emplace_back(x, y, z);
      }
    }
  }
  std::sort(goal_search_offsets_.begin(), goal_search_offsets_.end(),
            [](const Point& lhs, const Point& rhs) {
              return lhs.norm() < rhs.norm();
            });
}

void SkeletonPlanner::executePlanningIteration() {
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

void SkeletonPlanner::resetPlanner() {
  stage_ = Stage::k1ComputeFrontiers;
  vis_data_.frontiers_have_changed = false;
  vis_data_.execution_finished = false;
  vis_data_.finished_successfully = false;
}

bool SkeletonPlanner::computeFrontiers() {
  // Guarantee tha all frontiers are computed and update them to the current
  // state. If they are already pre-computed and frozen the computation step
  // will do nothing.
  vis_data_.frontiers_have_changed = true;
  std::vector<MapBase::SubmapData> data = comm_->map()->getAllSubmapData();

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
    // NOTE: The submap origin is in free space since it corresponds
    //       to a robot pose by construction.
    const Point submap_origin(0.0, 0.0, 0.0);
    computeFrontiersForSubmap(datum, /* initial_point= */ submap_origin);
    update_list[datum.id] = datum.T_M_S;
  }
  updateFrontiers(update_list);

  // Check there are still frontiers left.
  bool active_frontier_left = false;
  for (const auto& frontier_collection : getUpdatedCollections()) {
    if (!frontier_collection.second.getActiveFrontiers().empty()) {
      active_frontier_left = true;
      break;
    }
  }
  if (!active_frontier_left) {
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
  auto t_start = std::chrono::high_resolution_clock::now();

  // Get all frontiers.
  frontier_data_.clear();
  for (const auto& frontier_collection : getUpdatedCollections()) {
    for (const auto& frontier :
         frontier_collection.second.getActiveFrontiers()) {
      FrontierSearchData& data = frontier_data_.emplace_back();
      data.centroid = frontier->getCentroid();
      data.num_points = frontier->getNumberOfActivePoints();
    }
  }
  if (frontier_data_.empty()) {
    LOG(WARNING) << "No active frontiers found to compute goal points from.";
    return false;
  }

  // Frontier clustering.
  if (config_.use_centroid_clustering) {
    clusterFrontiers();
  }

  // Try to find feasible goal near centroid and remove infeasible frontiers.
  int unreachable_goal_counter = 0;
  const int total_frontiers = frontier_data_.size();
  const Point current_position = comm_->currentPose().position();
  for (auto& frontier : frontier_data_) {
    if (!findValidGoalPoint(&(frontier.centroid))) {
      unreachable_goal_counter++;
      frontier.reachability = FrontierSearchData::kInvalidGoal;
      frontier.euclidean_distance = std::numeric_limits<double>::max();
    } else {
      frontier.euclidean_distance =
          (current_position - frontier.centroid).norm();
    }
  }

  // Compute paths to frontiers to determine the closest reachable one. Start
  // with closest and use euclidean distance as lower bound to prune candidates.
  std::sort(frontier_data_.begin(), frontier_data_.end(),
            [](const FrontierSearchData& lhs, const FrontierSearchData& rhs) {
              return lhs.euclidean_distance < rhs.euclidean_distance;
            });
  int path_counter = 0;
  double shortest_path = std::numeric_limits<double>::max();
  bool found_a_valid_path = false;
  for (auto& candidate : frontier_data_) {
    if (candidate.euclidean_distance >= shortest_path) {
      // These points can never be closer than what we already have.
      candidate.path_distance = candidate.euclidean_distance;
      candidate.reachability = FrontierSearchData::kUnchecked;
    } else {
      // Try to find a path via linked skeleton planning.
      path_counter++;
      std::vector<WayPoint> way_points;
      if (computePath(candidate.centroid, &way_points)) {
        // Frontier is reachable, save path and compute path length.
        candidate.way_points = way_points;
        candidate.path_distance =
            (way_points[0].position() - current_position).norm();
        for (size_t i = 1; i < way_points.size(); ++i) {
          candidate.path_distance +=
              (way_points[i].position() - way_points[i - 1].position()).norm();
        }
        shortest_path = std::min(shortest_path, candidate.path_distance);
        candidate.reachability = FrontierSearchData::kReachable;
        found_a_valid_path = true;
      } else {
        // Inaccessible frontier.
        candidate.path_distance = std::numeric_limits<double>::max();
        candidate.reachability = FrontierSearchData::kUnreachable;
      }
    }
  }

  // Select result.
  if (!found_a_valid_path) {
    LOG_IF(INFO, config_.verbosity >= 2)
        << "No reachable frontier found, returning to local planning.";
    comm_->stateMachine()->signalLocalPlanning();
    vis_data_.execution_finished = true;
    return false;
  }
  auto best_path_it = std::min_element(
      frontier_data_.begin(), frontier_data_.end(),
      [](const FrontierSearchData& lhs, const FrontierSearchData& rhs) {
        return lhs.path_distance < rhs.path_distance;
      });
  way_points_ = best_path_it->way_points;

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

  ss = std::stringstream();
  ss << "Found a path of " << way_points_.size()
     << " waypoints to closest reachable frontier";
  if (config_.verbosity >= 3) {
    ss << " (" << std::fixed << std::setprecision(2)
       << best_path_it->path_distance << "m path, " << std::fixed
       << std::setprecision(2) << best_path_it->euclidean_distance
       << "m euclidean distance).";
  } else {
    ss << ".";
  }
  LOG_IF(INFO, config_.verbosity >= 2) << ss.str();
  return true;
}

void SkeletonPlanner::clusterFrontiers() {
  // Search all frontiers for nearby centroids and merge incrementally until all
  // centroids are further than 'centroid_clustering_radius' apart.
  const int num_frontiers = frontier_data_.size();
  for (auto it = frontier_data_.begin(); it != frontier_data_.end(); ++it) {
    auto it2 = it;
    it2++;
    while (it2 != frontier_data_.end()) {
      if ((it2->centroid - it->centroid).norm() <=
          config_.centroid_clustering_radius) {
        // Nearby frontier centroids are merged by weight.
        it->centroid =
            it->centroid * static_cast<FloatingPoint>(it->num_points) +
            it2->centroid * static_cast<FloatingPoint>(it2->num_points);
        it->num_points += it2->num_points;
        it->centroid /= static_cast<FloatingPoint>(it->num_points);
        it->clusters++;
        it2 = frontier_data_.erase(it2);
      } else {
        it2++;
      }
    }
  }
  // Logging.
  LOG_IF(INFO, config_.verbosity >= 3)
      << "Clustered " << num_frontiers - frontier_data_.size()
      << " frontier centroids (" << num_frontiers << "->"
      << frontier_data_.size() << ").";
}

void SkeletonPlanner::executeWayPoint() {
  if (comm_->targetIsReached()) {
    if (way_points_.empty()) {
      // Finished execution.
      comm_->stateMachine()->signalLocalPlanning();
      LOG_IF(INFO, config_.verbosity >= 2)
          << "Finished global path execution, switching to local planning.";
      vis_data_.execution_finished = true;
      vis_data_.finished_successfully = true;
    } else {
      // Request next way point.
      if (config_.use_path_verification) {
        if (!verifyNextWayPoints()) {
          // Might recompute the path, so execute it in the next iteration.
          return;
        }
      }

      // Send the next waypoint in the list.
      WayPoint& way_point = way_points_[0];
      way_point.yaw = std::atan2((way_point.y - comm_->currentPose().y),
                                 (way_point.x - comm_->currentPose().x));
      comm_->requestWayPoint(way_point);
      way_points_.erase(way_points_.begin());
    }
  }
}

bool SkeletonPlanner::verifyNextWayPoints() {
  // Connect as many waypoints as possible with a single line, all points
  // are checked in the sliding window map to guarantee safety.
  int waypoint_index = 0;
  Point goal;
  while (waypoint_index < way_points_.size()) {
    goal = way_points_[waypoint_index].position();
    if (lineIsIntraversableInSlidingWindowAt(&goal)) {
      break;
    } else {
      waypoint_index++;
    }
  }

  // If less than one segment is connected check for minimum distance.
  if (waypoint_index == 0) {
    Point current_position = comm_->currentPose().position();
    // TODO(schmluk): make this a param if we keep it.
    const double safety = 0.3;

    if ((current_position - goal).norm() >
        config_.path_verification_min_distance + safety) {
      // Insert intermediate goal s.t. path can be observed.
      WayPoint way_point;
      Point direction = goal - current_position;
      Point new_gaol =
          current_position + direction * (1.0 - safety / direction.norm());
      way_point.x = new_gaol.x();
      way_point.y = new_gaol.y();
      way_point.z = new_gaol.z();
      way_points_.insert(way_points_.begin(), way_point);
    } else {
      // The global path is no longer feasible, try to recompute it.
      goal = way_points_.back().position();
      bool found_a_new_path = true;
      if (computePath(goal, &way_points_)) {
        // Check the next step is now feasible.
        goal = way_points_[0].position();
        if ((current_position - goal).norm() <= 0.5) {
          // Since drone moves on the skeleton it's likely we sit on a waypoint.
          if (way_points_.size() > 1) {
            way_points_.erase(way_points_.begin());
            goal = way_points_[0].position();
          } else {
            found_a_new_path = false;
          }
        }
        if (lineIsIntraversableInSlidingWindowAt(&goal)) {
          if ((current_position - goal).norm() <
              config_.path_verification_min_distance + safety) {
            found_a_new_path = false;
          }
        }
      } else {
        found_a_new_path = false;
      }

      if (found_a_new_path) {
        LOG_IF(INFO, config_.verbosity >= 2)
            << "Global path became infeasible, was successfully recomputed.";
      } else {
        // The goal is inaccessible, return to local planning.
        comm_->stateMachine()->signalLocalPlanning();
        LOG_IF(INFO, config_.verbosity >= 2)
            << "Global path became infeasible, returning to local planning.";
        vis_data_.execution_finished = true;
      }
      return false;
    }
  } else {
    // The ith path was intraversible, remove previous way points.
    for (int i = 0; i < waypoint_index - 1; ++i) {
      way_points_.erase(way_points_.begin());
    }
  }
  return true;
}

bool SkeletonPlanner::lineIsIntraversableInSlidingWindowAt(Point* goal) {
  // Check for collision and return the distance [m] after which the line path
  // is no longer feasible. -1 If it is fully feasible.
  const Point start = comm_->currentPose().position();
  const int n_points =
      std::floor((start - *goal).norm() / comm_->map()->getVoxelSize()) + 1;
  const Point increment = (*goal - start) / static_cast<double>(n_points);
  for (int i = 1; i <= n_points; ++i) {
    if (!comm_->map()->isTraversableInActiveSubmap(
            start + static_cast<double>(i) * increment)) {
      *goal = start + static_cast<double>(i - 1) * increment;
      return true;
    }
  }
  return false;
}

bool SkeletonPlanner::findValidGoalPoint(Point* goal) {
  // Sample from a cube to find candidates.
  for (const Point& offset : goal_search_offsets_) {
    Point candidate = *goal + offset;
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
