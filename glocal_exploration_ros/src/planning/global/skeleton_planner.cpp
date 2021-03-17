#include "glocal_exploration_ros/planning/global/skeleton_planner.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace glocal_exploration {

SkeletonPlanner::Config::Config() { setConfigName("SkeletonPlanner"); }

void SkeletonPlanner::Config::checkParams() const {
  checkParamGT(goal_search_steps, 0, "goal_search_steps");
  checkParamGE(max_closest_frontier_search_time_sec, 0,
               "max_closest_frontier_search_time_sec");
  checkParamGT(max_replan_attempts_to_chosen_frontier, 0,
               "max_replan_attempts_to_chosen_frontier");
  checkParamGT(sensor_vertical_fov_rad, 0.f, "sensor_vertical_fov_rad");
}

void SkeletonPlanner::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam("use_centroid_clustering", &use_centroid_clustering);
  rosParam("centroid_clustering_radius", &centroid_clustering_radius);
  rosParam("use_path_verification", &use_path_verification);
  rosParam("path_verification_min_distance", &path_verification_min_distance);
  rosParam("goal_search_steps", &goal_search_steps);
  rosParam("goal_search_step_size", &goal_search_step_size);
  rosParam("safety_distance", &safety_distance);
  rosParam(&submap_frontier_config);
  rosParam("max_replan_attempts_to_chosen_frontier",
           &max_replan_attempts_to_chosen_frontier);
  rosParam("max_closest_frontier_search_time_sec",
           &max_closest_frontier_search_time_sec);
  rosParam("sensor_vertical_fov_rad", &sensor_vertical_fov_rad);
  rosParam("backtracking_distance_m", &backtracking_distance_m);
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
  printField("safety_distance", safety_distance);
  printField("submap_frontier_config", submap_frontier_config);
  printField("max_closest_frontier_search_time_sec",
             max_closest_frontier_search_time_sec);
  printField("max_replan_attempts_to_chosen_frontier",
             max_replan_attempts_to_chosen_frontier);
  printField("sensor_vertical_fov_rad", sensor_vertical_fov_rad);
  printField("backtracking_distance_m", backtracking_distance_m);
}

SkeletonPlanner::SkeletonPlanner(
    const Config& config, const SkeletonAStar::Config& skeleton_a_star_config,
    std::shared_ptr<Communicator> communicator)
    : SubmapFrontierEvaluator(config.submap_frontier_config, communicator),
      num_replan_attempts_to_chosen_frontier_(0),
      is_backtracking_(false),
      config_(config.checkValid()),
      skeleton_a_star_(skeleton_a_star_config, communicator) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // NOTE(schmluk): The skeleton planner has an internal cblox server that
  //                subscribes to submaps and submap poses via ros. One needs to
  //                make sure these are remapped properly. The submaps are
  //                skeletonized upon reception.
  // Setup the skeleton planner.
  ros::NodeHandle nh_private(config_.nh_private_namespace);
  ros::NodeHandle nh(ros::names::parentNamespace(config_.nh_private_namespace));

  // Precompute goal search offsets (points on cube ordered by distance).
  goal_search_offsets_.reserve(std::pow(config_.goal_search_steps, 3));
  for (int i_x = 0; i_x < config_.goal_search_steps; ++i_x) {
    FloatingPoint x =
        config_.goal_search_step_size *
        (static_cast<FloatingPoint>(i_x) -
         static_cast<FloatingPoint>(config_.goal_search_steps - 1) / 2.f);
    for (int i_y = 0; i_y < config_.goal_search_steps; ++i_y) {
      FloatingPoint y =
          config_.goal_search_step_size *
          (static_cast<FloatingPoint>(i_y) -
           static_cast<FloatingPoint>(config_.goal_search_steps - 1) / 2.f);
      for (int i_z = 0; i_z < config_.goal_search_steps; ++i_z) {
        FloatingPoint z =
            config_.goal_search_step_size *
            (static_cast<FloatingPoint>(i_z) -
             static_cast<FloatingPoint>(config_.goal_search_steps - 1) / 2.f);
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

  if (stage_ == Stage::k1ComputeFrontiers) {
    // Compute and update all frontiers to current state.
    if (computeFrontiers()) {
      stage_ = Stage::k2ComputeGoalAndPath;
    }
  }
  if (stage_ == Stage::k2ComputeGoalAndPath) {
    // Select a frontier to move towards, including path generation.
    if (computeGoalPoint()) {
      stage_ = Stage::k3ExecutePath;
    }
  }
  if (stage_ == Stage::k3ExecutePath) {
    // Execute way points until finished, then switch back to local.
    executeWayPoint();
  }
}

std::vector<WayPoint> SkeletonPlanner::getWayPoints() const {
  std::vector<WayPoint> global_way_points;
  for (const RelativeWayPoint& submap_way_point : way_points_) {
    global_way_points.emplace_back(static_cast<WayPoint>(submap_way_point));
  }
  return global_way_points;
}

void SkeletonPlanner::resetPlanner() {
  stage_ = Stage::k1ComputeFrontiers;
  is_backtracking_ = false;
  vis_data_.frontiers_have_changed = false;
  vis_data_.execution_finished = false;
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
    vis_data_.execution_finished = true;
    return false;
  }

  // Update all frontiers.
  updateFrontiers(data);

  // Check there are still frontiers left.
  if (getActiveFrontiers().empty()) {
    // No more open frontiers, exploration is done.
    LOG_IF(INFO, config_.verbosity >= 1)
        << "No active frontiers remaining, exploration terminated "
           "successfully.";
    comm_->stateMachine()->signalFinished();
    return false;
  }
  return true;
}

bool SkeletonPlanner::computeGoalPoint() {
  is_backtracking_ = false;

  // Compute the frontier with the shortest path to it.
  auto t_start = std::chrono::high_resolution_clock::now();

  // Get all frontiers.
  frontier_data_.clear();
  for (const auto& frontier : getActiveFrontiers()) {
    FrontierSearchData& data = frontier_data_.emplace_back();
    // Compute centroids and number of points.
    for (const Point& point : frontier) {
      data.centroid += point;
      data.num_points++;
    }
    data.centroid /= data.num_points;
    // NOTE(schmluk): Currently copy the frontiers since it's not that many.
    data.frontier_points = frontier;
  }
  if (frontier_data_.empty()) {
    LOG(WARNING) << "No active frontiers found to compute goal points from.";
    vis_data_.execution_finished = true;
    return false;
  }

  // Frontier clustering.
  if (config_.use_centroid_clustering) {
    clusterFrontiers();
  }

  // Search the closest reachable frontier.
  const Point current_robot_position = comm_->currentPose().position;
  Point start_point = current_robot_position;
  int path_counter = 0;
  int unobservable_frontier_counter = 0;
  const int total_frontiers = frontier_data_.size();
  bool found_a_valid_path = false;
  std::vector<GlobalVertexId> start_vertex_candidates;
  if (searchSkeletonStartVertices(&start_point, &start_vertex_candidates)) {
    // Sort the frontiers by euclidean distance.
    for (auto& frontier : frontier_data_) {
      frontier.euclidean_distance =
          (current_robot_position - frontier.centroid).norm();
    }
    std::sort(frontier_data_.begin(), frontier_data_.end(),
              [](const FrontierSearchData& lhs, const FrontierSearchData& rhs) {
                return lhs.euclidean_distance < rhs.euclidean_distance;
              });

    // Compute paths to frontiers to determine the closest reachable one. Start
    // with closest and use euclidean distance as lower bound to prune
    // candidates.
    FloatingPoint shortest_path = std::numeric_limits<FloatingPoint>::max();
    bool time_exceeded = false;
    for (auto& candidate : frontier_data_) {
      if (!time_exceeded &&
          config_.max_closest_frontier_search_time_sec <
              std::chrono::duration_cast<std::chrono::seconds>(
                  std::chrono::high_resolution_clock::now() - t_start)
                  .count()) {
        LOG_IF(INFO, config_.verbosity >= 1)
            << "Maximum closest frontier searching time exceeded. Will "
               "continue with the frontiers we found so far.";
        time_exceeded = true;
      }

      if (time_exceeded || candidate.euclidean_distance >= shortest_path) {
        // These points can never be closer than what we already have.
        candidate.path_distance = std::numeric_limits<FloatingPoint>::max();
        candidate.reachability = FrontierSearchData::kUnchecked;
      } else {
        // Try to find a path via linked skeleton planning.
        path_counter++;
        std::vector<RelativeWayPoint> way_points;
        bool frontier_is_observable = false;
        if (computePathToFrontier(start_point, start_vertex_candidates,
                                  candidate.centroid, candidate.frontier_points,
                                  &way_points, &frontier_is_observable)) {
          // Frontier is reachable, save path and compute path length.
          candidate.way_points = way_points;
          candidate.path_distance =
              (way_points[0].getGlobalPosition() - current_robot_position)
                  .norm();
          for (size_t i = 1; i < way_points.size(); ++i) {
            candidate.path_distance += (way_points[i].getGlobalPosition() -
                                        way_points[i - 1].getGlobalPosition())
                                           .norm();
          }
          shortest_path = std::min(shortest_path, candidate.path_distance);
          candidate.reachability = FrontierSearchData::kReachable;
          found_a_valid_path = true;
        } else {
          // Inaccessible frontier.
          candidate.path_distance = std::numeric_limits<FloatingPoint>::max();
          if (frontier_is_observable) {
            candidate.reachability = FrontierSearchData::kUnreachable;
          } else {
            ++unobservable_frontier_counter;
            candidate.reachability = FrontierSearchData::kInvalidGoal;
          }
        }
      }
    }
  }

  // Select result.
  if (!found_a_valid_path) {
    // Backtrack if enabled.
    if (0.f < config_.backtracking_distance_m) {
      way_points_.clear();
      const std::vector<WayPoint> past_poses = comm_->map()->getPoseHistory();
      // Find the oldest pose history crumb that we can directly connect to.
      // This is mainly done to avoid traveling back and forth in subsequent
      // backtracking attempts.
      auto oldest_adjacent_crumb = past_poses.cend();
      const FloatingPoint traversability_radius =
          comm_->map()->getTraversabilityRadius();
      const FloatingPoint max_traversability_check_distance =
          2 * traversability_radius;
      const bool kOptimistic = true;
      for (auto past_pose = past_poses.cbegin(); past_pose != past_poses.cend();
           ++past_pose) {
        const FloatingPoint distance_to_past_pose =
            (past_pose->position - current_robot_position).norm();
        if (max_traversability_check_distance < distance_to_past_pose) {
          continue;
        }
        if (distance_to_past_pose < traversability_radius ||
            comm_->map()->isLineTraversableInActiveSubmap(
                current_robot_position, past_pose->position,
                traversability_radius, nullptr, kOptimistic)) {
          oldest_adjacent_crumb = past_pose;
          break;
        }
      }

      Point t_odom_previous_crumb = Point::Zero();
      const FloatingPoint min_offset = traversability_radius;
      for (auto past_pose = oldest_adjacent_crumb;
           past_poses.cbegin() <= past_pose; --past_pose) {
        const Point t_odom_current_crumb = past_pose->position;
        if ((t_odom_current_crumb - current_robot_position).norm() <
            config_.backtracking_distance_m) {
          if (min_offset <
              (t_odom_current_crumb - t_odom_previous_crumb).norm()) {
            const std::vector<SubmapId> overlapping_submaps =
                comm_->map()->getSubmapIdsAtPosition(t_odom_current_crumb);
            SubmapId youngest_submap_id = *std::max_element(
                overlapping_submaps.begin(), overlapping_submaps.end());
            SkeletonSubmap::ConstPtr youngest_submap_ptr =
                skeleton_a_star_.getSkeletonSubmapCollection()
                    .getSubmapConstPtrById(youngest_submap_id);
            if (youngest_submap_ptr) {
              const Point t_submap_current_crumb =
                  youngest_submap_ptr->getPose().inverse() *
                  t_odom_current_crumb;
              way_points_.emplace_back(RelativeWayPoint(
                  youngest_submap_ptr, t_submap_current_crumb));

              t_odom_previous_crumb = t_odom_current_crumb;
            } else {
              LOG(WARNING) << "Could not get pointer to submap with ID: "
                           << youngest_submap_id << ". Skipping past pose.";
            }
          }
        } else {
          // Reached the end of the backtracking range.
          break;
        }
      }
      if (!way_points_.empty()) {
        LOG_IF(WARNING, config_.verbosity >= 2)
            << "No reachable frontier found, backtracking "
            << config_.backtracking_distance_m << " meters.";
        num_replan_attempts_to_chosen_frontier_ = 0;
        is_backtracking_ = true;
        return true;
      } else {
        LOG_IF(WARNING, config_.verbosity >= 2)
            << "Backtracking is enabled, but generating the backtracking path "
               "failed.";
      }
    }
    // Otherwise just fall back to local planning.
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
  num_replan_attempts_to_chosen_frontier_ = 0;

  // Logging.
  auto t_end = std::chrono::high_resolution_clock::now();
  std::stringstream ss;
  ss << "Evaluated " << path_counter << " of " << total_frontiers
     << " global paths in "
     << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
            .count()
     << "ms.";
  if (config_.verbosity >= 4) {
    ss << " Skipped " << unobservable_frontier_counter
       << " unobservable frontiers.";
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
    } else {
      // Request next way point.
      if (config_.use_path_verification) {
        if (!verifyNextWayPoints()) {
          // Might recompute the path, so execute it in the next iteration.
          return;
        }
      }

      // Send the next waypoint in the list.
      auto way_point = static_cast<WayPoint>(way_points_[0]);
      way_point.yaw = std::atan2(
          (way_point.position.y() - comm_->currentPose().position.y()),
          (way_point.position.x() - comm_->currentPose().position.x()));
      comm_->requestWayPoint(way_point);
      way_points_.erase(way_points_.begin());
    }
  }
}

bool SkeletonPlanner::verifyNextWayPoints() {
  const Point current_position = comm_->currentPose().position;
  const FloatingPoint traversability_radius =
      comm_->map()->getTraversabilityRadius();

  // Start by checking if the current position is intraversable.
  if (!comm_->map()->isTraversableInActiveSubmap(
          current_position, traversability_radius, /* optimistic= */ true)) {
    LOG_IF(INFO, config_.verbosity >= 2)
        << "Current position is intraversable.";
    // Attempt find a nearby traversable point and move to it.
    Point free_position = comm_->currentPose().position;
    if (comm_->map()->findSafestNearbyPoint(traversability_radius,
                                            &free_position)) {
      LOG_IF(INFO, config_.verbosity >= 2) << "Moving to safest nearby point.";
      comm_->requestWayPoint(WayPoint(free_position, 0.f));
    } else {
      // Fall back to local planning.
      LOG_IF(INFO, config_.verbosity) << "Could not find nearby traversable "
                                         "point. Returning to local planning.";
      comm_->stateMachine()->signalLocalPlanning();
      vis_data_.execution_finished = true;
    }
    return false;
  }

  // Check if the coming segment is intraversable.
  Point last_traversable_point;
  if (!comm_->map()->isLineTraversableInActiveSubmap(
          current_position, way_points_[0].getGlobalPosition(),
          traversability_radius, &last_traversable_point,
          /* optimistic= */ true)) {
    LOG_IF(INFO, config_.verbosity >= 2)
        << "Next global path segment is intraversable.";
    // As a first remedy, try to find a new path to the current frontier.
    if (num_replan_attempts_to_chosen_frontier_ <
        config_.max_replan_attempts_to_chosen_frontier) {
      ++num_replan_attempts_to_chosen_frontier_;
      LOG(WARNING) << "Attempt nr" << num_replan_attempts_to_chosen_frontier_
                   << " to find a new global path to the current frontier.";
      const RelativeWayPoint& current_frontier_goal = way_points_.back();
      std::vector<RelativeWayPoint> new_way_points;
      if (computePath(current_frontier_goal.getGlobalPosition(),
                      &new_way_points)) {
        LOG(INFO) << "Found a new global path to the current frontier.";
        is_backtracking_ = false;
        way_points_ = new_way_points;
        return true;
      } else {
        LOG(INFO)
            << "Could not find a new global path to the current frontier.";
      }
    }

    // As a second remedy, try to move a bit ahead to refresh the active submap.
    if ((current_position - last_traversable_point).norm() >
        config_.path_verification_min_distance + config_.safety_distance) {
      // Insert intermediate goal s.t. path can be observed.
      LOG_IF(INFO, config_.verbosity >= 2)
          << "Moving a bit ahead to refresh the active submap.";
      const Point direction = (last_traversable_point - current_position);
      const FloatingPoint move_ahead_distance =
          std::min(direction.norm(), 1.f - config_.safety_distance);
      const Point new_goal =
          current_position + move_ahead_distance * direction.normalized();
      way_points_.insert(way_points_.begin(), RelativeWayPoint(new_goal));
      return false;
    }

    // Give up and return to local planning.
    LOG_IF(INFO, config_.verbosity >= 2) << "Returning to local planning.";
    comm_->stateMachine()->signalLocalPlanning();
    // Try to avoid stopping in intraversable space.
    Point free_position = comm_->currentPose().position;
    if (comm_->map()->findSafestNearbyPoint(traversability_radius,
                                            &free_position)) {
      comm_->requestWayPoint(WayPoint(free_position, 0.f));
    }
    vis_data_.execution_finished = true;
    return false;
  }

  // The next path segment is traversable.
  // Check if we can skip some of the coming waypoints, by connecting the
  // current pose to the nth waypoint with a straight line.
  int waypoint_index = 0;
  while (waypoint_index < way_points_.size()) {
    if (!comm_->map()->isLineTraversableInActiveSubmap(
            current_position,
            way_points_[waypoint_index].getGlobalPosition())) {
      break;
    } else {
      waypoint_index++;
    }
  }
  if (0 < waypoint_index) {
    // Remove the unecessary waypoints to shorten the path.
    for (int i = 0; i < waypoint_index - 1; ++i) {
      way_points_.erase(way_points_.begin());
    }
  }

  return true;
}

bool SkeletonPlanner::computePath(const Point& goal,
                                  std::vector<RelativeWayPoint>* way_points) {
  CHECK_NOTNULL(way_points);
  const FloatingPoint traversability_radius =
      skeleton_a_star_.getTraversabilityRadius();
  Point start_point = comm_->currentPose().position;
  const bool current_position_is_traversable =
      comm_->map()->isTraversableInActiveSubmap(start_point,
                                                traversability_radius);
  if (current_position_is_traversable ||
      comm_->map()->findSafestNearbyPoint(traversability_radius,
                                          &start_point)) {
    // Compute path along skeletons
    if (skeleton_a_star_.planPath(start_point, goal, way_points)) {
      // Set the start point to the safe point if applicable
      if (!current_position_is_traversable) {
        way_points->emplace(way_points->begin(), RelativeWayPoint(start_point));
      }
      return true;
    }
  }

  LOG(WARNING) << "Skeleton planner: The active submap is not traversable "
                  "at the current pose. Could not find a nearby valid "
                  "start position.";
  return false;
}

bool SkeletonPlanner::searchSkeletonStartVertices(
    Point* start_point, std::vector<GlobalVertexId>* start_vertex_candidates) {
  CHECK_NOTNULL(start_point);
  CHECK_NOTNULL(start_vertex_candidates);

  const std::shared_ptr<MapBase> map = comm_->map();
  const FloatingPoint traversability_radius =
      skeleton_a_star_.getTraversabilityRadius();

  // Ensure the start point is valid
  const bool current_position_is_intraversable =
      !map->isTraversableInActiveSubmap(*start_point, traversability_radius);
  if (current_position_is_intraversable) {
    if (!comm_->map()->findSafestNearbyPoint(traversability_radius,
                                             start_point)) {
      LOG(WARNING) << "Skeleton planner: The active submap is not traversable "
                      "at the current pose. Could not find a nearby safe "
                      "start position.";
      return false;
    }
  }

  // Search the N nearest reachable start vertices on the skeleton graphs.
  // First try to find vertices that are reachable in the active submap.
  *start_vertex_candidates =
      skeleton_a_star_.searchClosestReachableSkeletonVertices(
          *start_point,
          skeleton_a_star_.getConfig().max_num_start_vertex_candidates,
          [&](const Point& start_point, const Point& end_point) {
            return map->isLineTraversableInActiveSubmap(start_point, end_point,
                                                        traversability_radius);
          });
  // If this fails, search vertices that are reachable in the global map to give
  // the path follower a chance to try to reach them.
  if (start_vertex_candidates->empty()) {
    *start_vertex_candidates =
        skeleton_a_star_.searchClosestReachableSkeletonVertices(
            *start_point,
            skeleton_a_star_.getConfig().max_num_start_vertex_candidates,
            [&](const Point& start_point, const Point& end_point) {
              return map->isLineTraversableInGlobalMap(start_point, end_point,
                                                       traversability_radius);
            });
    if (start_vertex_candidates->empty()) {
      LOG(INFO)
          << "Could not find any reachable skeleton vertices near start point ("
          << start_point->transpose() << "). Both in the local and global map.";
      return false;
    }
  }

  return true;
}

bool SkeletonPlanner::computePathToFrontier(
    const Point& start_point,
    const std::vector<GlobalVertexId>& start_vertex_candidates,
    const Point& frontier_centroid, const std::vector<Point>& frontier_points,
    std::vector<RelativeWayPoint>* way_points, bool* frontier_is_observable) {
  CHECK_NOTNULL(way_points);

  // Search the N skeleton vertices that are closest to the frontier centroid,
  // and from which at least M frontier points can be observed.
  std::vector<GlobalVertexId> end_vertex_candidates =
      skeleton_a_star_.searchClosestReachableSkeletonVertices(
          frontier_centroid,
          skeleton_a_star_.getConfig().max_num_end_vertex_candidates,
          [&](const Point& point, const Point& skeleton_vertex_point) {
            int num_visible_frontier_points = 0;
            for (const Point& frontier_point : frontier_points) {
              if (isFrontierPointObservableFromPosition(
                      frontier_point, skeleton_vertex_point)) {
                if (SubmapFrontierEvaluator::config_
                        .min_num_visible_frontier_points <
                    ++num_visible_frontier_points) {
                  return true;
                }
              }
            }
            return false;
          });
  if (end_vertex_candidates.empty()) {
    LOG(INFO) << "Could not find any skeleton vertices from which the frontier "
                 "with centroid ("
              << frontier_centroid.transpose() << ") can be observed.";
    if (frontier_is_observable != nullptr) {
      *frontier_is_observable = false;
    }
    return false;
  } else {
    if (frontier_is_observable != nullptr) {
      *frontier_is_observable = true;
    }
  }

  // Plan path along the skeleton
  std::vector<GlobalVertexId> vertex_path;
  if (!skeleton_a_star_.getPathBetweenVertices(
          start_vertex_candidates, end_vertex_candidates, start_point,
          frontier_centroid, &vertex_path)) {
    LOG(INFO) << "Could not find global path from start point ("
              << start_point.transpose() << ") to frontier with centroid ("
              << frontier_centroid.transpose() << ")";
    return false;
  }

  // Convert the path from vertex IDs to waypoints
  skeleton_a_star_.convertVertexToWaypointPath(vertex_path, frontier_centroid,
                                               way_points);
  // Add the start point to the plan if it differs from the current position
  if (comm_->map()->getTraversabilityRadius() <
      (start_point - comm_->currentPose().position).norm()) {
    way_points->emplace(way_points->begin(), RelativeWayPoint(start_point));
  }
  // Set the last waypoint to a safe point between the last skeleton vertex
  // and the frontier centroid.
  way_points->pop_back();
  if (!way_points->empty()) {
    const RelativeWayPoint& last_vertex_waypoint = way_points->back();
    Point t_odom_last_traversable_point;
    comm_->map()->isLineTraversableInGlobalMap(
        last_vertex_waypoint.getGlobalPosition(), frontier_centroid,
        skeleton_a_star_.getTraversabilityRadius(),
        &t_odom_last_traversable_point);
    SkeletonSubmap::ConstPtr last_vertex_submap_ptr =
        skeleton_a_star_.getSkeletonSubmapCollection().getSubmapConstPtrById(
            last_vertex_waypoint.getFrameId());
    const Point t_submap_last_traversable_point =
        last_vertex_submap_ptr->getPose().inverse() *
        t_odom_last_traversable_point;
    way_points->emplace_back(RelativeWayPoint(last_vertex_submap_ptr,
                                              t_submap_last_traversable_point));
  }

  return true;
}

bool SkeletonPlanner::isFrontierPointObservableFromPosition(
    const Point& frontier_point, const Point& skeleton_vertex_point) {
  // Check if the frontier point is within the LiDAR's FoV.
  const FloatingPoint vertical_offset =
      frontier_point.z() - skeleton_vertex_point.z();
  const FloatingPoint horizontal_offset =
      (frontier_point - skeleton_vertex_point).head<2>().norm();
  if (config_.sensor_vertical_fov_rad / 2.f <
      std::abs(std::atan2(vertical_offset, horizontal_offset))) {
    return false;
  }

  // Check for occlusions.
  return !comm_->map()->lineIntersectsSurfaceInGlobalMap(skeleton_vertex_point,
                                                         frontier_point);
}

}  // namespace glocal_exploration
