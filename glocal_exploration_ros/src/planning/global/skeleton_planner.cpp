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
      SubmapFrontierEvaluator(config.submap_frontier_config, communicator),
      skeleton_a_star_(communicator) {
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
  const Point current_position = comm_->currentPose().position;
  for (auto& frontier : frontier_data_) {
    if (!findValidGoalPoint(&(frontier.centroid))) {
      unreachable_goal_counter++;
      frontier.reachability = FrontierSearchData::kInvalidGoal;
      frontier.euclidean_distance = std::numeric_limits<FloatingPoint>::max();
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
  FloatingPoint shortest_path = std::numeric_limits<FloatingPoint>::max();
  bool found_a_valid_path = false;
  bool time_exceeded = false;
  for (auto& candidate : frontier_data_) {
    constexpr int64_t kMaxClosestFrontierSearchingTimeSec = 25;
    if (!time_exceeded &&
        kMaxClosestFrontierSearchingTimeSec <
            std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::high_resolution_clock::now() - t_start)
                .count()) {
      LOG_IF(INFO, config_.verbosity >= 1)
          << "Maximum closest frontier searching time exceeded. Will continue "
             "with the frontiers we found so far.";
      time_exceeded = true;
    }

    if (time_exceeded || candidate.euclidean_distance >= shortest_path) {
      // These points can never be closer than what we already have.
      candidate.path_distance = std::numeric_limits<FloatingPoint>::max();
      candidate.reachability = FrontierSearchData::kUnchecked;
    } else {
      // Try to find a path via linked skeleton planning.
      path_counter++;
      std::vector<WayPoint> way_points;
      if (computePath(candidate.centroid, &way_points)) {
        // Frontier is reachable, save path and compute path length.
        candidate.way_points = way_points;
        candidate.path_distance =
            (way_points[0].position - current_position).norm();
        for (size_t i = 1; i < way_points.size(); ++i) {
          candidate.path_distance +=
              (way_points[i].position - way_points[i - 1].position).norm();
        }
        shortest_path = std::min(shortest_path, candidate.path_distance);
        candidate.reachability = FrontierSearchData::kReachable;
        found_a_valid_path = true;
      } else {
        // Inaccessible frontier.
        candidate.path_distance = std::numeric_limits<FloatingPoint>::max();
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
      way_point.yaw = std::atan2(
          (way_point.position.y() - comm_->currentPose().position.y()),
          (way_point.position.x() - comm_->currentPose().position.x()));
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
    if (!comm_->map()->isLineTraversableInActiveSubmap(
            comm_->currentPose().position, way_points_[waypoint_index].position,
            &goal)) {
      break;
    } else {
      waypoint_index++;
    }
  }

  // If less than one segment is connected check for minimum distance.
  if (waypoint_index == 0) {
    Point current_position = comm_->currentPose().position;
    // TODO(schmluk): make this a param if we keep it.
    const FloatingPoint safety = 0.3;

    if ((current_position - goal).norm() >
        config_.path_verification_min_distance + safety) {
      // Insert intermediate goal s.t. path can be observed.
      Point direction = goal - current_position;
      Point new_goal =
          current_position + direction * (1.f - safety / direction.norm());
      way_points_.insert(way_points_.begin(), WayPoint(new_goal, 0.f));
    } else {
      // The goal is inaccessible, return to local planning.
      comm_->stateMachine()->signalLocalPlanning();
      LOG_IF(INFO, config_.verbosity >= 2)
          << "Global path became infeasible, returning to local planning.";

      // Make sure we don't stop in intraversable space.
      Point free_position = comm_->currentPose().position;
      findNearbyTraversablePoint(&free_position);
      comm_->requestWayPoint(WayPoint(free_position, 0.f));

      vis_data_.execution_finished = true;
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

  Point start_point = comm_->currentPose().position;
  if (comm_->map()->isTraversableInActiveSubmap(start_point) ||
      findNearbyTraversablePoint(&start_point)) {
    // Compute path along skeletons
    return skeleton_a_star_.planPath(start_point, goal, way_points);
  }

  LOG(WARNING) << "Skeleton planner: The active submap is not traversable "
                  "at the current pose. Could not find a nearby valid "
                  "start position.";
  return false;
}

bool SkeletonPlanner::findNearbyTraversablePoint(Point* position) {
  CHECK_NOTNULL(position);
  const Point initial_position = *position;

  constexpr int kMaxNumSteps = 20;
  const std::shared_ptr<MapBase> map_ptr = comm_->map();
  // TODO(victorr): Get traversability radius from map interface
  const FloatingPoint traversability_radius = 1.f;

  FloatingPoint distance;
  Point gradient;
  int step_idx = 1;
  for (; step_idx < kMaxNumSteps; ++step_idx) {
    // Determine if we found a solution
    if (map_ptr->isTraversableInActiveSubmap(*position)) {
      LOG(INFO) << "Skeleton planner: Succesfully moved point from "
                   "intraversable initial position ("
                << initial_position.x() << ", " << initial_position.y() << ", "
                << initial_position.z() << ") to traversable start point ("
                << position->x() << ", " << position->y() << ", "
                << position->z() << "), after " << step_idx
                << " gradient ascent steps.";
      return true;
    }
    // Get the distance
    if (!map_ptr->getDistanceAndGradientAtPositionInActiveSubmap(
            *position, &distance, &gradient)) {
      LOG(WARNING) << "Failed to look up distance and gradient "
                      "information at: "
                   << position->transpose();
      return false;
    }
    // Take a step in the direction that maximizes the distance
    const FloatingPoint step_size =
        std::max(map_ptr->getVoxelSize(), traversability_radius - distance);
    *position += step_size * gradient;
  }
  return false;
}

}  // namespace glocal_exploration
