#include "glocal_exploration/planning/local/rh_rrt_star.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <unordered_set>
#include <utility>
#include <vector>

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>

#include "glocal_exploration/state/communicator.h"

namespace glocal_exploration {

RHRRTStar::Config::Config() { setConfigName("RHRRTStar"); }

void RHRRTStar::Config::checkParams() const {
  checkParamGT(max_path_length, 0.f, "max_path_length");
  checkParamGE(path_cropping_length, 0.f, "path_cropping_length");
  checkParamGT(traversability_radius, 0.f, "traversability_radius");
  checkParamGT(max_number_of_neighbors, 0, "max_number_of_neighbors");
  checkParamGT(maximum_rewiring_iterations, 0, "maximum_rewiring_iterations");
  checkParamGT(sampling_range, 0.f, "sampling_range");
  checkParamGE(min_path_length, 0.f, "min_path_length");
  checkParamGE(min_sampling_distance, 0.f, "min_sampling_distance");
  checkParamGE(terminaton_min_tree_size, 0, "terminaton_min_tree_size");
  checkParamGE(termination_max_gain, 0.f, "termination_max_gain");
  checkParamGE(reconsideration_time, 0.f, "reconsideration_time");
  checkParamConfig(lidar_config);
}

void RHRRTStar::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam("sampling_range", &sampling_range);
  rosParam("min_path_length", &min_path_length);
  rosParam("min_sampling_distance", &min_sampling_distance);
  rosParam("max_path_length", &max_path_length);
  rosParam("path_cropping_length", &path_cropping_length);
  rosParam("traversability_radius", &traversability_radius);
  rosParam("max_number_of_neighbors", &max_number_of_neighbors);
  rosParam("maximum_rewiring_iterations", &maximum_rewiring_iterations);
  rosParam("terminaton_min_tree_size", &terminaton_min_tree_size);
  rosParam("termination_max_gain", &termination_max_gain);
  rosParam("reconsideration_time", &reconsideration_time);
  rosParam("DEBUG_number_of_iterations", &DEBUG_number_of_iterations);
  rosParam(&lidar_config);
}

void RHRRTStar::Config::printFields() const {
  printField("verbosity", verbosity);
  printField("sampling_range", sampling_range);
  printField("min_path_length", min_path_length);
  printField("min_sampling_distance", min_sampling_distance);
  printField("max_path_length", max_path_length);
  printField("path_cropping_length", path_cropping_length);
  printField("traversability_radius", traversability_radius);
  printField("max_number_of_neighbors", max_number_of_neighbors);
  printField("maximum_rewiring_iterations", maximum_rewiring_iterations);
  printField("terminaton_min_tree_size", terminaton_min_tree_size);
  printField("termination_max_gain", termination_max_gain);
  printField("reconsideration_time", reconsideration_time);
  printField("DEBUG_number_of_iterations", DEBUG_number_of_iterations);
  printField("lidar_config", lidar_config);
}

RHRRTStar::RHRRTStar(const Config& config,
                     std::shared_ptr<Communicator> communicator)
    : LocalPlannerBase(std::move(communicator)), config_(config.checkValid()) {
  // Initialize the sensor model.
  sensor_model_ = std::make_unique<LidarModel>(config_.lidar_config, comm_);
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" + config_.toString();
}

void RHRRTStar::executePlanningIteration() {
  // Newly started local planning.
  if (comm_->stateMachine()->previousState() !=
      StateMachine::State::kLocalPlanning) {
    resetPlanner(comm_->currentPose());
    comm_->stateMachine()->signalLocalPlanning();
  }

  // Requested a view point so update.
  if (gain_update_needed_) {
    updateGains();
    gain_update_needed_ = false;
  }

  // expansion step
  expandTree();

  // Goal reached: request next point if there is a valid candidate
  if (comm_->targetIsReached()) {
    // If this param is set exactly this number of steps are executed before
    // switching to global.
    if (config_.DEBUG_number_of_iterations > 0) {
      number_of_executed_waypoints_++;
      if (number_of_executed_waypoints_ >= config_.DEBUG_number_of_iterations) {
        comm_->stateMachine()->signalGlobalPlanning();
        return;
      }
    } else if (isTerminationCriterionMet()) {
      // Check whether a local minimum is reached and change to global planning.
      if (config_.reconsideration_time > 0.f) {
        LOG_IF(INFO, config_.verbosity >= 3)
            << "Termination criterion is met, trying to overcome it for "
            << config_.reconsideration_time
            << "s before moving to global planning.";
        sampleReconsideration();
      }
      if (isTerminationCriterionMet()) {
        comm_->stateMachine()->signalGlobalPlanning();
        return;
      }
    }

    // Select the next point to go to from local planning.
    updateCollision();
    WayPoint next_waypoint;
    if (selectNextBestWayPoint(&next_waypoint)) {
      comm_->requestWayPoint(next_waypoint);
      gain_update_needed_ = true;
    }
  }
}

void RHRRTStar::sampleReconsideration() {
  // Just try to expand the tree for that amount of time.
  auto t_start = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now() - t_start)
             .count() <= config_.reconsideration_time * 1000.f) {
    expandTree();
  }
}

bool RHRRTStar::isTerminationCriterionMet() {
  // Check min tree size.
  if (tree_data_.points.size() < config_.terminaton_min_tree_size) {
    return false;
  }

  // Check minimum gain (not value!)
  for (const auto& point : tree_data_.points) {
    if (point->gain > config_.termination_max_gain) {
      return false;
    }
  }
  return true;
}

void RHRRTStar::resetPlanner(const WayPoint& new_origin) {
  // clear the tree and initialize with a point at the current pose
  tree_data_.points.clear();
  auto point = std::make_unique<ViewPoint>();
  point->pose = new_origin;
  point->is_root = true;
  kdtree_ = std::make_unique<KDTree>(3, tree_data_);
  tree_data_.points.push_back(std::move(point));
  kdtree_->addPoints(0, 0);

  // reset counters
  root_ = tree_data_.points[0].get();
  previous_view_point_ = nullptr;
  current_connection_ = nullptr;
  gain_update_needed_ = false;
  pruned_points_ = 0;
  new_points_ = 0;
  reconsidered_ = false;
  number_of_executed_waypoints_ = 0;

  // Logging.
  LOG_IF(INFO, config_.verbosity >= 4) << "Reset the RH-RRT* planner.";
}

void RHRRTStar::expandTree() {
  auto new_point = std::make_unique<ViewPoint>();
  // sample a goal pose
  if (!sampleNewPoint(new_point.get())) {
    return;
  }

  // establish connections to nearby neighbors (at least 1 should be guaranteed
  // by the sampling procedure)
  if (!connectViewPoint(new_point.get())) {
    return;
  }

  // evaluate the gain of the point
  evaluateViewPoint(new_point.get());

  // Add it to the kdtree
  tree_data_.points.push_back(std::move(new_point));
  kdtree_->addPoints(tree_data_.points.size() - 1,
                     tree_data_.points.size() - 1);

  new_points_++;
}

bool RHRRTStar::selectNextBestWayPoint(WayPoint* next_waypoint) {
  if (tree_data_.points.size() < 2) {
    return false;
  }

  // Optimize the tree.
  size_t next_point_index;
  optimizeTreeAndFindBestGoal(&next_point_index);

  // Check whether we're about to reverse unforced (in intraversable areas the
  // tree size will be =2, always reverse then).
  if (config_.reconsideration_time > 0.f &&
      root_->getConnectedViewPoint(next_point_index) == previous_view_point_ &&
      tree_data_.points.size() > 2) {
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Planner is about to reverse, trying to overcome it for "
        << config_.reconsideration_time << "s before moving backwards.";
    sampleReconsideration();
    optimizeTreeAndFindBestGoal(&next_point_index);
  }

  // result
  *next_waypoint = root_->getConnectedViewPoint(next_point_index)->pose;
  previous_view_point_ = root_;

  // update the roots
  ViewPoint* new_root = root_->getConnectedViewPoint(next_point_index);
  root_->is_root = false;
  new_root->is_root = true;
  root_->setActiveConnection(next_point_index);  // make the old root connect
                                                 // to the new root
  current_connection_ = root_->getConnections()[next_point_index].second.get();
  root_ = new_root;

  // logging
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Published next segment: " << new_points_ << " new, " << pruned_points_
      << " killed, " << tree_data_.points.size() << " total.";
  pruned_points_ = 0;
  new_points_ = 0;

  return true;
}

bool RHRRTStar::optimizeTreeAndFindBestGoal(size_t* next_root_idx) {
  // set up
  CHECK_NOTNULL(next_root_idx);
  int iterations = 0;
  auto t_start = std::chrono::high_resolution_clock::now();

  // Optimize the tree structure
  while (iterations++ < config_.maximum_rewiring_iterations) {
    bool something_changed = false;
    for (auto& view_point : tree_data_.points) {
      if (!view_point->is_root) {
        // optimize local connections
        size_t previous_index = view_point->getActiveConnectionIndex();
        selectBestConnection(view_point.get());
        if (view_point->getActiveConnectionIndex() != previous_index) {
          something_changed = true;
        }
      }
    }
    if (!something_changed) {
      break;
    }
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  LOG_IF(INFO, config_.verbosity >= 3)
      << "Optimized the tree in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << "ms, " << iterations << " iterations.";

  // select the best node from the current root
  int next_point_idx = -1;
  FloatingPoint best_value = -std::numeric_limits<FloatingPoint>::max();
  for (size_t i = 0; i < root_->getConnections().size(); ++i) {
    const ViewPoint* target = root_->getConnectedViewPoint(i);
    if (target->getActiveViewPoint() == root_) {
      // the candidate is wired to the root
      if (target->value > best_value) {
        best_value = target->value;
        next_point_idx = i;
      }
    }
  }
  if (next_point_idx < 0) {
    // This should never happen as the previous segment should remain active.
    return false;
  } else {
    *next_root_idx = next_point_idx;
    return true;
  }
}

void RHRRTStar::updateCollision() {
  int num_previous_points = tree_data_.points.size();
  // update all connections
  for (auto& viewpoint : tree_data_.points) {
    int offset = 0;
    for (int _i = 0; _i < viewpoint->getConnections().size(); ++_i) {
      int i = _i + offset;

      // Update every connection only once (by the parent).
      if (viewpoint->getConnections()[i].first) {
        Connection* connection = viewpoint->getConnection(i);
        if (connection == current_connection_) {
          // don't update the currently executed connection, this always allows
          // backtracking as well.
          continue;
        }

        // Remove far away and colliding connections.
        Point position = comm_->currentPose().position;
        if ((connection->getTarget()->pose.position - position).norm() >=
                config_.sampling_range ||
            (connection->getParent()->pose.position - position).norm() >=
                config_.sampling_range ||
            !comm_->map()->isLineTraversableInActiveSubmap(
                connection->getParent()->pose.position,
                connection->getTarget()->pose.position,
                config_.traversability_radius)) {
          viewpoint->removeConnection(connection);
          offset--;
        }
      }
    }
  }

  // Remove view_points that don't have a connection to the root anymore.
  computePointsConnectedToRoot(false);
  tree_data_.points.erase(
      std::remove_if(tree_data_.points.begin(), tree_data_.points.end(),
                     [](std::unique_ptr<ViewPoint>& v) {
                       return !v->is_connected_to_root;
                     }),
      tree_data_.points.end());

  // reset the kdtree
  kdtree_ = std::make_unique<KDTree>(3, tree_data_);
  kdtree_->addPoints(0, tree_data_.points.size() - 1);

  // Set active connections to form a tree again.
  computePointsConnectedToRoot(true);
  std::queue<ViewPoint*> not_connected;
  for (auto& vp : tree_data_.points) {
    if (!vp->is_connected_to_root) {
      not_connected.push(vp.get());
    }
  }
  while (!not_connected.empty()) {
    ViewPoint* current = not_connected.front();
    not_connected.pop();
    for (size_t i = 0; i < current->getConnections().size(); ++i) {
      if (current->getConnectedViewPoint(i)->is_connected_to_root) {
        current->setActiveConnection(i);
        current->is_connected_to_root = true;
        break;
      }
    }
    if (!current->is_connected_to_root) {
      not_connected.push(current);
    }
  }

  // track stats
  pruned_points_ += num_previous_points - tree_data_.points.size();
}

void RHRRTStar::computePointsConnectedToRoot(
    bool count_only_active_connections) {
  // Sets the is_connected_to_root flag for the entire tree
  std::queue<ViewPoint*> points_to_check;

  // setup
  for (auto& vp : tree_data_.points) {
    if (vp->is_root) {
      points_to_check.push(vp.get());
      vp->is_connected_to_root = true;
    } else {
      vp->is_connected_to_root = false;
    }
  }

  // breadth first search
  while (!points_to_check.empty()) {
    if (count_only_active_connections) {
      // label all children (active connection) as connected.
      for (auto& child : points_to_check.front()->getChildViewPoints()) {
        if (!child->is_connected_to_root) {
          points_to_check.push(child);
          child->is_connected_to_root = true;
        }
      }
    } else {
      // Label all connected points as connected.
      for (size_t i = 0; i < points_to_check.front()->getConnections().size();
           ++i) {
        ViewPoint* connected_vp =
            points_to_check.front()->getConnectedViewPoint(i);
        if (!connected_vp->is_connected_to_root) {
          points_to_check.push(connected_vp);
          connected_vp->is_connected_to_root = true;
        }
      }
    }
    points_to_check.pop();
  }
}

void RHRRTStar::updateGains() {
  auto t_start = std::chrono::high_resolution_clock::now();

  // update all relevant points
  for (auto& point : tree_data_.points) {
    if (point->getActiveConnection() == current_connection_) {
      // don't update the old or new root
      point->gain = 0.f;
      continue;
    }
    evaluateViewPoint(point.get());
  }

  // logging
  auto t_end = std::chrono::high_resolution_clock::now();
  LOG_IF(INFO, config_.verbosity >= 3)
      << "Updated all gains in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start)
             .count()
      << "ms.";
}

bool RHRRTStar::connectViewPoint(ViewPoint* view_point) {
  // This method is called on newly sampled points, so they can not look up
  // themselves or duplicate connections
  std::vector<size_t> nearest_viewpoints;
  if (!findNearestNeighbors(view_point->pose.position, &nearest_viewpoints,
                            config_.max_number_of_neighbors)) {
    return false;
  }
  bool connection_found = false;
  for (const auto& index : nearest_viewpoints) {
    FloatingPoint distance =
        (view_point->pose.position - tree_data_.points[index]->pose.position)
            .norm();
    if (distance > config_.max_path_length ||
        distance < config_.min_path_length) {
      continue;
    }
    Connection* new_connection = view_point->addConnection(
        tree_data_.points[index].get(), comm_->map().get());
    if (new_connection) {
      new_connection->cost = computeCost(*new_connection);
      connection_found = true;
    }
  }
  return connection_found;
}

bool RHRRTStar::selectBestConnection(ViewPoint* view_point) {
  // This operation is an iteration step to optimize the tree structure.
  if (view_point->getConnections().empty() || view_point->is_root) {
    return false;
  }
  FloatingPoint best_value = std::numeric_limits<FloatingPoint>::min();
  int best_connection = -1;
  for (size_t i = 0; i < view_point->getConnections().size(); ++i) {
    // Make sure there are no loops in the tree.
    bool is_loop = false;
    ViewPoint* current = view_point->getConnectedViewPoint(i);
    // NOTE(schmluk): Iteration counting is currently a back up to detect
    // detached segments and prevent the system from getting stuck.
    int it = tree_data_.points.size();
    while (!current->is_root) {
      it--;
      current = current->getActiveViewPoint();
      if (current == view_point) {
        // This is a loop of candidates, which is valid but cannot be activated.
        is_loop = true;
        break;
      }
      if (it < 0) {
        // Found a loop of active connections, which invalid.
        is_loop = true;
        LOG(WARNING) << "Found a infinite loop searching the tree root: "
                     << current;
        break;
      }
    }
    if (is_loop) {
      continue;
    }

    // compute the value
    view_point->setActiveConnection(i);
    computeValue(view_point);
    if (view_point->value > best_value) {
      best_value = view_point->value;
      best_connection = i;
    }
  }
  if (best_connection < 0) {
    return false;
  }

  // apply the result
  view_point->value = best_value;
  view_point->setActiveConnection(best_connection);
  return true;
}

void RHRRTStar::evaluateViewPoint(ViewPoint* view_point) {
  voxblox::LongIndexSet voxels;
  sensor_model_->getVisibleUnknownVoxelsAndOptimalYaw(&view_point->pose,
                                                      &voxels);
  view_point->gain = voxels.size();
}

FloatingPoint RHRRTStar::computeCost(const Connection& connection) {
  // just use distance
  return (connection.getParent()->pose.position -
          connection.getTarget()->pose.position)
      .norm();
}

void RHRRTStar::computeValue(ViewPoint* view_point) {
  if (view_point->is_root) {
    view_point->value = 0.f;
    return;
  }
  FloatingPoint gain = 0.f;
  FloatingPoint cost = 0.f;
  ViewPoint* current = view_point;
  while (true) {
    // propagate the new value up to the root
    current = current->getActiveViewPoint();
    if (current->is_root) {
      break;
    } else {
      gain += current->gain;
      cost += current->getActiveConnection()->cost;
    }
  }
  // propagate recursively to the leaves
  std::unordered_set<ViewPoint*> visited;
  view_point->value = computeGNVStep(view_point, gain, cost, &visited);
}

FloatingPoint RHRRTStar::computeGNVStep(
    ViewPoint* view_point, FloatingPoint gain, FloatingPoint cost,
    std::unordered_set<ViewPoint*>* visited) {
  // recursively iterate towards leaf, then iterate backwards and select best
  // value of children.
  FloatingPoint value = 0.f;
  gain += view_point->gain;
  cost += view_point->getActiveConnection()->cost;
  if (cost > 0.f) {
    value = gain / cost;
  }

  // If we find a loop it cannot be more effective.
  if (visited->find(view_point) != visited->end()) {
    return value;
  }
  visited->insert(view_point);

  for (ViewPoint* child : view_point->getChildViewPoints()) {
    if (!child->is_root) {
      value = std::max(value, computeGNVStep(child, gain, cost, visited));
    }
  }
  return value;
}

bool RHRRTStar::sampleNewPoint(ViewPoint* point) {
  // Sample the goal point.

  const FloatingPoint theta = 2.f * M_PI *
                              static_cast<FloatingPoint>(std::rand()) /
                              static_cast<FloatingPoint>(RAND_MAX);
  const FloatingPoint phi =
      acos(1.f - 2.f * static_cast<FloatingPoint>(std::rand()) /
                     static_cast<FloatingPoint>(RAND_MAX));
  const Point direction =
      Point(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
  Point goal =
      comm_->currentPose().position + config_.sampling_range * direction;

  // Find the nearest neighbor.
  std::vector<size_t> nearest_viewpoint;
  if (!findNearestNeighbors(goal, &nearest_viewpoint)) {
    return false;
  }
  Point origin = tree_data_.points[nearest_viewpoint.front()]->pose.position;
  FloatingPoint distance_max =
      std::min((goal - origin).norm(), config_.max_path_length);
  if (distance_max < config_.min_sampling_distance) {
    return false;
  }

  // Verify and crop the sampled path.
  goal = origin + direction * (distance_max + config_.path_cropping_length);
  Point goal_cropped;
  comm_->map()->isLineTraversableInActiveSubmap(
      origin, goal, config_.traversability_radius, &goal_cropped);
  // Substract a safety interval.
  const FloatingPoint distance =
      (goal_cropped - origin).norm() - config_.path_cropping_length;
  if (distance < config_.min_sampling_distance) {
    return false;
  }
  goal_cropped = origin + direction * distance;

  // Check min distance.
  if (!findNearestNeighbors(goal_cropped, &nearest_viewpoint)) {
    return false;
  }
  if ((tree_data_.points[nearest_viewpoint.front()]->pose.position -
       goal_cropped)
          .norm() < config_.min_sampling_distance) {
    return false;
  }

  // Write the result.
  point->pose.position = goal_cropped;
  point->pose.yaw = 2.f * M_PI * static_cast<FloatingPoint>(std::rand()) /
                    static_cast<FloatingPoint>(RAND_MAX);
  return true;
}

bool RHRRTStar::findNearestNeighbors(const Point& position,
                                     std::vector<size_t>* result,
                                     int n_neighbors) {
  // how to use nanoflann (:
  // Returns the indices of the neighbors in tree data.
  FloatingPoint query_pt[3] = {position.x(), position.y(), position.z()};
  std::size_t ret_index[n_neighbors];   // NOLINT
  FloatingPoint out_dist[n_neighbors];  // NOLINT
  nanoflann::KNNResultSet<FloatingPoint> resultSet(n_neighbors);
  resultSet.init(ret_index, out_dist);
  kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
  if (resultSet.size() == 0) {
    return false;
  }
  result->clear();
  result->reserve(resultSet.size());
  for (int i = 0; i < resultSet.size(); ++i) {
    result->push_back(ret_index[i]);
  }
  return true;
}

void RHRRTStar::visualizeGain(const WayPoint& pose, std::vector<Point>* voxels,
                              std::vector<Point>* colors,
                              FloatingPoint* scale) const {
  CHECK_NOTNULL(voxels);
  CHECK_NOTNULL(colors);
  CHECK_NOTNULL(scale);
  // NOTE(schmluk): Simply re-compute the view gains for visualization.
  // This is neither beautiful nor efficient but it doesn't get called often.

  // get voxel indices
  voxblox::LongIndexSet voxels_idx;
  sensor_model_->getVisibleUnknownVoxels(pose, &voxels_idx);

  // voxel size
  *scale = comm_->map()->getVoxelSize();
  const FloatingPoint voxel_size = comm_->map()->getVoxelSize();

  // get centers
  voxels->clear();
  voxels->reserve(voxels_idx.size());
  for (const auto& idx : voxels_idx) {
    voxels->push_back(voxblox::getCenterPointFromGridIndex(idx, voxel_size));
  }

  // Uniform coloring [0, 1].
  colors->assign(voxels->size(), Point(1, 0.8, 0));
}

RHRRTStar::Connection* RHRRTStar::ViewPoint::addConnection(ViewPoint* target,
                                                           MapBase* map) {
  // Check for duplicates.
  for (size_t i = 0; i < connections.size(); ++i) {
    if (getConnectedViewPoint(i) == this ||
        getConnectedViewPoint(i) == target) {
      return nullptr;
      LOG(WARNING) << "Tried to add a duplicate to the tree!";
    }
  }

  // Check traversability.
  if (!map->isLineTraversableInActiveSubmap(pose.position,
                                            target->pose.position)) {
    return nullptr;
  }

  // Add the connection.
  auto connection = std::make_shared<Connection>();
  connection->parent = this;
  connection->target = target;
  connections.emplace_back(std::make_pair(true, connection));
  target->connections.emplace_back(std::make_pair(false, connection));
  return connection.get();
}

RHRRTStar::Connection* RHRRTStar::ViewPoint::getActiveConnection() {
  if (connections.size() <= active_connection) {
    return nullptr;
  }
  return connections[active_connection].second.get();
}

const RHRRTStar::Connection* RHRRTStar::ViewPoint::getActiveConnection() const {
  if (connections.size() <= active_connection) {
    return nullptr;
  }
  return connections[active_connection].second.get();
}

RHRRTStar::ViewPoint* RHRRTStar::ViewPoint::getConnectedViewPoint(
    size_t index) const {
  if (index >= connections.size()) {
    LOG(WARNING) << "Tried to access a viewpoint out of range (" << index << "/"
                 << connections.size() << ").";
    return nullptr;
  }
  if (connections[index].first) {
    return connections[index].second->target;
  }
  return connections[index].second->parent;
}

RHRRTStar::Connection* RHRRTStar::ViewPoint::getConnection(size_t index) const {
  if (index >= connections.size()) {
    LOG(WARNING) << "Tried to access a connection out of range (" << index
                 << "/" << connections.size() << ").";
    return nullptr;
  }
  return connections[index].second.get();
}

RHRRTStar::ViewPoint* RHRRTStar::ViewPoint::getActiveViewPoint() const {
  return getConnectedViewPoint(active_connection);
}

std::vector<RHRRTStar::ViewPoint*> RHRRTStar::ViewPoint::getChildViewPoints()
    const {
  std::vector<RHRRTStar::ViewPoint*> result;
  for (size_t i = 0; i < connections.size(); ++i) {
    ViewPoint* candidate = getConnectedViewPoint(i);
    if (!candidate->is_root) {
      if (candidate->getActiveViewPoint() == this) {
        result.push_back(candidate);
      }
    }
  }
  return result;
}

void RHRRTStar::ViewPoint::removeConnection(const Connection* connection) {
  ViewPoint* other;
  if (connection->target == this) {
    other = connection->parent;
  } else if (connection->parent == this) {
    other = connection->target;
  } else {
    // Tried to remove a connection that is not attached to this viewpoint.
    return;
  }

  // Remove both shared pointers .
  // NOTE: This does not yet remove inaccessible points. These need to be
  // separately pruned.
  connections.erase(std::remove_if(connections.begin(), connections.end(),
                                   [connection](auto& c) {
                                     return c.second.get() == connection;
                                   }),
                    connections.end());
  other->connections.erase(
      std::remove_if(
          other->connections.begin(), other->connections.end(),
          [connection](auto& c) { return c.second.get() == connection; }),
      other->connections.end());

  // Keep active connections within bounds. These will be corrected by the
  // collision update to form a proper tree.
  if (active_connection >= connections.size()) {
    active_connection = 0;
  }
  if (other->active_connection >= other->connections.size()) {
    other->active_connection = 0;
  }
}

void RHRRTStar::ViewPoint::setActiveConnection(size_t index) {
  if (index >= connections.size()) {
    LOG(WARNING) << "Tried to set an invalid active connection (" << index
                 << "/" << connections.size() << ")";
    return;
  }
  active_connection = index;
}

}  // namespace glocal_exploration
