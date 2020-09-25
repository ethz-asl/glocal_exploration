#include "glocal_exploration/planning/local/rh_rrt_star.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <utility>
#include <vector>

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>

#include "glocal_exploration/state/communicator.h"

namespace glocal_exploration {

RHRRTStar::Config::Config() { setConfigName("RHRRTStar"); }

void RHRRTStar::Config::checkParams() const {
  checkParamGT(max_path_length, 0.0, "max_path_length");
  checkParamGE(path_cropping_length, 0.0, "path_cropping_length");
  checkParamGT(max_number_of_neighbors, 0, "max_number_of_neighbors");
  checkParamGT(maximum_rewiring_iterations, 0, "maximum_rewiring_iterations");
  checkParamGT(sampling_range, 0.0, "sampling_range");
  checkParamGE(min_path_length, 0.0, "min_path_length");
  checkParamGE(min_sampling_distance, 0.0, "min_sampling_distance");
  checkParamGE(terminaton_min_tree_size, 0, "terminaton_min_tree_size");
  checkParamGE(termination_max_gain, 0.0, "termination_max_gain");
  checkParamGE(termination_min_time, 0.0, "termination_min_time");
  checkParamConfig(lidar_config);
}

void RHRRTStar::Config::fromRosParam() {
  rosParam("verbosity", &verbosity);
  rosParam("sampling_range", &sampling_range);
  rosParam("min_path_length", &min_path_length);
  rosParam("min_sampling_distance", &min_sampling_distance);
  rosParam("max_path_length", &max_path_length);
  rosParam("path_cropping_length", &path_cropping_length);
  rosParam("max_number_of_neighbors", &max_number_of_neighbors);
  rosParam("maximum_rewiring_iterations", &maximum_rewiring_iterations);
  rosParam("terminaton_min_tree_size", &terminaton_min_tree_size);
  rosParam("termination_max_gain", &termination_max_gain);
  rosParam("termination_min_time", &termination_min_time);
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
  printField("max_number_of_neighbors", max_number_of_neighbors);
  printField("maximum_rewiring_iterations", maximum_rewiring_iterations);
  printField("terminaton_min_tree_size", terminaton_min_tree_size);
  printField("termination_max_gain", termination_max_gain);
  printField("termination_min_time", termination_min_time);
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
  if (comm_->targetIsReached() && !termination_time_is_active_) {
    WayPoint next_waypoint;
    updateCollision();
    if (selectNextBestWayPoint(&next_waypoint)) {
      comm_->requestWayPoint(next_waypoint);
      gain_update_needed_ = true;
    }
  }

  // Check whether a local minimum is reached and change to global planning.
  if (tree_data_.points.size() >= config_.terminaton_min_tree_size) {
    for (const auto& point : tree_data_.points) {
      if (point->gain > config_.termination_max_gain) {
        termination_time_is_active_ = false;
        return;
      }
    }
    if (termination_time_is_active_) {
      if (std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::high_resolution_clock::now() - termination_time_)
              .count() >= config_.termination_min_time) {
        comm_->stateMachine()->signalGlobalPlanning();
      }
    } else {
      LOG_IF(INFO, config_.verbosity >= 3)
          << "Termination criterion is met, trying to overcome it for "
          << config_.termination_min_time
          << "s before moving to global planning.";
      termination_time_ = std::chrono::high_resolution_clock::now();
      termination_time_is_active_ = true;
    }
  }
}

void RHRRTStar::resetPlanner(const WayPoint& origin) {
  // clear the tree and initialize with a point at the current pose
  tree_data_.points.clear();
  auto point = std::make_unique<ViewPoint>();
  point->pose = origin;
  point->is_root = true;
  kdtree_ = std::make_unique<KDTree>(3, tree_data_);
  tree_data_.points.push_back(std::move(point));
  kdtree_->addPoints(0, 0);

  // reset counters
  root_ = tree_data_.points[0].get();
  current_connection_ = nullptr;
  gain_update_needed_ = false;
  pruned_points_ = 0;
  new_points_ = 0;
  termination_time_is_active_ = false;

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

  // set up
  int iterations = 0;
  auto t_start = std::chrono::high_resolution_clock::now();

  // Connect every viewpoint to form a tree (this is needed for the value
  // computation)
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
    for (size_t i = 0; i < current->connections.size(); ++i) {
      if (current->getConnectedViewPoint(i)->is_connected_to_root) {
        current->active_connection = i;
        current->is_connected_to_root = true;
        break;
      }
    }
    if (!current->is_connected_to_root) {
      not_connected.push(current);
    }
  }

  // Optimize the tree structure
  while (iterations++ < config_.maximum_rewiring_iterations) {
    bool something_changed = false;
    for (auto& view_point : tree_data_.points) {
      if (!view_point->is_root) {
        // optimize local connections
        size_t previous_connection = view_point->active_connection;
        selectBestConnection(view_point.get());
        if (view_point->active_connection != previous_connection) {
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
  double best_value = -std::numeric_limits<double>::max();
  for (size_t i = 0; i < root_->connections.size(); ++i) {
    ViewPoint* target = root_->getConnectedViewPoint(i);
    if (target->getConnectedViewPoint(target->active_connection) == root_) {
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
  }

  // result
  *next_waypoint = root_->getConnectedViewPoint(next_point_idx)->pose;

  // update the roots
  ViewPoint* new_root = root_->getConnectedViewPoint(next_point_idx);
  root_->is_root = false;
  new_root->is_root = true;
  root_->active_connection =
      next_point_idx;  // make the old root connect to the new root
  current_connection_ = root_->connections[next_point_idx].second.get();
  root_ = new_root;

  // logging
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Published next segment: " << new_points_ << " new, " << pruned_points_
      << " killed, " << tree_data_.points.size() << " total.";
  pruned_points_ = 0;
  new_points_ = 0;

  return true;
}

void RHRRTStar::updateCollision() {
  int num_previous_points = tree_data_.points.size();
  // update all connections
  for (auto& viewpoint : tree_data_.points) {
    int offset = 0;
    for (int _i = 0; _i < viewpoint->connections.size(); ++_i) {
      int i = _i + offset;

      // Update every connection only once (by the parent).
      if (viewpoint->connections[i].first) {
        Connection* connection = viewpoint->connections[i].second.get();
        if (connection == current_connection_) {
          // don't update the currently executed connection, this always allows
          // backtracking as well.
          continue;
        }

        // Remove colliding connections.
        if (comm_->map()->isLineTraversableInActiveSubmap(
                connection->parent->pose.position,
                connection->target->pose.position)) {
          ViewPoint* target = connection->target;
          target->connections.erase(std::remove_if(target->connections.begin(),
                                                   target->connections.end(),
                                                   [connection](auto& c) {
                                                     return c.second.get() ==
                                                            connection;
                                                   }),
                                    target->connections.end());
          viewpoint->connections.erase(viewpoint->connections.begin() + i);
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
    for (size_t i = 0; i < points_to_check.front()->connections.size(); ++i) {
      ViewPoint* connected_vp =
          points_to_check.front()->getConnectedViewPoint(i);
      if (count_only_active_connections) {
        if (connected_vp->active_connection >=
                connected_vp->connections.size() ||
            connected_vp->active_connection < 0) {
          continue;
        } else if (connected_vp->getConnectedViewPoint(
                       connected_vp->active_connection) !=
                   points_to_check.front()) {
          continue;
        }
      }
      if (!connected_vp->is_connected_to_root) {
        points_to_check.push(connected_vp);
        connected_vp->is_connected_to_root = true;
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
      point->gain = 0.0;
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
    double distance =
        (view_point->pose.position - tree_data_.points[index]->pose.position)
            .norm();
    if (distance > config_.max_path_length ||
        distance < config_.min_path_length) {
      continue;
    }
    if (view_point->tryAddConnection(tree_data_.points[index].get(),
                                     comm_->map().get())) {
      view_point->connections.back().second->cost =
          computeCost(*view_point->connections.back().second);
      connection_found = true;
    }
  }
  return connection_found;
}

bool RHRRTStar::selectBestConnection(ViewPoint* view_point) {
  // This operation is an iteration step to optimize the tree structure.
  if (view_point->connections.empty() || view_point->is_root) {
    return false;
  }
  double best_value = -std::numeric_limits<double>::max();
  size_t best_connection = -1;
  for (size_t i = 0; i < view_point->connections.size(); ++i) {
    // make sure there are no loops in the tree
    view_point->active_connection = i;
    bool is_loop = false;
    ViewPoint* current = view_point;
    int iterations = 0;
    while (!current->is_root) {
      iterations++;
      current = current->getConnectedViewPoint(current->active_connection);
      if (current == view_point) {
        is_loop = true;
        break;
      }
    }
    if (is_loop) {
      continue;
    }

    // compute the value
    computeValue(view_point);
    if (view_point->value > best_value) {
      best_value = view_point->value;
      best_connection = i;
    }
  }
  if (best_connection == -1) {
    return false;
  }

  // apply the result
  view_point->value = best_value;
  view_point->active_connection = best_connection;
  return true;
}

void RHRRTStar::evaluateViewPoint(ViewPoint* view_point) {
  voxblox::LongIndexSet voxels;
  sensor_model_->getVisibleUnknownVoxels(view_point->pose, &voxels);
  view_point->gain = voxels.size();
}

double RHRRTStar::computeCost(const Connection& connection) {
  // just use distance
  return (connection.parent->pose.position - connection.target->pose.position)
      .norm();
}

void RHRRTStar::computeValue(ViewPoint* view_point) {
  if (view_point->is_root) {
    view_point->value = 0.0;
    return;
  }
  double gain = 0.0;
  double cost = 0.0;
  ViewPoint* current = view_point;
  while (true) {
    // propagate the new value up to the root
    current = current->getConnectedViewPoint(current->active_connection);
    if (current->is_root) {
      break;
    } else {
      gain += current->gain;
      cost += current->getActiveConnection()->cost;
    }
  }
  // propagate recursively to the leaves
  view_point->value = computeGNVStep(view_point, gain, cost);
}

double RHRRTStar::computeGNVStep(ViewPoint* view_point, double gain,
                                 double cost) {
  // recursively iterate towards leaf, then iterate backwards and select best
  // value of children
  double value = 0.0;
  gain += view_point->gain;
  cost += view_point->getActiveConnection()->cost;
  if (cost > 0) {
    value = gain / cost;
    // TEST: foucs more on local expl
    // value = gain / cost / cost;
  }
  for (size_t i = 0; i < view_point->connections.size(); ++i) {
    ViewPoint* target = view_point->getConnectedViewPoint(i);
    if (!target->is_root) {
      if (target->getConnectedViewPoint(target->active_connection) ==
          view_point) {
        // this means target is a child of view_point
        value = std::max(value, computeGNVStep(target, gain, cost));
      }
    }
  }
  return value;
}

bool RHRRTStar::sampleNewPoint(ViewPoint* point) {
  // Sample the goal point.

  const double theta = 2.0 * M_PI * static_cast<double>(std::rand()) /
                       static_cast<double>(RAND_MAX);
  const double phi = acos(1.0 - 2.0 * static_cast<double>(std::rand()) /
                                    static_cast<double>(RAND_MAX));
  Point goal = comm_->currentPose().position +
               config_.sampling_range * Point(sin(phi) * cos(theta),
                                              sin(phi) * sin(theta), cos(phi));

  // Find the nearest neighbor.
  std::vector<size_t> nearest_viewpoint;
  if (!findNearestNeighbors(goal, &nearest_viewpoint)) {
    return false;
  }
  Point origin = tree_data_.points[nearest_viewpoint.front()]->pose.position;
  double distance_max =
      std::min((goal - origin).norm(), config_.max_path_length);
  if (distance_max < config_.min_sampling_distance) {
    return false;
  }
  distance_max += config_.path_cropping_length;

  // Verify and crop the sampled path.
  goal = origin + (goal - origin).normalized() * distance_max;
  Point goal_cropped;
  comm_->map()->isLineTraversableInActiveSubmap(origin, goal, &goal_cropped);

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
  point->pose.yaw = 2.0 * M_PI * static_cast<double>(std::rand()) /
                    static_cast<double>(RAND_MAX);
  return true;
}

bool RHRRTStar::findNearestNeighbors(Point position,
                                     std::vector<size_t>* result,
                                     int n_neighbors) {
  // how to use nanoflann (:
  // Returns the indices of the neighbors in tree data.
  double query_pt[3] = {position.x(), position.y(), position.z()};
  std::size_t ret_index[n_neighbors];  // NOLINT
  double out_dist[n_neighbors];        // NOLINT
  nanoflann::KNNResultSet<double> resultSet(n_neighbors);
  resultSet.init(ret_index, out_dist);
  kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));
  if (resultSet.size() == 0) {
    return false;
  }
  for (int i = 0; i < resultSet.size(); ++i) {
    result->push_back(ret_index[i]);
  }
  return true;
}

void RHRRTStar::visualizeGain(const WayPoint& pose, std::vector<Point>* voxels,
                              std::vector<Point>* colors, double* scale) const {
  CHECK_NOTNULL(voxels);
  CHECK_NOTNULL(colors);
  CHECK_NOTNULL(scale);
  // TODO(schmluk): This is neither beautiful nor efficient but it doesn't get
  //  called often...

  // get voxel indices
  voxblox::LongIndexSet voxels_idx;
  sensor_model_->getVisibleUnknownVoxels(pose, &voxels_idx);

  // voxel size
  *scale = comm_->map()->getVoxelSize();
  const double voxel_size = comm_->map()->getVoxelSize();

  // get centers
  voxels->clear();
  voxels->reserve(voxels_idx.size());
  for (const auto& idx : voxels_idx) {
    voxels->push_back(
        voxblox::getCenterPointFromGridIndex(idx, voxel_size).cast<double>());
  }

  // Uniform coloring [0, 1].
  colors->assign(voxels->size(), Point(1, 0.8, 0));
}

bool RHRRTStar::ViewPoint::tryAddConnection(ViewPoint* target, MapBase* map) {
  // Check traversability.
  Point origin = pose.position;
  if (!map->isLineTraversableInActiveSubmap(pose.position,
                                            target->pose.position)) {
    return false;
  }

  // Add the connection.
  auto connection = std::make_shared<Connection>();
  connection->parent = this;
  connection->target = target;
  connections.emplace_back(std::make_pair(true, connection));
  target->connections.emplace_back(std::make_pair(false, connection));
  return true;
}

RHRRTStar::Connection* RHRRTStar::ViewPoint::getActiveConnection() {
  if (active_connection < 0 || active_connection >= connections.size()) {
    return nullptr;
  }
  return connections[active_connection].second.get();
}

RHRRTStar::Connection const* RHRRTStar::ViewPoint::getActiveConnection() const {
  if (active_connection < 0 || active_connection >= connections.size()) {
    return nullptr;
  }
  return connections[active_connection].second.get();
}

RHRRTStar::ViewPoint* RHRRTStar::ViewPoint::getConnectedViewPoint(
    size_t index) const {
  if (index < 0 || index >= connections.size()) {
    LOG(WARNING) << "Tried to access a connection out of range (" << index
                 << "/" << connections.size() << ").";
    return nullptr;
  }
  if (connections[index].first) {
    return connections[index].second->target;
  }
  return connections[index].second->parent;
}

}  // namespace glocal_exploration
