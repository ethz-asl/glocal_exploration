#include "glocal_exploration/planning/local_planner/rh_rrt_star.h"

#include <random>
#include <numeric>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <queue>

namespace glocal_exploration {

RHRRTStar::RHRRTStar(std::shared_ptr<MapBase> map, std::shared_ptr<StateMachine> state_machine)
    : LocalPlannerBase(std::move(map), std::move(state_machine)) {}

bool RHRRTStar::setupFromConfig(LocalPlannerBase::Config *config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config *>(config);
  if (!cfg) {
    LOG(ERROR) << "Failed to setup: config is not of type 'RHRRTStar::Config'.";
    return false;
  }
  config_ = *cfg;

  // setup the sensor
  sensor_model_ = std::make_unique<LidarModel>(map_);
  sensor_model_->setupFromConfig(&(config_.lidar_config));
  return true;
}

void RHRRTStar::planningIteration() {
  // Newly started local planning
  if (state_machine_->previousState() != StateMachine::LocalPlanning) {
    resetPlanner(state_machine_->currentPose());
    state_machine_->signalLocalPlanning();
  }

  // Requested a view point so update
  if (next_root_index_ > -1) {
    updateTree();
  }

  // expansion step
  expandTree();

  // Goal reached: request next point if there is a valid candidate
  if (state_machine_->targetIsReached()) {
    WayPoint next_waypoint;
    if (selectNextBestWayPoint(&next_waypoint)) {
      state_machine_->requestWayPoint(next_waypoint);
    }
  }

  // TODO(schmluk): Somewhere here we need to identify when to switch to global
}

void RHRRTStar::resetPlanner(const WayPoint &origin) {
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
  local_sampled_points_ = config_.min_local_points;
  num_previous_points_ = 1;
  next_root_index_ = -1;
}

void RHRRTStar::expandTree() {
  auto new_point = std::make_unique<ViewPoint>();
  // sample a goal pose
  if (!sampleNewPoint(new_point.get())) {
    return;
  }

  // establish connections to nearby neighbors (at least 1 should be guaranteed by the sampling procedure)
  if (!connectViewPoint(new_point.get())) {
    return;
  }

  // evaluate the gain of the point
  evaluateViewPoint(new_point.get());

  // Add it to the kdtree
  tree_data_.points.push_back(std::move(new_point));
  kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
  if (local_sampled_points_ > 0) {
    local_sampled_points_ -= 1;
  }
}

bool RHRRTStar::selectNextBestWayPoint(WayPoint *next_waypoint) {
  if (tree_data_.points.size() < 2) { return false; }
  // set up
  int iterations = 0;
  auto t_start = std::chrono::high_resolution_clock::now();

  // Connect every viewpoint to form a tree (this is needed for the value computation)
  computePointsConnectedToRoot(true);
  std::queue<ViewPoint*> not_connected;
  for (auto & vp: tree_data_.points) {
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
    for (auto &view_point : tree_data_.points) {
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
  VLOG(3) << "Optimized the tree in " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()
          << "ms, " << iterations << " iterations.";

  // select the best node from the current root
  int next_point = -1;
  double best_value = -std::numeric_limits<double>::max();
  for (size_t i = 0; i < root_->connections.size(); ++i) {
    ViewPoint *target = root_->getConnectedViewPoint(i);
    if (target->getConnectedViewPoint(target->active_connection) == root_) {
      if (target->value > best_value) {
        best_value = target->value;
        next_point = i;
      }
    }
  }
  if (next_point < 0) {
    return false;
  }

  // result
  next_root_index_ = next_point;
  *next_waypoint = root_->getConnectedViewPoint(next_root_index_)->pose;
  return true;
}

void RHRRTStar::updateTree() {
  // counting
  int points_before_update = tree_data_.points.size();
  auto t_start = std::chrono::high_resolution_clock::now();

  // update collision
  updateCollision();

  // update gains
  updateGains();

  // reset the root (after the gains so the correct nodes get updated)
  ViewPoint *new_root = root_->getConnectedViewPoint(next_root_index_);
  root_->is_root = false;
  new_root->is_root = true;
  root_->active_connection = next_root_index_;  // make the old root connect to the new root
  root_ = new_root;

  // refresh the number of local points
  if (config_.min_local_points > 0) {
    Eigen::Vector3d goal = new_root->pose.position();
    local_sampled_points_ = config_.min_local_points;
    std::vector<size_t> nearest_viewpoints;
    findNearestNeighbors(goal, &nearest_viewpoints, config_.min_local_points);
    for (auto index: nearest_viewpoints) {
      if ((tree_data_.points[index]->pose.position() - goal).norm() <= config_.local_sampling_radius) {
        local_sampled_points_--;
      }
    }
  }

  // logging
  auto t_end = std::chrono::high_resolution_clock::now();
  VLOG(2)
  << "Updated the tree in " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count()
  << "ms: " << points_before_update - num_previous_points_ << " new, "
  << tree_data_.points.size() - points_before_update
  << " killed.";
  num_previous_points_ = tree_data_.points.size();

  // finish
  next_root_index_ = -1;    // this signifies no updating
}

void RHRRTStar::updateCollision() {
  // update all connections
  for (auto &viewpoint : tree_data_.points) {
    int offset = 0;
    for (int _i = 0; _i < viewpoint->connections.size(); ++_i) {
      int i = _i + offset;

      // update every connection only once (by the parent)
      if (viewpoint->connections[i].first) {
        Connection *connection = viewpoint->connections[i].second.get();
        ViewPoint *target = connection->target;
        if (target->is_root) {
          if (target->connections.size() > next_root_index_) {
            if (target->connections[next_root_index_].second.get() == connection) {
              // don't update the currently executed connection, this always allows backtracking as well.
              continue;
            }
          }
        }

        // check collision
        bool collided = false;
        for (auto &path_point : connection->path_points) {
          if (!map_->isTraversableInActiveSubmap(path_point)) {
            collided = true;
            break;
          }
        }

        // remove these connections
        if (collided) {
          target->connections.erase(std::remove_if(target->connections.begin(), target->connections.end(),
                                                   [connection](auto &c) {
                                                     return c.second.get() == connection;
                                                   }), target->connections.end());
          viewpoint->connections.erase(viewpoint->connections.begin() + i);
          offset -= 1;
        }
      }
    }
  }

  // remove view_points that don't have a connection to the root anymore anymore
  computePointsConnectedToRoot(false);
  tree_data_.points.erase(std::remove_if(tree_data_.points.begin(), tree_data_.points.end(),
                                         [](std::unique_ptr<ViewPoint> &v) {
                                           return !v->is_connected_to_root;
                                         }), tree_data_.points.end());

  // reset the kdtree
  kdtree_ = std::make_unique<KDTree>(3, tree_data_);
  kdtree_->addPoints(0, tree_data_.points.size() - 1);
}

void RHRRTStar::computePointsConnectedToRoot(bool count_only_active_connections) {
  // Sets the is_connected_to_root flag for the entire tree
  std::queue<ViewPoint *> points_to_check;

  // setup
  for (auto & vp: tree_data_.points) {
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
        ViewPoint *connected_vp = points_to_check.front()->getConnectedViewPoint(i);
        if (count_only_active_connections) {
          if (connected_vp->getConnectedViewPoint(connected_vp->active_connection) != points_to_check.front()) {
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
  for (auto &point : tree_data_.points) {
    if (point->is_root || point.get() == root_->getConnectedViewPoint(next_root_index_)) {
      // don't update the old or new root
      point->gain = 0.0;
      continue;
    }
    evaluateViewPoint(point.get());
  }
}

bool RHRRTStar::connectViewPoint(ViewPoint *view_point) {
  // This method is called on newly sampled points, so they can not look up themselves or duplicate connections
  std::vector<size_t> nearest_viewpoints;
  if (!findNearestNeighbors(view_point->pose.position(), &nearest_viewpoints, config_.max_number_of_neighbors)) {
    return false;
  }
  bool connection_found = false;
  for (const auto &index : nearest_viewpoints) {
    double distance = (view_point->pose.position() - tree_data_.points[index]->pose.position()).norm();
    if (distance > config_.max_path_length || distance < config_.min_path_length) { continue; }
    if (view_point->tryAddConnection(tree_data_.points[index].get(), map_)) {
      connection_found = true;
    }
  }
  return connection_found;
}

bool RHRRTStar::selectBestConnection(ViewPoint *view_point) {
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
    ViewPoint *current = view_point;
    int iterations = 0;
    while (!current->is_root) {
      iterations++;
      current = current->getConnectedViewPoint(current->active_connection);
      if (current == view_point) {
        is_loop = true;
        break;
      }
    }
    if (is_loop) { continue; }

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

void RHRRTStar::computeValue(ViewPoint *view_point) {
  if (view_point->is_root) {
    view_point->value = 0.0;
    return;
  }
  double gain = 0.0;
  double cost = 0.0;
  ViewPoint *current = view_point;
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

double RHRRTStar::computeGNVStep(ViewPoint *view_point, double gain, double cost) {
  // recursively iterate towards leaf, then iterate backwards and select best value of children
  double value = 0.0;
  gain += view_point->gain;
  cost += view_point->getActiveConnection()->cost;
  if (cost > 0) {
    value = gain / cost;
  }
  for (size_t i = 0; i < view_point->connections.size(); ++i) {
    ViewPoint *target = view_point->getConnectedViewPoint(i);
    if (!target->is_root) {
      if (target->getConnectedViewPoint(target->active_connection) == view_point) {
        // this means target is a child of view_point
        value = std::max(value, computeGNVStep(target, gain, cost));
      }
    }
  }
  return value;
}

bool RHRRTStar::sampleNewPoint(ViewPoint *point) {
  // sample the goal point
  double theta = 2.0 * M_PI * (double) std::rand() / (double) RAND_MAX;
  double phi = acos(1.0 - 2.0 * (double) std::rand() / (double) RAND_MAX);
  double rho = local_sampled_points_ > 0 ? config_.local_sampling_radius : config_.global_sampling_radius;
  Eigen::Vector3d goal = rho * Eigen::Vector3d(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
  goal += state_machine_->currentPose().position();

  // Find the nearest neighbor
  std::vector<size_t> nearest_viewpoint;
  if (!findNearestNeighbors(goal, &nearest_viewpoint)) { return false; }
  Eigen::Vector3d origin = tree_data_.points[nearest_viewpoint.front()]->pose.position();
  double distance_max = std::min(std::max((goal - origin).norm(), config_.min_path_length), config_.max_path_length)
      + config_.path_cropping_length;

  // verify and crop the sampled path
  double range_increment = map_->getVoxelSize();
  double range = range_increment;
  auto orientation = Eigen::Quaterniond();
  Eigen::Vector3d direction = (goal - origin).normalized();
  while (map_->isTraversableInActiveSubmap(origin + range * direction, orientation)
      && range < distance_max) {
    range += range_increment;
  }
  range = range - config_.path_cropping_length - range_increment;
  if (range < config_.min_path_length) {
    return false;
  }

  // write the result
  goal = origin + range * direction;
  point->pose.x = goal.x();
  point->pose.y = goal.y();
  point->pose.z = goal.z();
  point->pose.yaw = 2.0 * M_PI * (double) std::rand() / (double) RAND_MAX;
  return true;
}

bool RHRRTStar::findNearestNeighbors(Eigen::Vector3d position, std::vector<size_t> *result, int n_neighbors) {
  // how to use nanoflann (:
  // Returns the indices of the neighbors in tree data
  double query_pt[3] = {position.x(), position.y(), position.z()};
  std::size_t ret_index[n_neighbors];
  double out_dist[n_neighbors];
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

void RHRRTStar::evaluateViewPoint(ViewPoint *view_point) {
  std::vector<Eigen::Vector3d> visible_voxels;
  sensor_model_->getVisibleVoxels(&visible_voxels, view_point->pose);
  view_point->gain = visible_voxels.size();
}

bool RHRRTStar::ViewPoint::tryAddConnection(ViewPoint *target, const std::shared_ptr<MapBase> &map) {
  // Check traversability
  Eigen::Vector3d origin = pose.position();
  Eigen::Vector3d direction = target->pose.position() - origin;
  int n_points = std::floor(direction.norm() / map->getVoxelSize());
  direction.normalize();
  std::vector<Eigen::Vector3d> path_points;
  path_points.resize(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    path_points[i] = origin + (double) i / n_points * direction;
    if (!map->isTraversableInActiveSubmap(path_points[i])) {
      return false;
    }
  }

  // Add the connection
  auto connection = std::make_shared<Connection>();
  connection->parent = this;
  connection->target = target;
  connections.emplace_back(std::make_pair(true, connection));
  target->connections.emplace_back(std::make_pair(false, connection));
  connection->path_points = std::move(path_points);

  // at the moment just use length as the cost
  connection->cost = (origin - target->pose.position()).norm();
  return true;
}

RHRRTStar::Connection *RHRRTStar::ViewPoint::getActiveConnection() {
  return connections[active_connection].second.get();
}

RHRRTStar::Connection const *RHRRTStar::ViewPoint::getActiveConnection() const {
  return connections[active_connection].second.get();
}

RHRRTStar::ViewPoint *RHRRTStar::ViewPoint::getConnectedViewPoint(size_t index) {
  if (index < 0 || index >= connections.size()) {
    LOG(WARNING) << "Tried to access a connection out of range (" << index << "/" << connections.size() << ").";
    return nullptr;
  }
  if (connections[index].first) {
    return connections[index].second->target;
  }
  return connections[index].second->parent;
}

} // namespace glocal_exploration
