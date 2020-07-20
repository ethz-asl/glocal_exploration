#include "glocal_exploration/planning/local/rh_rrt_star.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <random>
#include <utility>
#include <vector>

namespace glocal_exploration {

RHRRTStar::RHRRTStar(std::shared_ptr<Communicator> communicator)
    : LocalPlannerBase(std::move(communicator)) {}

bool RHRRTStar::setupFromConfig(LocalPlannerBase::Config* config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config*>(config);
  if (!cfg) {
    LOG(ERROR) << "Failed to setup: config is not of type 'RHRRTStar::Config'.";
    return false;
  }
  config_ = *cfg;

  // setup the sensor
  std::shared_ptr<MapBase> map(comm_->map());
  std::shared_ptr<RegionOfInterest> roi(comm_->regionOfInterest());
  sensor_model_ = std::make_unique<LidarModel>(map, roi);
  sensor_model_->setupFromConfig(&(config_.lidar_config));
  return true;
}

void RHRRTStar::planningIteration() {
  // Newly started local planning
  if (comm_->stateMachine()->previousState() != StateMachine::kLocalPlanning) {
    resetPlanner(comm_->currentPose());
    comm_->stateMachine()->signalLocalPlanning();
  }

  // Requested a view point so update
  if (next_root_index_ > -1) {
    updateTree();
    next_root_index_ = -1;
  }

  // expansion step
  expandTree();

  // Goal reached: request next point if there is a valid candidate
  if (comm_->targetIsReached()) {
    WayPoint next_waypoint;
    if (selectNextBestWayPoint(&next_waypoint)) {
      comm_->requestWayPoint(next_waypoint);
    }
  }

  // TODO(schmluk): Somewhere here we need to identify when to switch to global
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
  if (local_sampled_points_ > 0) {
    local_sampled_points_ -= 1;
  }
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
  VLOG(3) << "Optimized the tree in "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t_end -
                                                                   t_start)
                 .count()
          << "ms, " << iterations << " iterations.";

  // select the best node from the current root
  int next_point = -1;
  double best_value = -std::numeric_limits<double>::max();
  for (size_t i = 0; i < root_->connections.size(); ++i) {
    ViewPoint* target = root_->getConnectedViewPoint(i);
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
  // NOTE: this method requires the next_root_index_ to be computed by
  // selectNextBestWayPoint() previously. counting
  int points_before_update = tree_data_.points.size();
  auto t_start = std::chrono::high_resolution_clock::now();

  // update collision
  updateCollision();

  // update gains
  updateGains();

  // reset the root (after the gains so the correct nodes get updated)
  ViewPoint* new_root = root_->getConnectedViewPoint(next_root_index_);
  root_->is_root = false;
  new_root->is_root = true;
  root_->active_connection =
      next_root_index_;  // make the old root connect to the new root
  root_ = new_root;

  // refresh the number of local points
  if (config_.min_local_points > 0) {
    Eigen::Vector3d goal = new_root->pose.position();
    local_sampled_points_ = config_.min_local_points;
    std::vector<size_t> nearest_viewpoints;
    findNearestNeighbors(goal, &nearest_viewpoints, config_.min_local_points);
    for (auto index : nearest_viewpoints) {
      if ((tree_data_.points[index]->pose.position() - goal).norm() <=
          config_.local_sampling_radius) {
        local_sampled_points_--;
      }
    }
  }

  // logging
  auto t_end = std::chrono::high_resolution_clock::now();
  VLOG(2) << "Update took "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t_end -
                                                                   t_start)
                 .count()
          << "ms: " << points_before_update - num_previous_points_ << " new, "
          << points_before_update - tree_data_.points.size() << " killed, "
          << tree_data_.points.size() << " total.";
  num_previous_points_ = tree_data_.points.size();
}

void RHRRTStar::updateCollision() {
  // cache the next connection so the index can be corrected after pruning
  Connection* next_connection =
      root_->connections[next_root_index_].second.get();

  // update all connections
  for (auto& viewpoint : tree_data_.points) {
    int offset = 0;
    for (int _i = 0; _i < viewpoint->connections.size(); ++_i) {
      int i = _i + offset;

      // update every connection only once (by the parent)
      if (viewpoint->connections[i].first) {
        Connection* connection = viewpoint->connections[i].second.get();
        ViewPoint* target = connection->target;
        if (connection == next_connection) {
          // don't update the currently executed connection, this always allows
          // backtracking as well.
          continue;
        }

        // check collision
        bool collided = false;
        for (auto& path_point : connection->path_points) {
          if (!comm_->map()->isTraversableInActiveSubmap(path_point)) {
            collided = true;
            break;
          }
        }

        // remove these connections
        if (collided) {
          target->connections.erase(std::remove_if(target->connections.begin(),
                                                   target->connections.end(),
                                                   [connection](auto& c) {
                                                     return c.second.get() ==
                                                            connection;
                                                   }),
                                    target->connections.end());
          viewpoint->connections.erase(viewpoint->connections.begin() + i);
          offset -= 1;
        }
      }
    }
  }

  // remove view_points that don't have a connection to the root anymore anymore
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

  // adjust the next_root index
  for (size_t i = 0; i < root_->connections.size(); ++i) {
    if (root_->connections[i].second.get() == next_connection) {
      next_root_index_ = i;
      break;
    }
  }
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
  for (auto& point : tree_data_.points) {
    if (point->is_root ||
        point.get() == root_->getConnectedViewPoint(next_root_index_)) {
      // don't update the old or new root
      point->gain = 0.0;
      continue;
    }
    evaluateViewPoint(point.get());
  }
}

bool RHRRTStar::connectViewPoint(ViewPoint* view_point) {
  // This method is called on newly sampled points, so they can not look up
  // themselves or duplicate connections
  std::vector<size_t> nearest_viewpoints;
  if (!findNearestNeighbors(view_point->pose.position(), &nearest_viewpoints,
                            config_.max_number_of_neighbors)) {
    return false;
  }
  bool connection_found = false;
  for (const auto& index : nearest_viewpoints) {
    double distance = (view_point->pose.position() -
                       tree_data_.points[index]->pose.position())
                          .norm();
    if (distance > config_.max_path_length ||
        distance < config_.min_path_length) {
      continue;
    }
    if (view_point->tryAddConnection(tree_data_.points[index].get(),
                                     comm_->map())) {
      view_point->connections.back().second->cost =
          computeCost(*view_point->connections.back().second.get());
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

double RHRRTStar::computeGain(
    const std::vector<Eigen::Vector3d>& visible_voxels) {
  double gain = 0.0;
  for (auto& point : visible_voxels) {
    if (comm_->map()->getVoxelStateInLocalArea(point) == MapBase::kUnknown) {
      gain += 1.0;
    }
  }
  return gain;
}

double RHRRTStar::computeCost(const Connection& connection) {
  // just use distance
  return (connection.path_points.front() - connection.path_points.back())
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
  // sample the goal point
  double theta = 2.0 * M_PI * static_cast<double>(std::rand()) /
                 static_cast<double>(RAND_MAX);
  double phi = acos(1.0 - 2.0 * static_cast<double>(std::rand()) /
                              static_cast<double>(RAND_MAX));
  double rho = local_sampled_points_ > 0 ? config_.local_sampling_radius
                                         : config_.global_sampling_radius;
  Eigen::Vector3d goal = rho * Eigen::Vector3d(sin(phi) * cos(theta),
                                               sin(phi) * sin(theta), cos(phi));
  goal += comm_->currentPose().position();

  // Find the nearest neighbor
  std::vector<size_t> nearest_viewpoint;
  if (!findNearestNeighbors(goal, &nearest_viewpoint)) {
    return false;
  }
  Eigen::Vector3d origin =
      tree_data_.points[nearest_viewpoint.front()]->pose.position();
  double distance_max =
      std::min(std::max((goal - origin).norm(), config_.min_sampling_distance),
               config_.max_path_length) +
      config_.path_cropping_length;

  // verify and crop the sampled path
  double range_increment = comm_->map()->getVoxelSize();
  double range = range_increment;
  auto orientation = Eigen::Quaterniond();
  Eigen::Vector3d direction = (goal - origin).normalized();
  while (comm_->map()->isTraversableInActiveSubmap(origin + range * direction,
                                                   orientation) &&
         range < distance_max) {
    range += range_increment;
  }
  range = range - config_.path_cropping_length - range_increment;
  if (range < config_.min_sampling_distance) {
    return false;
  }

  // write the result
  goal = origin + range * direction;
  point->pose.x = goal.x();
  point->pose.y = goal.y();
  point->pose.z = goal.z();
  point->pose.yaw = 2.0 * M_PI * static_cast<double>(std::rand()) /
                    static_cast<double>(RAND_MAX);
  return true;
}

bool RHRRTStar::findNearestNeighbors(Eigen::Vector3d position,
                                     std::vector<size_t>* result,
                                     int n_neighbors) {
  // how to use nanoflann (:
  // Returns the indices of the neighbors in tree data
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

void RHRRTStar::evaluateViewPoint(ViewPoint* view_point) {
  std::vector<Eigen::Vector3d> visible_voxels;
  sensor_model_->getVisibleVoxels(&visible_voxels, view_point->pose);
  view_point->gain = computeGain(visible_voxels);
}

void RHRRTStar::visualizeGain(std::vector<Eigen::Vector3d>* voxels,
                              std::vector<Eigen::Vector3d>* colors,
                              double* scale, const WayPoint& pose) const {
  CHECK_NOTNULL(voxels);
  CHECK_NOTNULL(colors);
  CHECK_NOTNULL(scale);
  // get voxels
  sensor_model_->getVisibleVoxels(voxels, pose);
  voxels->erase(std::remove_if(voxels->begin(), voxels->end(),
                               [this](const Eigen::Vector3d& pt) {
                                 return comm_->map()->getVoxelStateInLocalArea(
                                            pt) != MapBase::kUnknown;
                               }),
                voxels->end());

  // uniform coloring [0, 1]
  colors->assign(voxels->size(), Eigen::Vector3d(1, 0.8, 0));

  // voxel size
  *scale = comm_->map()->getVoxelSize();
}

bool RHRRTStar::ViewPoint::tryAddConnection(ViewPoint* target, MapBase* map) {
  // Check traversability
  Eigen::Vector3d origin = pose.position();
  Eigen::Vector3d direction = target->pose.position() - origin;
  int n_points = std::floor(direction.norm() / map->getVoxelSize());
  direction.normalize();
  std::vector<Eigen::Vector3d> path_points;
  path_points.resize(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    path_points[i] = origin + static_cast<double>(i) / n_points * direction;
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
  return true;
}

RHRRTStar::Connection* RHRRTStar::ViewPoint::getActiveConnection() {
  return connections[active_connection].second.get();
}

RHRRTStar::Connection const* RHRRTStar::ViewPoint::getActiveConnection() const {
  return connections[active_connection].second.get();
}

RHRRTStar::ViewPoint* RHRRTStar::ViewPoint::getConnectedViewPoint(
    size_t index) {
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
