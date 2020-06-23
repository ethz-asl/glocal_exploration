#include "glocal_exploration/planning/local_planner/rh_rrt_star.h"

#include <random>
#include <numeric>
#include <chrono>
#include <algorithm>

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
  if (state_machine_->previousState() != StateMachine::LocalPlanning) {
    // Newly started local planning
    resetPlanner(state_machine_->currentPose());
    state_machine_->signalLocalPlanning();
  }
  if (state_machine_->targetIsReached()) {
    // Goal reached: request next point
    WayPoint next_waypoint;
    selectNextBestWayPoint(&next_waypoint);
    state_machine_->requestWayPoint(next_waypoint);
    updateTree();
  }
  // TODO(schmluk): Somewhere here we need to identify when to switch to global
  expandTree();
}

void RHRRTStar::resetPlanner(const WayPoint &origin) {
  // clear the tree and initialize with a point at the current pose
  tree_data_.points.clear();
  ViewPoint point;
  point.pose = origin;
  point.is_root = true;
  tree_data_.points.push_back(point);
  kdtree_ = std::make_unique<KDTree>(3, tree_data_);

  // reset counters
  local_sampled_points_ = config_.min_local_points;
}

void RHRRTStar::expandTree() {
  ViewPoint new_point;
  // sample a goal pose
  if (!sampleNewPoint(&new_point)) { return; }

  // establish connections to nearby neighbors (at least 1 should be guaranteed by the sampling procedure)
  if (!connectViewPoint(&new_point)) { return; }

  // evaluate the gain of the point
  evaluateViewPoint(&new_point);

  // activate the best connection and add to the tree
  selectBestConnection(&new_point);
  tree_data_.points.push_back(new_point);
  kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
  if (local_sampled_points_ > 0) {
    local_sampled_points_ -= 1;
  }
}

void RHRRTStar::selectNextBestWayPoint(WayPoint *way_point) {
  // Optimize the tree structure
  int iterations = 0;
  auto t_start = std::chrono::high_resolution_clock::now();
  ViewPoint *root;
  while (iterations++ < config_.maximum_rewiring_iterations) {
    bool something_changed = false;
    for (auto &view_point : tree_data_.points) {
      if (!view_point.is_root) {
        // optimize local connections
        size_t previous_connection = view_point.active_connection;
        selectBestConnection(&view_point);
        if (view_point.active_connection != previous_connection) {
          something_changed = true;
        }
      } else {
        root = &view_point;
      }
    }
    if (!something_changed) {
      break;
    }
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  VLOG(4) << "Optimized the tree structure in "
          << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << "ms and " << iterations
          << " iterations ";

  // select the best node from the current root
  ViewPoint *next_point;
  double best_value = -std::numeric_limits<double>::max();
  for (size_t i = 0; i < root->connections.size(); ++i) {
    ViewPoint *target = root->getConnectedViewPoint(i);
    if (target->getConnectedViewPoint(target->active_connection) == root) {
      if (target->value > best_value) {
        best_value = target->value;
        next_point = target;
        root->active_connection = i;
      }
    }
  }

  // update the root nodes
  root->is_root = false;
  next_point->is_root = true;
  *way_point = next_point->pose;
}

void RHRRTStar::updateTree() {
  // update collision
  updateCollision();

  // update gains
  updateGains();

  // refresh the number of local points
  if (config_.min_local_points > 0) {
    auto root = std::find_if(tree_data_.points.begin(), tree_data_.points.end(), [](const ViewPoint &p) {
      return p.is_root;
    });
    Eigen::Vector3d goal = root->pose.position();
    local_sampled_points_ = config_.min_local_points;
    std::vector<ViewPoint *> nearest_viewpoints;
    findNearestNeighbors(goal, &nearest_viewpoints, config_.min_local_points);
    for (auto viewpoint: nearest_viewpoints) {
      if ((viewpoint->pose.position() - goal).norm() <= config_.local_sampling_radius) {
        local_sampled_points_--;
      }
    }
  }
}

void RHRRTStar::updateCollision() {

}

void RHRRTStar::updateGains() {

}

bool RHRRTStar::connectViewPoint(ViewPoint *view_point) {
  std::vector<ViewPoint *> nearest_viewpoints;
  if (!findNearestNeighbors(view_point->pose.position(), &nearest_viewpoints, config_.max_number_of_neighbors)) {
    return false;
  }
  bool connection_found = false;
  for (auto &neighbor : nearest_viewpoints) {
    if (view_point->tryAddConnection(neighbor, map_)) {
      connection_found = true;
    }
  }
  return connection_found;
}

bool RHRRTStar::selectBestConnection(ViewPoint *view_point) {
  if (view_point->connections.empty()) {
    return false;
  }
  double best_value = -std::numeric_limits<double>::max();
  size_t best_connection = -1;
  size_t previous_conection = view_point->active_connection;
  for (size_t i = 0; i < view_point->connections.size(); ++i) {
    // make sure there are no loops in the tree
    ViewPoint *current = view_point->getConnectedViewPoint(i);
    while (!current->is_root) {
      current = current->getConnectedViewPoint(current->active_connection);
      if (current == view_point) {
        return false;
      }
    }

    // compute the value
    view_point->active_connection = i;
    computeValue(view_point);
    if (view_point->value > best_value) {
      best_value = view_point->value;
      best_connection = i;
    }
  }
  if (view_point->connections.empty()) {
    return false;
  }
  view_point->value = best_value;
  view_point->active_connection = best_connection;
  return true;
}

void RHRRTStar::computeValue(ViewPoint *view_point) {
  double gain = 0.0;
  double cost = 0.0;
  ViewPoint *current = view_point;
  while (!current->is_root) {
    // propagate the new value up to the root
    current = current->getConnectedViewPoint(current->active_connection);
    gain += current->gain;
    cost += current->getActiveConnection()->cost;
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
    if (target->getConnectedViewPoint(target->active_connection) == view_point) {
      // this means target is a child of view_point
      value = std::max(value, computeGNVStep(target, gain, cost));
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

  // Find the nearest neighbor
  std::vector<ViewPoint *> nearest_viewpoint;
  if (!findNearestNeighbors(goal, &nearest_viewpoint)) { return false; }
  Eigen::Vector3d origin = nearest_viewpoint.front()->pose.position();

  // verify and crop the sampled path
  if ((goal - origin).norm() < config_.min_path_length) {
    return false;
  }
  double range_increment = map_->getVoxelSize();
  double range = range_increment;
  auto orientation = Eigen::Quaterniond();
  Eigen::Vector3d direction = (goal - origin).normalized();
  while (map_->isTraversableInActiveSubmap(origin + range * direction, orientation)
      && range < config_.max_path_length + config_.path_cropping_length) {
    range += range_increment;
  }
  if (range - config_.path_cropping_length < config_.min_path_length) {
    return false;
  }

  // write the result
  goal = origin + (range - config_.path_cropping_length) * direction;
  point->pose.x = goal.x();
  point->pose.y = goal.y();
  point->pose.z = goal.z();
  return true;
}

bool RHRRTStar::findNearestNeighbors(Eigen::Vector3d position, std::vector<ViewPoint *> *result, int n_neighbors) {
  // how to use nanoflann (:
  double query_pt[3] = {position.x(), position.y(), position.z()};
  std::size_t ret_index[n_neighbors];
  double out_dist[n_neighbors];
  nanoflann::KNNResultSet<double> resultSet(n_neighbors);
  resultSet.init(ret_index, out_dist);
  if (!kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10))) {
    return false;
  }
  for (int i = 0; i < resultSet.size(); ++i) {
    result->push_back(&tree_data_.points[ret_index[i]]);
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
  direction = direction.normalized();
  std::vector<Eigen::Vector3d> path_points;
  path_points.resize(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    path_points[i] = origin + (double) i / (n_points - 1) * direction;
    if (!map->isTraversableInActiveSubmap(path_points[i])) {
      return false;
    }
  }

  // Add the connection
  Connection connection;
  connection.parent = this;
  connection.target = target;
  connections.emplace_back(std::make_pair(true, std::shared_ptr<Connection>(&connection)));
  target->connections.emplace_back(std::make_pair(false, std::shared_ptr<Connection>(&connection)));
  connection.path_points = std::move(path_points);

  // at the moment just use length as the cost
  connection.cost = (origin - target->pose.position()).norm();
  return true;
}

RHRRTStar::Connection *RHRRTStar::ViewPoint::getActiveConnection() {
  return connections[active_connection].second.get();
}
RHRRTStar::ViewPoint *RHRRTStar::ViewPoint::getConnectedViewPoint(size_t index) {
  if (index < 0 || index >= connections.size()) {
    return nullptr;
  }
  if (connections[index].first) {
    return connections[index].second->target;
  }
  return connections[index].second->parent;
}

} // namespace glocal_exploration
