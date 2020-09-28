#include "glocal_exploration_ros/mapping/voxgraph_map.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <glocal_exploration/planning/global/submap_frontier_evaluator.h>
#include <glocal_exploration/state/communicator.h>

#include "glocal_exploration_ros/planning/global/skeleton_planner.h"

namespace glocal_exploration {

VoxgraphMap::Config::Config() { setConfigName("VoxgraphMap"); }

void VoxgraphMap::Config::checkParams() const {
  checkParamGT(traversability_radius, 0.f, "traversability_radius");
}

void VoxgraphMap::Config::fromRosParam() {
  rosParam("traversability_radius", &traversability_radius);
  rosParam("clearing_radius", &clearing_radius);
  rosParam("verbosity", &verbosity);
  nh_private_namespace = rosParamNameSpace();
}

void VoxgraphMap::Config::printFields() const {
  printField("verbosity", verbosity);
  printField("clearing_radius", clearing_radius);
  printField("traversability_radius", traversability_radius);
  printField("nh_private_namespace", nh_private_namespace);
}

VoxgraphMap::VoxgraphMap(const Config& config,
                         const std::shared_ptr<Communicator>& communicator)
    : MapBase(communicator),
      config_(config.checkValid()),
      local_area_needs_update_(false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" + config_.toString();
  // Launch the sliding window local map and global map servers
  ros::NodeHandle nh(ros::names::parentNamespace(config_.nh_private_namespace));
  ros::NodeHandle nh_private(config_.nh_private_namespace);
  voxblox_server_ = std::make_unique<ThreadsafeVoxbloxServer>(nh, nh_private);
  voxgraph_server_ = std::make_unique<ThreadsafeVoxgraphServer>(nh, nh_private);

  // Setup the local area
  local_area_ = std::make_unique<VoxgraphLocalArea>(
      voxblox::getTsdfMapConfigFromRosParam(nh_private));
  voxblox_server_->setExternalNewPoseCallback(
      [&] { local_area_needs_update_ = true; });
  local_area_pub_ = nh_private.advertise<pcl::PointCloud<pcl::PointXYZI>>(
      "local_area", 1, true);
  local_area_pruning_timer_ = nh_private.createTimer(
      ros::Duration(local_area_pruning_period_s_),
      std::bind(&VoxgraphLocalArea::prune, local_area_.get()));

  // Setup the spatial hash
  voxgraph_spatial_hash_pub_ =
      nh_private.advertise<visualization_msgs::MarkerArray>("spatial_hash", 1,
                                                            true);

  // Setup the new voxgraph submap callback
  voxgraph_server_->setExternalNewSubmapCallback([&] {
    // Update the spatial submap ID hash
    voxgraph_spatial_hash_.update(voxgraph_server_->getSubmapCollection());
    if (0 < voxgraph_spatial_hash_pub_.getNumSubscribers()) {
      voxgraph_spatial_hash_.publishSpatialHash(voxgraph_spatial_hash_pub_);
    }

    // If the global planner is a frontier based planner we compute the frontier
    // candidates every time a submap is finished to reduce overhead when
    // switching to global planning.
    auto frontier_evaluator =
        dynamic_cast<SubmapFrontierEvaluator*>(comm_->globalPlanner().get());
    if (frontier_evaluator) {
      SubmapData datum;
      datum.id = voxgraph_server_->getSubmapCollection().getLastSubmapId();
      // Copy construct the submap s.t. the local are
      datum.tsdf_layer =
          std::make_shared<const voxblox::Layer<voxblox::TsdfVoxel>>(
              voxgraph_server_->getSubmapCollection()
                  .getSubmap(datum.id)
                  .getTsdfMap()
                  .getTsdfLayer());
      Point initial_point(0.0, 0.0, 0.0);  // The origin is always free space.
      frontier_evaluator->computeFrontiersForSubmap(datum, initial_point);
    }

    // If the global planner is a skeleton planner,
    // add a new skeleton submap corresponding to the new voxgraph submap
    auto skeleton_planner =
        dynamic_cast<SkeletonPlanner*>(comm_->globalPlanner().get());
    if (skeleton_planner) {
      voxgraph::SubmapID new_submap_id =
          voxgraph_server_->getSubmapCollection().getLastSubmapId();
      voxgraph::VoxgraphSubmap::ConstPtr new_submap_ptr =
          voxgraph_server_->getSubmapCollection().getSubmapConstPtr(
              new_submap_id);
      skeleton_planner->addSubmap(
          std::move(new_submap_ptr),
          static_cast<float>(config_.traversability_radius));
    }
  });

  // Cached params
  c_voxel_size_ = voxblox_server_->getEsdfMapPtr()->voxel_size();
  c_block_size_ = voxblox_server_->getEsdfMapPtr()->block_size();
}

bool VoxgraphMap::isTraversableInActiveSubmap(const Point& position) {
  if (!comm_->regionOfInterest()->contains(position)) {
    return false;
  }
  double distance = 0.0;
  if (voxblox_server_->getEsdfMapPtr()->getDistanceAtPosition(
          position.cast<double>(), &distance)) {
    // This means the voxel is observed.
    return (distance > config_.traversability_radius);
  }

  return (position - comm_->currentPose().position).norm() <
         config_.clearing_radius;
}

MapBase::VoxelState VoxgraphMap::getVoxelStateInLocalArea(
    const Point& position) {
  // NOTE: The local area consists of the local map + all overlapping global
  //       submaps. We cache and incrementally update the merged global submap
  //       neighborhood. But instead of also merging in the local map, we keep
  //       it separate and perform the lookups in both. This way the cached
  //       neighborhood only needs to be updated when the neighboring global
  //       submaps change. This happens when different submaps start overlapping
  //       with the local map, new submaps are finished or submap poses change
  //       (e.g. every 20s), whereas the local map changes every time a new
  //       pointcloud comes in (e.g. at 10Hz).

  // Start by checking the state in active submap
  double distance;
  if (voxblox_server_->getEsdfMapPtr()->getDistanceAtPosition(
          position.cast<double>(), &distance)) {
    // If getDistanceAtPosition(...) returns true, the voxel is observed
    if (distance > c_voxel_size_) {
      return VoxelState::kFree;
    }
    return VoxelState::kOccupied;
  }

  updateLocalAreaIfNeeded();
  return local_area_->getVoxelStateAtPosition(position);
}

Point VoxgraphMap::getVoxelCenterInLocalArea(const Point& position) {
  return (position / c_voxel_size_).array().round() * c_voxel_size_;
}

void VoxgraphMap::updateLocalAreaIfNeeded() {
  if (local_area_needs_update_) {
    CHECK_NOTNULL(local_area_);

    local_area_->update(voxgraph_server_->getSubmapCollection(),
                        voxgraph_spatial_hash_,
                        *voxblox_server_->getEsdfMapPtr());
    local_area_needs_update_ = false;

    if (0 < local_area_pub_.getNumSubscribers()) {
      local_area_->publishLocalArea(local_area_pub_);
    }
  }
}

bool VoxgraphMap::isObservedInGlobalMap(const Point& position) {
  // Start by checking the state in active submap
  if (voxblox_server_->getEsdfMapPtr()->isObserved(position.cast<double>())) {
    return true;
  }

  // Then fall back to local area
  updateLocalAreaIfNeeded();
  if (local_area_->isObserved(position)) {
    return true;
  }

  // As a last resort, check the submaps in the global map that overlap with
  // the queried position
  for (const voxgraph::SubmapID submap_id :
       voxgraph_spatial_hash_.getSubmapsAtPosition(position)) {
    voxgraph::VoxgraphSubmap::ConstPtr submap_ptr =
        voxgraph_server_->getSubmapCollection().getSubmapConstPtr(submap_id);
    if (submap_ptr) {
      Point local_position = submap_ptr->getPose().inverse() * position;
      if (submap_ptr->getEsdfMap().isObserved(local_position.cast<double>())) {
        return true;
      }
    }
  }
  return false;
}

bool VoxgraphMap::isTraversableInGlobalMap(const Point& position) {
  if (!comm_->regionOfInterest()->contains(position)) {
    return false;
  }

  // Discard early if the point isn't traversable in the local area
  updateLocalAreaIfNeeded();
  // NOTE: We can only check whether the local area is not occupied. Since the
  //       local area only consists of a TSDF (no ESDF) and the traversability
  //       radius generally exceeds the TSDF truncation distance.
  if (local_area_->getVoxelStateAtPosition(position) == VoxelState::kOccupied) {
    return false;
  }

  // Check the submaps that overlap with the queried position
  bool traversable_anywhere = false;
  for (const voxgraph::SubmapID submap_id :
       voxgraph_spatial_hash_.getSubmapsAtPosition(position)) {
    double distance = 0.0;
    voxgraph::VoxgraphSubmap::ConstPtr submap_ptr =
        voxgraph_server_->getSubmapCollection().getSubmapConstPtr(submap_id);
    if (submap_ptr) {
      Point local_position = submap_ptr->getPose().inverse() * position;
      if (submap_ptr->getEsdfMap().getDistanceAtPosition(
              local_position.cast<double>(), &distance)) {
        // This means the voxel is observed.
        if (distance <= config_.traversability_radius) {
          return false;
        } else {
          traversable_anywhere = true;
        }
      }
    }
  }
  // Avoid allowing never observed points to be traversable. Also we ignore the
  // clearing radius for global planning.
  return traversable_anywhere;
}

std::vector<MapBase::SubmapData> VoxgraphMap::getAllSubmapData() {
  // Add all submap pointers and poses data for global frontier computation.
  // Since the submaps are frozen after insertion to the collection we can
  // directly use them by returning a pointer.
  std::vector<SubmapData> data;
  auto submaps = voxgraph_server_->getSubmapCollection().getSubmapConstPtrs();
  for (const auto& submap : submaps) {
    SubmapData datum;
    datum.id = submap->getID();
    datum.T_M_S = submap->getPose();
    datum.tsdf_layer = std::make_shared<voxblox::Layer<voxblox::TsdfVoxel>>(
        submap->getTsdfMap().getTsdfLayer());
    data.push_back(datum);
  }
  return data;
}

bool VoxgraphMap::isLineTraversableInActiveSubmap(
    const Point& start_point, const Point& end_point,
    Point* last_traversable_point) {
  const FloatingPoint line_length = (end_point - start_point).norm();
  const Point line_direction = (end_point - start_point) / line_length;
  FloatingPoint traveled_distance = 0.f;
  Point current_position = start_point;
  Point previous_position = start_point;
  CHECK(c_voxel_size_ < config_.traversability_radius);
  while (true) {
    double esdf_distance = 0.0;
    bool collided;

    if (voxblox_server_->getEsdfMapPtr()->getDistanceAtPosition(
            current_position.cast<double>(), &esdf_distance)) {
      // This means the voxel is observed.
      collided = esdf_distance <= config_.traversability_radius;
    } else {
      // Check whether we're within the clearing distance.
      collided = (current_position - comm_->currentPose().position).norm() >=
                 config_.clearing_radius;
    }
    if (collided) {
      if (last_traversable_point) {
        *last_traversable_point = previous_position;
      }
      return false;
    }
    if (traveled_distance >= line_length) {
      if (last_traversable_point) {
        *last_traversable_point = end_point;
      }
      return true;
    }
    const FloatingPoint step_size =
        std::max(c_voxel_size_, static_cast<FloatingPoint>(esdf_distance) -
                                    config_.traversability_radius);
    previous_position = current_position;
    current_position += step_size * line_direction;
    traveled_distance += step_size;
  }
}

bool VoxgraphMap::getDistanceAndGradientAtPositionInActiveSubmap(
    const Point& position, FloatingPoint* distance, Point* gradient) {
  CHECK_NOTNULL(distance);
  CHECK_NOTNULL(gradient);
  double distance_tmp;
  Eigen::Vector3d gradient_tmp;
  if (voxblox_server_->getEsdfMapPtr()->getDistanceAndGradientAtPosition(
          position.cast<double>(), &distance_tmp, &gradient_tmp)) {
    *distance = static_cast<FloatingPoint>(distance_tmp);
    *gradient = gradient_tmp.cast<FloatingPoint>();
    return true;
  } else {
    return false;
  }
}

bool VoxgraphMap::isLineTraversableInGlobalMap(const Point& start_point,
                                               const Point& end_point,
                                               Point* last_traversable_point) {
  const FloatingPoint line_length = (end_point - start_point).norm();
  const Point line_direction = (end_point - start_point) / line_length;
  FloatingPoint traveled_distance = 0.f;
  Point current_position = start_point;
  Point previous_position = start_point;

  CHECK(c_voxel_size_ < config_.traversability_radius);
  while (traveled_distance <= line_length) {
    FloatingPoint esdf_distance = 0.f;
    if (!getDistanceInGlobalMapAtPosition(current_position, &esdf_distance) ||
        esdf_distance < config_.traversability_radius) {
      if (last_traversable_point) {
        *last_traversable_point = previous_position;
      }
      return false;
    }
    previous_position = current_position;
    const FloatingPoint step_size =
        std::max(c_voxel_size_, esdf_distance - config_.traversability_radius);
    current_position += step_size * line_direction;
    traveled_distance += step_size;
  }

  if (last_traversable_point) {
    *last_traversable_point = end_point;
  }
  return true;
}

bool VoxgraphMap::getDistanceInGlobalMapAtPosition(
    const Point& position, FloatingPoint* min_esdf_distance) {
  CHECK_NOTNULL(min_esdf_distance);

  if (!comm_->regionOfInterest()->contains(position)) {
    return false;
  }

  // Check the submaps that overlap with the queried position
  bool distance_available_anywhere = false;
  *min_esdf_distance = std::numeric_limits<FloatingPoint>::max();
  for (const voxgraph::SubmapID submap_id :
       voxgraph_spatial_hash_.getSubmapsAtPosition(position)) {
    voxgraph::VoxgraphSubmap::ConstPtr submap_ptr =
        voxgraph_server_->getSubmapCollection().getSubmapConstPtr(submap_id);
    if (submap_ptr) {
      Point local_position = submap_ptr->getPose().inverse() * position;
      double submap_esdf_distance = 0.0;
      if (submap_ptr->getEsdfMap().getDistanceAtPosition(
              local_position.cast<double>(), &submap_esdf_distance)) {
        // This means the voxel is observed.
        *min_esdf_distance =
            std::min(*min_esdf_distance,
                     static_cast<FloatingPoint>(submap_esdf_distance));
        distance_available_anywhere = true;
      }
    }
  }

  return distance_available_anywhere;
}

}  // namespace glocal_exploration
