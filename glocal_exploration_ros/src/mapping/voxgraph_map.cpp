#include "glocal_exploration_ros/mapping/voxgraph_map.h"

#include <algorithm>
#include <memory>
#include <vector>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <voxblox_ros/ptcloud_vis.h>

#include <glocal_exploration/state/communicator.h>
#include <glocal_exploration/utility/config_checker.h>

namespace glocal_exploration {

bool VoxgraphMap::Config::isValid() const {
  ConfigChecker checker("VoxgraphMap");
  // TODO(@victorr): check param validity
  checker.check_gt(traversability_radius, 0.0, "traversability_radius");
  return checker.isValid();
}

VoxgraphMap::Config VoxgraphMap::Config::checkValid() const {
  CHECK(isValid());
  return Config(*this);
}

VoxgraphMap::VoxgraphMap(const Config& config,
                         const std::shared_ptr<Communicator>& communicator)
    : MapBase(communicator),
      config_(config.checkValid()),
      local_area_needs_update_(false) {
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

  // Cached params
  c_voxel_size_ = voxblox_server_->getEsdfMapPtr()->voxel_size();
  c_block_size_ = voxblox_server_->getEsdfMapPtr()->block_size();
}

bool VoxgraphMap::isTraversableInActiveSubmap(const Point& position) {
  if (!comm_->regionOfInterest()->contains(position)) {
    return false;
  }
  double distance = 0.0;
  if (voxblox_server_->getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    // This means the voxel is observed
    return (distance > config_.traversability_radius);
  }
  return (position - comm_->currentPose().position()).norm() <
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
  if (voxblox_server_->getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    // If getDistanceAtPosition(...) returns true, the voxel is observed
    if (distance > c_voxel_size_) {
      return VoxelState::kFree;
    }
    return VoxelState::kOccupied;
  }

  if (local_area_needs_update_) {
    updateLocalArea();
  }
  return local_area_->getVoxelStateAtPosition(position);
}

Point VoxgraphMap::getVoxelCenterInLocalArea(const Point& position) {
  return (position / c_voxel_size_).array().round() * c_voxel_size_;
}

void VoxgraphMap::updateLocalArea() {
  CHECK_NOTNULL(local_area_);

  local_area_->update(voxgraph_server_->getSubmapCollection(),
                      *voxblox_server_->getEsdfMapPtr());
  local_area_needs_update_ = false;

  if (local_area_pub_.getNumSubscribers() > 0) {
    local_area_->publishLocalArea(local_area_pub_);
  }
}

bool VoxgraphMap::isObservedInGlobalMap(const Point& position) {
  // TODO(@victorr): Don't know if you know of a nicer way to check this :)

  auto submaps = voxgraph_server_->getSubmapCollection().getSubmapConstPtrs();
  return std::any_of(submaps.begin(), submaps.end(),
                     [position](const voxgraph::VoxgraphSubmap::ConstPtr& s) {
                       return s->getEsdfMap().isObserved(position);
                     });
}

void VoxgraphMap::getAllSubmapData(std::vector<SubmapData>* data) {
  // TODO(@victorr): This is a first implementation for global frontier
  //  tracking. The global planner has a function computeFrontiers (or similar)
  //  that can be called upon submap completion to store frontiers.
  //  This function is solely needed to guarantee all maps get frontiers, the
  //  evaluator has a param sumaps_are_fronzen that prevents it from recomputing
  //  frontiers for a given ID. Btw I don't know this includes the active
  //  submap, but it should not be included here.

  CHECK_NOTNULL(data);
  auto submaps = voxgraph_server_->getSubmapCollection().getSubmapConstPtrs();
  for (const auto& submap : submaps) {
    SubmapData datum;
    datum.id = submap->getID();
    datum.T_M_S = submap->getPose().cast<FloatingPoint>();
    datum.tsdf_layer.reset(submap->getTsdfMap().getTsdfLayerConstPtr());
    data->push_back(datum);
  }
}

}  // namespace glocal_exploration
