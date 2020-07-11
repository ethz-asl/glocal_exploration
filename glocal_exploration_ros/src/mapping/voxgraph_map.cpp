#include "glocal_exploration_ros/mapping/voxgraph_map.h"

#include <memory>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include "voxblox_ros/ptcloud_vis.h"

namespace glocal_exploration {

VoxgraphMap::VoxgraphMap(const std::shared_ptr<StateMachine>& state_machine)
    : MapBase(state_machine), local_area_needs_update_(false) {}

bool VoxgraphMap::setupFromConfig(MapBase::Config* config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config*>(config);
  if (!cfg) {
    LOG(ERROR)
        << "Failed to setup: config is not of type 'VoxgraphMap::Config'.";
    return false;
  }
  config_ = *cfg;

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

  // Cached params
  c_voxel_size_ = voxblox_server_->getEsdfMapPtr()->voxel_size();
  c_block_size_ = voxblox_server_->getEsdfMapPtr()->block_size();
}

bool VoxgraphMap::isTraversableInActiveSubmap(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  if (!state_machine_->pointInROI(position)) {
    return false;
  }
  double distance = 0.0;
  if (voxblox_server_->getEsdfMapPtr()->getDistanceAtPosition(position,
                                                              &distance)) {
    // This means the voxel is observed
    return (distance > config_.traversability_radius);
  }
  return (position - state_machine_->currentPose().position()).norm() <
         config_.clearing_radius;
}

MapBase::VoxelState VoxgraphMap::getVoxelStateInLocalArea(
    const Eigen::Vector3d& position) {
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
      return VoxelState::Free;
    }
    return VoxelState::Occupied;
  }

  if (local_area_needs_update_) {
    updateLocalArea();
  }
  return local_area_->getVoxelStateAtPosition(position);
}

Eigen::Vector3d VoxgraphMap::getVoxelCenterInLocalArea(
    const Eigen::Vector3d& point) {
  return (point / c_voxel_size_).array().round() * c_voxel_size_;
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

}  // namespace glocal_exploration
