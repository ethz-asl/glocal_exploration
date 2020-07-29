#include "glocal_exploration_ros/mapping/voxblox_map.h"

#include <memory>

#include <glocal_exploration/common.h>
#include <glocal_exploration/state/communicator.h>
#include <glocal_exploration/utility/config_checker.h>

namespace glocal_exploration {

bool VoxbloxMap::Config::isValid() const {
  ConfigChecker checker("VoxbloxMap");
  checker.check_gt(traversability_radius, 0.0, "traversability_radius");
  return checker.isValid();
}

VoxbloxMap::Config VoxbloxMap::Config::checkValid() const {
  CHECK(isValid());
  return Config(*this);
}

VoxbloxMap::VoxbloxMap(const Config& config,
                       const std::shared_ptr<Communicator>& communicator)
    : MapBase(communicator), config_(config.checkValid()) {
  // create a voxblox server
  ros::NodeHandle nh_private(config_.nh_private_namespace);
  ros::NodeHandle nh(ros::names::parentNamespace(config_.nh_private_namespace));
  server_ = std::make_unique<ThreadsafeVoxbloxServer>(nh, nh_private);

  // cache important values
  c_voxel_size_ = server_->getEsdfMapPtr()->voxel_size();
  c_block_size_ = server_->getEsdfMapPtr()->block_size();
}

double VoxbloxMap::getVoxelSize() { return c_voxel_size_; }

bool VoxbloxMap::isTraversableInActiveSubmap(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  if (!comm_->regionOfInterest()->contains(position)) {
    return false;
  }
  double distance = 0.0;
  if (server_->getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
    // This means the voxel is observed
    return (distance > config_.traversability_radius);
  }
  return (position - comm_->currentPose().position()).norm() <
         config_.clearing_radius;
}

MapBase::VoxelState VoxbloxMap::getVoxelStateInLocalArea(
    const Eigen::Vector3d& point) {
  double distance = 0.0;
  if (server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
    // This means the voxel is observed
    if (distance > c_voxel_size_) {
      return VoxelState::kFree;
    }
    return VoxelState::kOccupied;
  }
  return VoxelState::kUnknown;
}

Eigen::Vector3d VoxbloxMap::getVoxelCenterInLocalArea(
    const Eigen::Vector3d& point) {
  return (point / c_voxel_size_).array().round() * c_voxel_size_;
}

}  // namespace glocal_exploration
