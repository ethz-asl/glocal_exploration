#include "glocal_exploration_ros/mapping/voxblox_map.h"

#include <glocal_exploration/common.h>

namespace glocal_exploration {

VoxbloxMap::VoxbloxMap(const std::shared_ptr<StateMachine>& state_machine)
    : MapBase(state_machine) {};

bool VoxbloxMap::setupFromConfig(MapBase::Config* config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config*>(config);
  if (!cfg) {
    LOG(ERROR)
        << "Failed to setup: config is not of type 'VoxbloxMap::Config'.";
    return false;
  }
  config_ = *cfg;
  ros::NodeHandle nh_private(config_.nh_private_namespace);
  ros::NodeHandle nh(ros::names::parentNamespace(config_.nh_private_namespace));
  server_ = std::make_unique<voxblox::EsdfServer>(nh, nh_private);
  c_voxel_size_ = server_->getEsdfMapPtr()->voxel_size();
  c_block_size_ = server_->getEsdfMapPtr()->block_size();
}

double VoxbloxMap::getVoxelSize() { return c_voxel_size_; }

bool VoxbloxMap::isTraversableInActiveSubmap(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  if (!state_machine_->pointInROI(position)) {
    return false;
  }
  double distance = 0.0;
  if (server_->getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
    // This means the voxel is observed
    return (distance > config_.traversability_radius);
  }
  return (position - state_machine_->currentPose().position()).norm() <
      config_.clearing_radius;
}

MapBase::VoxelState VoxbloxMap::getVoxelStateInLocalArea(
    const Eigen::Vector3d& point) {
  double distance = 0.0;
  if (server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
    // This means the voxel is observed
    if (distance > c_voxel_size_) {
      return VoxelState::Free;
    }
    return VoxelState::Occupied;
  }
  return VoxelState::Unknown;
}

Eigen::Vector3d VoxbloxMap::getVoxelCenterInLocalArea(const Eigen::Vector3d& point) {
  return (point / c_voxel_size_).array().round() * c_voxel_size_;
}

}  // namespace glocal_exploration