#include "glocal_exploration_ros/mapping/voxblox_map.h"

namespace glocal_exploration {

VoxbloxMap::VoxbloxMap(const std::shared_ptr<StateMachine> &state_machine) : MapBase(state_machine) {};

bool VoxbloxMap::setupFromConfig(MapBase::Config *config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config *>(config);
  if (!cfg) {
    LOG(ERROR) << "Failed to setup: config is not of type 'VoxbloxMap::Config'.";
    return false;
  }
  config_ = *cfg;
  ros::NodeHandle nh_private(config_.nh_private_namespace);
  ros::NodeHandle nh(ros::names::parentNamespace(config_.nh_private_namespace));
  server_ = std::make_unique<voxblox::EsdfServer>(nh, nh_private);
  c_voxel_size_ = server_->getEsdfMapPtr()->voxel_size();
  c_block_size_ = server_->getEsdfMapPtr()->block_size();
}

double VoxbloxMap::getVoxelSize() {
  return c_voxel_size_;
}

bool VoxbloxMap::isTraversableInActiveSubmap(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) {
  double distance = 0.0;
  if (server_->getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
    // This means the voxel is observed
    return (distance > config_.collision_radius);
  }
  return (position - state_machine_->currentPose().position()).norm() < config_.clearing_radius;
}

bool VoxbloxMap::getVoxelCenterInLocalArea(Eigen::Vector3d *center, const Eigen::Vector3d &point) {
  voxblox::BlockIndex block_id = server_->getEsdfMapPtr()->getEsdfLayerPtr()->
      computeBlockIndexFromCoordinates(point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_).cast<double>();
  voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
      (point - *center).cast<voxblox::FloatingPoint>(), 1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_).cast<double>();
  return true;
}

MapBase::VoxelState VoxbloxMap::getVoxelStateInLocalArea(const Eigen::Vector3d &point) {
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

} // namespace glocal_exploration {