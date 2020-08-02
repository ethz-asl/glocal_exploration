#include "glocal_exploration_ros/mapping/voxblox_map.h"

#include <memory>
#include <vector>

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

bool VoxbloxMap::isTraversableInActiveSubmap(const Point& position) {
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
    const Point& position) {
  double distance = 0.0;
  if (server_->getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
    // This means the voxel is observed
    if (distance > c_voxel_size_) {
      return VoxelState::kFree;
    }
    return VoxelState::kOccupied;
  }
  return VoxelState::kUnknown;
}

Point VoxbloxMap::getVoxelCenterInLocalArea(const Point& position) {
  return (position / c_voxel_size_).array().round() * c_voxel_size_;
}

bool VoxbloxMap::isObservedInGlobalMap(const Point& position) {
  return server_->getEsdfMapPtr()->isObserved(position);
}

void VoxbloxMap::getAllSubmapData(std::vector<SubmapData>* data) {
  CHECK_NOTNULL(data);
  SubmapData datum;
  datum.id = 0;
  datum.T_M_S.setIdentity();
  datum.tsdf_layer = std::make_shared<const voxblox::Layer<voxblox::TsdfVoxel>>(
      server_->getTsdfMapPtr()->getTsdfLayer());
  data->push_back(datum);
}

}  // namespace glocal_exploration
