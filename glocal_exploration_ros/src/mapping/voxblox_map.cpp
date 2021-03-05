#include "glocal_exploration_ros/mapping/voxblox_map.h"

#include <algorithm>
#include <memory>
#include <vector>

#include <glocal_exploration/common.h>
#include <glocal_exploration/state/communicator.h>

namespace glocal_exploration {

VoxbloxMap::Config::Config() { setConfigName("VoxbloxMap"); }

void VoxbloxMap::Config::checkParams() const {
  checkParamGT(traversability_radius, 0.f, "traversability_radius");
}

void VoxbloxMap::Config::fromRosParam() {
  rosParam("traversability_radius", &traversability_radius);
  rosParam("clearing_radius", &clearing_radius);
  nh_private_namespace = rosParamNameSpace();
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

bool VoxbloxMap::isTraversableInActiveSubmap(
    const Point& position, const FloatingPoint traversability_radius,
    const bool optimistic) const {
  if (!comm_->regionOfInterest()->contains(position)) {
    return false;
  }
  FloatingPoint distance = 0.f;
  if (getDistanceInActiveSubmap(position, &distance)) {
    // This means the voxel is observed.
    return (distance > traversability_radius);
  } else {
    const bool within_clear_sphere =
        (position - comm_->currentPose().position).norm() <=
        config_.clearing_radius;
    return optimistic || within_clear_sphere;
  }
}

bool VoxbloxMap::isLineTraversableInActiveSubmap(
    const Point& start_point, const Point& end_point,
    const FloatingPoint traversability_radius, Point* last_traversable_point,
    const bool optimistic) {
  CHECK_GT(c_voxel_size_, 0.f);
  if (last_traversable_point) {
    *last_traversable_point = start_point;
  }

  const FloatingPoint line_length = (end_point - start_point).norm();
  if (line_length <= voxblox::kFloatEpsilon) {
    return isTraversableInActiveSubmap(start_point, traversability_radius,
                                       optimistic);
  } else if (kMaxLineTraversabilityCheckLength < line_length) {
    LOG(WARNING) << "Requested traversability check for segment exceeding "
                 << kMaxLineTraversabilityCheckLength
                 << "m. Returning false to avoid long wait.";
    return false;
  }

  const Point line_direction = (end_point - start_point) / line_length;
  Point current_position = start_point;

  FloatingPoint traveled_distance = 0.f;
  while (traveled_distance <= line_length) {
    FloatingPoint esdf_distance = 0.f;
    if (getDistanceInActiveSubmap(current_position, &esdf_distance)) {
      // This means the voxel is observed.
      if (esdf_distance < traversability_radius) {
        return false;
      }
    } else {
      // Check whether we're within the clearing distance.
      const bool within_clear_sphere =
          (current_position - comm_->currentPose().position).norm() <=
          config_.clearing_radius;
      if (optimistic || within_clear_sphere) {
        esdf_distance = 0.f;
      } else {
        return false;
      }
    }

    if (last_traversable_point) {
      *last_traversable_point = current_position;
    }
    const FloatingPoint step_size =
        std::max(c_voxel_size_, esdf_distance - traversability_radius);
    current_position += step_size * line_direction;
    traveled_distance += step_size;
  }

  if (isTraversableInActiveSubmap(end_point, traversability_radius,
                                  optimistic)) {
    if (last_traversable_point) {
      *last_traversable_point = end_point;
    }
    return true;
  } else {
    return false;
  }
}

bool VoxbloxMap::lineIntersectsSurfaceInActiveSubmap(const Point& start_point,
                                                     const Point& end_point) {
  CHECK_GT(c_voxel_size_, 0.f);

  const FloatingPoint line_length = (end_point - start_point).norm();
  if (line_length <= voxblox::kFloatEpsilon) {
    if (isOccupiedInActiveSubmap(start_point)) {
      return true;
    }
  }

  const Point line_direction = (end_point - start_point) / line_length;
  Point current_position = start_point;

  FloatingPoint traveled_distance = 0.f;
  while (traveled_distance <= line_length) {
    FloatingPoint esdf_distance = 0.f;
    if (getDistanceInActiveSubmap(current_position, &esdf_distance) &&
        esdf_distance < c_voxel_size_) {
      return true;
    }

    const FloatingPoint step_size =
        std::max(c_voxel_size_, esdf_distance - c_voxel_size_);
    current_position += step_size * line_direction;
    traveled_distance += step_size;
  }

  if (isOccupiedInActiveSubmap(end_point)) {
    return true;
  } else {
    return false;
  }
}

bool VoxbloxMap::getDistanceInActiveSubmap(const Point& position,
                                           FloatingPoint* distance) const {
  CHECK_NOTNULL(distance);
  double distance_tmp;
  if (server_->getEsdfMapPtr()->getDistanceAtPosition(position.cast<double>(),
                                                      &distance_tmp)) {
    *distance = static_cast<FloatingPoint>(distance_tmp);
    return true;
  } else {
    return false;
  }
}

bool VoxbloxMap::getDistanceAndGradientInActiveSubmap(const Point& position,
                                                      FloatingPoint* distance,
                                                      Point* gradient) const {
  CHECK_NOTNULL(distance);
  CHECK_NOTNULL(gradient);
  double distance_tmp;
  Eigen::Vector3d gradient_tmp;
  if (server_->getEsdfMapPtr()->getDistanceAndGradientAtPosition(
          position.cast<double>(), &distance_tmp, &gradient_tmp)) {
    *distance = static_cast<FloatingPoint>(distance_tmp);
    *gradient = gradient_tmp.cast<FloatingPoint>();
    return true;
  } else {
    return false;
  }
}

MapBase::VoxelState VoxbloxMap::getVoxelStateInLocalArea(
    const Point& position) {
  FloatingPoint distance = 0.f;
  if (getDistanceInActiveSubmap(position, &distance)) {
    // This means the voxel is observed
    if (distance > c_voxel_size_) {
      return VoxelState::kFree;
    }
    return VoxelState::kOccupied;
  }
  return VoxelState::kUnknown;
}

std::vector<MapBase::SubmapData> VoxbloxMap::getAllSubmapData() {
  std::vector<SubmapData> data;
  SubmapData datum;
  datum.id = 0;
  datum.T_M_S.setIdentity();
  datum.tsdf_layer = std::make_shared<const voxblox::Layer<voxblox::TsdfVoxel>>(
      server_->getTsdfMapPtr()->getTsdfLayer());
  data.push_back(datum);
  return data;
}

}  // namespace glocal_exploration
