#include "glocal_exploration/state/region_of_interest.h"

namespace glocal_exploration {

bool BoundingBox::contains(const Eigen::Vector3d& point) {
  if (point.x() > config_.x_max) {
    return false;
  }
  if (point.y() > config_.y_max) {
    return false;
  }
  if (point.z() > config_.z_max) {
    return false;
  }
  if (point.x() < config_.x_min) {
    return false;
  }
  if (point.y() < config_.y_min) {
    return false;
  }
  return point.z() >= config_.z_min;
}

BoundingBox::BoundingBox(const Config& config)
    : RegionOfInterest(), config_(config) {}

}  // namespace glocal_exploration
