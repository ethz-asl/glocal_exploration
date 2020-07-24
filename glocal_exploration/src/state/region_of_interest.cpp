#include "glocal_exploration/state/region_of_interest.h"

namespace glocal_exploration {

BoundingBox::Config BoundingBox::Config::isValid() const {
  CHECK_GT(x_max, x_min) << "The bounding box encompasses no volume.";
  CHECK_GT(y_max, y_min) << "The bounding box encompasses no volume.";
  CHECK_GT(z_max, z_min) << "The bounding box encompasses no volume.";
  return Config(*this);
}

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
    : RegionOfInterest(), config_(config.isValid()) {}

}  // namespace glocal_exploration
