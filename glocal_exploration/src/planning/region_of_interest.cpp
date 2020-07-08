#include "glocal_exploration/planning/region_of_interest.h"

namespace glocal_exploration {

bool BoundingBox::contains(const Eigen::Vector3d &point) {
  if (point.x() > config_.x_max) { return false; }
  if (point.y() > config_.y_max) { return false; }
  if (point.z() > config_.z_max) { return false; }
  if (point.x() < config_.x_min) { return false; }
  if (point.y() < config_.y_min) { return false; }
  return point.z() >= config_.z_min;
}

bool BoundingBox::setupFromConfig(RegionOfInterest::Config *config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config *>(config);
  if (!cfg) {
    LOG(ERROR) << "Failed to setup: config is not of type 'BoundingBox::Config'.";
    return false;
  }
  config_ = *cfg;
}

} // namespace glocal_exploration
