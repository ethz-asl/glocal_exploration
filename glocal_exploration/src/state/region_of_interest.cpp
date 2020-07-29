#include "glocal_exploration/state/region_of_interest.h"

#include "glocal_exploration/utility/config_checker.h"

namespace glocal_exploration {

bool BoundingBox::Config::isValid() const {
  ConfigChecker checker("BoundingBox");
  checker.check(x_max > x_min, "x_max is expected > x_min.");
  checker.check(y_max > y_min, "y_max is expected > y_min.");
  checker.check(z_max > z_min, "z_max is expected > z_min.");
  return checker.isValid();
}

BoundingBox::Config BoundingBox::Config::checkValid() const {
  CHECK(isValid());
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
<<<<<<< HEAD
    : RegionOfInterest(), config_(config.isValid()) {}
=======
    : RegionOfInterest(), config_(config.checkValid()) {}
>>>>>>> 1bf1cdbfe193766c3e1255aa35582574ee52cb3f

}  // namespace glocal_exploration
