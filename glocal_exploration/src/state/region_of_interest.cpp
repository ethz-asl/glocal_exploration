#include "glocal_exploration/state/region_of_interest.h"

namespace glocal_exploration {

BoundingBox::Config::Config() { setConfigName("BoundingBox"); }

void BoundingBox::Config::checkParams() const {
  checkParamCond(x_max > x_min, "x_max is expected > x_min.");
  checkParamCond(y_max > y_min, "y_max is expected > y_min.");
  checkParamCond(z_max > z_min, "z_max is expected > z_min.");
}

void BoundingBox::Config::fromRosParam() {
  rosParam("x_min", &x_min);
  rosParam("x_max", &x_max);
  rosParam("y_min", &y_min);
  rosParam("y_max", &y_max);
  rosParam("z_min", &z_min);
  rosParam("z_max", &z_max);
}

void BoundingBox::Config::printFields() const {
  printField("x_min", x_min);
  printField("x_max", x_max);
  printField("y_min", y_min);
  printField("y_max", y_max);
  printField("z_min", z_min);
  printField("z_max", z_max);
}

bool BoundingBox::contains(const Point& point) {
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
    : RegionOfInterest(), config_(config.checkValid()) {}

}  // namespace glocal_exploration
