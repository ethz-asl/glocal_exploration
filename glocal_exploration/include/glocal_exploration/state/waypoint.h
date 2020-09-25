#ifndef GLOCAL_EXPLORATION_STATE_WAYPOINT_H_
#define GLOCAL_EXPLORATION_STATE_WAYPOINT_H_

#include <utility>

#include "glocal_exploration/common.h"

namespace glocal_exploration {

/**
 * This struct defines a way point that are produced and passed by planners.
 */
struct WayPoint {
  // position m
  Point position = Point(0.0, 0.0, 0.0);
  // orientation rad
  double yaw = 0;

  // Constructors.
  WayPoint() = default;
  WayPoint(Point _position, double _yaw)
      : position(std::move(_position)), yaw(_yaw) {}
  WayPoint(double x, double y, double z, double _yaw)
      : position(x, y, z), yaw(_yaw) {}
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_STATE_WAYPOINT_H_
