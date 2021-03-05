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
  Point position = Point(0.f, 0.f, 0.f);
  // orientation rad
  FloatingPoint yaw = 0.f;

  // Constructors.
  WayPoint() = default;
  WayPoint(Point _position, FloatingPoint _yaw)
      : position(std::move(_position)), yaw(_yaw) {}
  WayPoint(FloatingPoint x, FloatingPoint y, FloatingPoint z,
           FloatingPoint _yaw)
      : position(x, y, z), yaw(_yaw) {}
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_STATE_WAYPOINT_H_
