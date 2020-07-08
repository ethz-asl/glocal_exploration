#ifndef GLOCAL_EXPLORATION_PLANNING_WAYPOINT_H_
#define GLOCAL_EXPLORATION_PLANNING_WAYPOINT_H_

#include "glocal_exploration/common.h"

namespace glocal_exploration {

/**
 * This struct defines a way point that are produced and passed by planners
 */
struct WayPoint {
  // position m
  double x = 0;
  double y = 0;
  double z = 0;
  // orientation rad
  double yaw = 0;

  // utilities
  Eigen::Vector3d position() const { return Eigen::Vector3d(x, y, z); }
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_COMMON_H_
