#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_RELATIVE_WAYPOINT_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_RELATIVE_WAYPOINT_H_

#include <utility>

#include "glocal_exploration/common.h"
#include "glocal_exploration/planning/global/skeleton/skeleton_submap.h"
#include "glocal_exploration/state/waypoint.h"

namespace glocal_exploration {
class RelativeWayPoint {
 public:
  static constexpr SubmapId kOdomFrameId = -1u;

  // Construct with pose in odom frame
  explicit RelativeWayPoint(Point global_position)
      : local_frame_id_(kOdomFrameId),
        t_local_position_(std::move(global_position)) {}

  // Construct with pose in submap frame
  RelativeWayPoint(SkeletonSubmap::ConstPtr submap_ptr, Point local_position)
      : local_frame_id_(CHECK_NOTNULL(submap_ptr)->getId()),
        t_local_position_(std::move(local_position)),
        submap_ptr_(std::move(submap_ptr)) {}

  // Query which frame the waypoint is in
  bool isInGlobalFrame() const { return local_frame_id_ == kOdomFrameId; }
  SubmapId getFrameId() const { return local_frame_id_; }

  // Position in odom frame
  Point getGlobalPosition() const {
    if (local_frame_id_ == kOdomFrameId) {
      return t_local_position_;
    } else {
      CHECK_NOTNULL(submap_ptr_);
      return submap_ptr_->getPose() * t_local_position_;
    }
  }

  // Position in submap frame
  Point getLocalPosition() const { return t_local_position_; }

  // Convert ("flatten") to normal waypoint in odom frame
  explicit operator WayPoint() const {
    return WayPoint(getGlobalPosition(), 0.f);
  }

 protected:
  SubmapId local_frame_id_ = kOdomFrameId;
  Point t_local_position_ = Point(0.f, 0.f, 0.f);
  SkeletonSubmap::ConstPtr submap_ptr_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_RELATIVE_WAYPOINT_H_
