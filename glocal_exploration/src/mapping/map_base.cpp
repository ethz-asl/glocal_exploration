#include "glocal_exploration/mapping/map_base.h"

#include <algorithm>
#include <vector>

namespace glocal_exploration {

bool MapBase::findNearbyTraversablePoint(
    const FloatingPoint traversability_radius, Point* position) const {
  CHECK_NOTNULL(position);
  const Point initial_position = *position;

  constexpr int kMaxNumSteps = 20;

  FloatingPoint distance;
  Point gradient;
  int step_idx = 1;
  for (; step_idx < kMaxNumSteps; ++step_idx) {
    // Determine if we found a solution
    if (this->isTraversableInActiveSubmap(*position, traversability_radius)) {
      LOG(INFO) << "Skeleton planner: Succesfully moved point from "
                   "intraversable initial position ("
                << initial_position.transpose()
                << ") to traversable start point (" << position->transpose()
                << "), after " << step_idx << " gradient ascent steps.";
      return true;
    }
    // Get the distance
    if (!this->getDistanceAndGradientInActiveSubmap(*position, &distance,
                                                    &gradient)) {
      LOG(WARNING) << "Failed to look up distance and gradient "
                      "information at: "
                   << position->transpose();
      return false;
    }
    // Take a step in the direction that maximizes the distance
    const FloatingPoint step_size =
        std::max(this->getVoxelSize(), traversability_radius - distance);
    *position += step_size * gradient;
  }
  return false;
}

bool MapBase::findSafestNearbyPoint(const FloatingPoint minimum_distance,
                                    Point* position) {
  CHECK_NOTNULL(position);
  const Point initial_position = *position;

  // Start by trying gradient ascent from the current position.
  if (performGradientAscentFromStartPoint(minimum_distance, position)) {
    return true;
  }

  // If this fails, try nearby start points.
  // Setup.
  const FloatingPoint traversability_radius = getTraversabilityRadius();
  const std::vector<FloatingPoint> scales{getVoxelSize(), 2 * getVoxelSize(),
                                          3 * getVoxelSize()};
  // Search.
  FloatingPoint best_distance = 0.f;
  Point best_position = initial_position;
  for (const FloatingPoint scale : scales) {
    for (const Point& offset :
         safe_nearby_point_search_offsets_.getVertices()) {
      Point start_point = initial_position + scale * offset;
      if (isLineTraversableInActiveSubmap(initial_position, start_point,
                                          traversability_radius, nullptr,
                                          /* optimistic= */ true)) {
        if (performGradientAscentFromStartPoint(minimum_distance,
                                                &start_point)) {
          FloatingPoint new_distance = 0.f;
          if (getDistanceInActiveSubmap(start_point, &new_distance) &&
              best_distance < new_distance &&
              isLineTraversableInActiveSubmap(initial_position, start_point,
                                              traversability_radius, nullptr,
                                              /* optimistic= */ false)) {
            best_distance = new_distance;
            best_position = start_point;
          }
        }
      }
    }
    if (minimum_distance < best_distance) {
      *position = best_position;
      return true;
    }
  }

  return false;
}

bool MapBase::performGradientAscentFromStartPoint(
    const FloatingPoint minimum_distance, Point* position) const {
  CHECK_NOTNULL(position);
  const Point initial_position = *position;

  constexpr int kMaxNumSteps = 80;
  const FloatingPoint voxel_size = this->getVoxelSize();

  Point current_position = initial_position;
  FloatingPoint best_distance_so_far = 0.f;
  constexpr FloatingPoint kMaxMapDistance = 1.9;
  Point best_position_so_far = initial_position;
  int step_idx = 1;
  Point gradient;
  for (; step_idx <= kMaxNumSteps; ++step_idx) {
    // Get the distance.
    FloatingPoint distance = 0.f;
    if (!this->getDistanceAndGradientInActiveSubmap(current_position, &distance,
                                                    &gradient) &&
        step_idx == 1) {
      return false;
    }

    // Save the new position if it is better, or see if it's time to terminate.
    if (best_distance_so_far < distance) {
      best_distance_so_far = distance;
      best_position_so_far = current_position;
    } else if (kMaxMapDistance <= distance ||
               distance + voxel_size / 2 < best_distance_so_far ||
               step_idx == kMaxNumSteps) {
      if (this->isTraversableInActiveSubmap(current_position,
                                            minimum_distance)) {
        LOG(INFO) << "Found a safe point near initial position ("
                  << initial_position.transpose() << "), at ("
                  << best_position_so_far.transpose() << ") with distance "
                  << best_distance_so_far << " after " << step_idx
                  << " gradient ascent steps.";
        *position = best_position_so_far;
        return true;
      }
    }

    // Take a tiny step in the direction that maximizes the distance.
    const FloatingPoint step_size = voxel_size;
    current_position += step_size * gradient;
  }

  return false;
}

}  // namespace glocal_exploration
