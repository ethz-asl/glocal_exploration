#include "glocal_exploration/planning/global/submap_frontier.h"

#include <vector>


namespace glocal_exploration {

void Frontier::addPoint(const Point& point) {
  points_.emplace_back(FrontierCandidate(point));
}

void Frontier::setPoints(const std::vector<Point>& points) {
  points_.clear();
  points_.reserve(points.size());
  for (const auto& point : points) {
    points_.emplace_back(FrontierCandidate(point));
  }
}

void Frontier::computeCentroid(bool only_active_frontiers) {
  FloatingPoint count = 0;
  centroid_ = Point(0, 0, 0);
  for (const auto& pt : points_) {
    if (!only_active_frontiers || pt.is_active) {
      centroid_ += pt.position;
      count += 1;
    }
  }
  centroid_ /= count;
}

}  // namespace glocal_exploration
