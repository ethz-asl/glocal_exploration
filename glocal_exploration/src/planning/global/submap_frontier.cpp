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

void Frontier::applyTransformation(const Transformation& transformation) {
  for (FrontierCandidate& point : points_) {
    point.position = transformation * point.position;
  }
}

FrontierCollection::FrontierCollection(int id,
                                       const Transformation& T_M_S_initial)
    : id_(id), T_M_S_prev_(T_M_S_initial) {}

Frontier& FrontierCollection::addFrontier() {
  frontiers_.emplace_back(Frontier());
  return frontiers_.back();
}

void FrontierCollection::transformFrontiers(const Transformation& T_M_S) {
  if (T_M_S == T_M_S_prev_) {
    return;
  }
  Transformation T_new_prev = T_M_S.inverse() * T_M_S_prev_;
  for (Frontier& frontier : frontiers_) {
    frontier.applyTransformation(T_new_prev);
  }
  T_M_S_prev_ = T_M_S;
}

}  // namespace glocal_exploration
