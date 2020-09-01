#include "glocal_exploration/planning/global/submap_frontier.h"

#include <vector>

namespace glocal_exploration {

void Frontier::addPoint(const FrontierCandidate& point) {
  points_.emplace_back(point);
}

void Frontier::setPoints(const std::vector<FrontierCandidate>& points) {
  points_ = points;
}

void Frontier::computeCentroid(bool count_inactive_points) {
  FloatingPoint count = 0.0;
  centroid_ = Point(0.0, 0.0, 0.0);
  for (const auto& point : points_) {
    if (count_inactive_points || point.is_active) {
      centroid_ += point.position;
      count += 1.0;
    }
  }
  if (count > 0.0) {
    centroid_ /= count;
  }
}

void Frontier::applyTransformation(const Transformation& transformation) {
  for (FrontierCandidate& point : points_) {
    point.position = transformation * point.position;
  }
}

FrontierCollection::FrontierCollection(int id) : id_(id) {
  T_M_S_prev_.setIdentity();
}

Frontier& FrontierCollection::addFrontier() {
  frontiers_.emplace_back(Frontier());
  return frontiers_.back();
}

void FrontierCollection::updateFrontierFrame(const Transformation& T_M_S) {
  if (T_M_S == T_M_S_prev_) {
    return;
  }
  Transformation T_new_prev = T_M_S * T_M_S_prev_.inverse();
  for (Frontier& frontier : frontiers_) {
    frontier.applyTransformation(T_new_prev);
  }
  T_M_S_prev_ = T_M_S;
}

std::vector<Frontier const*> FrontierCollection::getActiveFrontiers() const {
  std::vector<Frontier const*> result;
  for (const Frontier& frontier : frontiers_) {
    if (frontier.isActive()) {
      result.push_back(&frontier);
    }
  }
  return result;
}

}  // namespace glocal_exploration
