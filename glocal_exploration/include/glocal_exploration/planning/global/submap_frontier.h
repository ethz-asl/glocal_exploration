#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_H_

#include <memory>
#include <utility>
#include <vector>

#include "glocal_exploration/common.h"

namespace glocal_exploration {

/**
 * Smallest unit of a frontier, where each candidate describes a point of a
 * submap that could be a frontier.
 */
struct FrontierCandidate {
  FrontierCandidate() = default;
  explicit FrontierCandidate(Point _position)
      : position(std::move(_position)) {}
  bool is_active = false;  // true: this candidate is a frontier point.
  Point position;
};

/**
 * Contains all candidate points that form a connected frontier.
 */
class Frontier {
 public:
  Frontier() = default;
  ~Frontier() = default;

  // point access
  [[nodiscard]] size_t size() const {
    return points_.size();
  } std::vector<FrontierCandidate>::iterator begin() {
    return points_.begin();
  }
  std::vector<FrontierCandidate>::iterator end() { return points_.end(); }

  // accessors
  [[nodiscard]] const Point& centroid() const {
    return centroid_;
  }[[nodiscard]] bool isActive() const {
    return is_active_;
  }

  // interaction
  void addPoint(const Point& point);
  void setPoints(const std::vector<Point>& points);
  void computeCentroid(bool only_active_frontiers = false);

 private:
  std::vector<FrontierCandidate> points_;
  Point centroid_;
  bool is_active_;
};

/**
 * The frontier collection contains all frontiers of a submap.
 */
struct FrontierCollection {
  int id;
  std::vector<Frontier> frontiers;
  Transformation T_M_S;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_H_
