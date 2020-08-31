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
 * Contains all candidate points that form a connected frontier. All points are
 * in Mission frame (M), where transforms are managed by the FrontierCollection.
 */
class Frontier {
 public:
  Frontier() = default;
  virtual ~Frontier() = default;

  // point access
  size_t size() const { return points_.size(); }
  std::vector<FrontierCandidate>::const_iterator begin() const {
    return points_.begin();
  }
  std::vector<FrontierCandidate>::const_iterator end() const {
    return points_.end();
  }
  std::vector<FrontierCandidate>::iterator begin() { return points_.begin(); }
  std::vector<FrontierCandidate>::iterator end() { return points_.end(); }

  // accessors
  const Point& getCentroid() const { return centroid_; }
  bool isActive() const { return is_active_; }

  // interaction
  void addPoint(const Point& point);
  void setPoints(const std::vector<Point>& points);
  void computeCentroid(bool only_active_frontiers = false);
  void applyTransformation(const Transformation& transformation);
  void setIsActive(bool is_active) { is_active_ = is_active; }

 private:
  std::vector<FrontierCandidate> points_;
  Point centroid_;
  bool is_active_;
};

/**
 * The frontier collection contains all frontiers of a submap.
 */
class FrontierCollection {
 public:
  FrontierCollection() = default;
  explicit FrontierCollection(int id);
  virtual ~FrontierCollection() = default;

  // frontier access
  size_t size() const { return frontiers_.size(); }
  std::vector<Frontier>::const_iterator begin() const {
    return frontiers_.begin();
  }
  std::vector<Frontier>::const_iterator end() const { return frontiers_.end(); }
  std::vector<Frontier>::iterator begin() { return frontiers_.begin(); }
  std::vector<Frontier>::iterator end() { return frontiers_.end(); }

  // accessors
  int getID() const { return id_; }

  // interaction
  Frontier& addFrontier();
  void updateFrontierFrame(const Transformation& T_M_S);

 private:
  int id_;
  std::vector<Frontier> frontiers_;
  Transformation T_M_S_prev_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIER_H_
