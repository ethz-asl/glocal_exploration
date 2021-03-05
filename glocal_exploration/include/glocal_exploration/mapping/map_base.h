#ifndef GLOCAL_EXPLORATION_MAPPING_MAP_BASE_H_
#define GLOCAL_EXPLORATION_MAPPING_MAP_BASE_H_

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include <voxblox/core/layer.h>

#include "glocal_exploration/common.h"
#include "glocal_exploration/mapping/neighborhood_offsets.h"
#include "glocal_exploration/state/waypoint.h"

namespace glocal_exploration {

class Communicator;

/**
 * Defines the interface of a map module that is needed by the planner.
 */
class MapBase {
 public:
  enum class VoxelState { kUnknown, kOccupied, kFree };

  struct SubmapData {
    int id;
    Transformation T_M_S;
    std::shared_ptr<const voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;
  };

  explicit MapBase(std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~MapBase() = default;

  /* General and Accessors */
  virtual FloatingPoint getVoxelSize() const = 0;
  virtual FloatingPoint getTraversabilityRadius() const = 0;
  virtual std::vector<WayPoint> getPoseHistory() const = 0;

  /* Local planner */
  bool isTraversableInActiveSubmap(const Point& position) {
    return isTraversableInActiveSubmap(position, getTraversabilityRadius());
  }
  virtual bool isTraversableInActiveSubmap(
      const Point& position, const FloatingPoint traversability_radius,
      const bool optimistic = false) const = 0;

  bool isLineTraversableInActiveSubmap(
      const Point& start_point, const Point& end_point,
      Point* last_traversable_point = nullptr) {
    return isLineTraversableInActiveSubmap(start_point, end_point,
                                           getTraversabilityRadius(),
                                           last_traversable_point);
  }
  virtual bool isLineTraversableInActiveSubmap(
      const Point& start_point, const Point& end_point,
      const FloatingPoint traversability_radius,
      Point* last_traversable_point = nullptr,
      const bool optimistic = false) = 0;
  virtual bool lineIntersectsSurfaceInActiveSubmap(const Point& start_point,
                                                   const Point& end_point) = 0;

  virtual bool getDistanceInActiveSubmap(const Point& position,
                                         FloatingPoint* distance) const = 0;
  virtual bool getDistanceAndGradientInActiveSubmap(const Point& position,
                                                    FloatingPoint* distance,
                                                    Point* gradient) const = 0;

  bool findNearbyTraversablePoint(const FloatingPoint traversability_radius,
                                  Point* position) const;
  bool findSafestNearbyPoint(const FloatingPoint minimum_distance,
                             Point* position);
  bool performGradientAscentFromStartPoint(const FloatingPoint minimum_distance,
                                           Point* position) const;

  // Voxels are referred in the planner by their center points.
  virtual Point getVoxelCenterInLocalArea(const Point& position) const = 0;

  virtual VoxelState getVoxelStateInLocalArea(const Point& position) = 0;

  /* Global planner */
  virtual bool isObservedInGlobalMap(const Point& position) = 0;

  bool isTraversableInGlobalMap(const Point& position) {
    return isTraversableInGlobalMap(position, getTraversabilityRadius());
  }
  virtual bool isTraversableInGlobalMap(
      const Point& position, const FloatingPoint traversability_radius) = 0;

  virtual bool isLineTraversableInGlobalMap(
      const Point& start_point, const Point& end_point,
      Point* last_traversable_point = nullptr) {
    return isLineTraversableInGlobalMap(start_point, end_point,
                                        getTraversabilityRadius(),
                                        last_traversable_point);
  }
  virtual bool isLineTraversableInGlobalMap(
      const Point& start_point, const Point& end_point,
      const FloatingPoint traversability_radius,
      Point* last_traversable_point = nullptr) = 0;
  virtual bool lineIntersectsSurfaceInGlobalMap(const Point& start_point,
                                                const Point& end_point) = 0;

  virtual bool getDistanceInGlobalMap(const Point& position,
                                      FloatingPoint* distance) = 0;

  virtual std::vector<SubmapId> getSubmapIdsAtPosition(
      const Point& position) const = 0;
  virtual std::vector<SubmapData> getAllSubmapData() = 0;

 protected:
  const std::shared_ptr<Communicator> comm_;
  NeighborhoodOffsets safe_nearby_point_search_offsets_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_MAPPING_MAP_BASE_H_
