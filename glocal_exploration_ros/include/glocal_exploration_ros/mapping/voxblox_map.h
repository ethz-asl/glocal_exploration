#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_

#include <memory>
#include <string>
#include <vector>

#include <glocal_exploration/3rd_party/config_utilities.hpp>
#include <glocal_exploration/mapping/map_base.h>

#include "glocal_exploration_ros/mapping/threadsafe_wrappers/threadsafe_voxblox_server.h"

namespace glocal_exploration {
/**
 * Map class that just uses voxblox as a monolithic map baseline.
 */
class VoxbloxMap : public MapBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    // Since this is a ros-class anyways we make it easy and just get the nh.
    std::string nh_private_namespace = "~";
    FloatingPoint traversability_radius = 0.3f;  // m
    FloatingPoint clearing_radius = 0.5f;        // m

    Config();
    void checkParams() const override;
    void fromRosParam() override;
  };

  explicit VoxbloxMap(const Config& config,
                      const std::shared_ptr<Communicator>& communicator);
  ~VoxbloxMap() override = default;

  /* General and Accessors */
  FloatingPoint getVoxelSize() const override { return c_voxel_size_; }
  FloatingPoint getTraversabilityRadius() const override {
    return config_.traversability_radius;
  }
  std::vector<WayPoint> getPoseHistory() const override {
    LOG(ERROR) << "Pose history not implemented for voxblox maps.";
    return std::vector<WayPoint>(0);
  }

  /* Local planner */
  bool isTraversableInActiveSubmap(
      const Point& position, const FloatingPoint traversability_radius,
      const bool optimistic = false) const override;
  bool isLineTraversableInActiveSubmap(
      const Point& start_point, const Point& end_point,
      const FloatingPoint traversability_radius,
      Point* last_traversable_point = nullptr,
      const bool optimistic = false) override;
  bool lineIntersectsSurfaceInActiveSubmap(const Point& start_point,
                                           const Point& end_point) override;
  bool isOccupiedInActiveSubmap(const Point& position) {
    FloatingPoint esdf_distance = 0.f;
    return getDistanceInActiveSubmap(position, &esdf_distance) &&
           esdf_distance < c_voxel_size_;
  }

  bool getDistanceInActiveSubmap(const Point& position,
                                 FloatingPoint* distance) const override;
  bool getDistanceAndGradientInActiveSubmap(const Point& position,
                                            FloatingPoint* distance,
                                            Point* gradient) const override;

  VoxelState getVoxelStateInLocalArea(const Point& position) override;
  Point getVoxelCenterInLocalArea(const Point& position) const override {
    return (position / c_voxel_size_).array().round() * c_voxel_size_;
  }

  /* Global planner */
  // Since map is monolithic global = local.
  bool isObservedInGlobalMap(const Point& position) override {
    return server_->getEsdfMapPtr()->isObserved(position.cast<double>());
  }
  bool isTraversableInGlobalMap(
      const Point& position,
      const FloatingPoint traversability_radius) override {
    return isTraversableInActiveSubmap(position, traversability_radius);
  }
  bool isLineTraversableInGlobalMap(
      const Point& start_point, const Point& end_point,
      const FloatingPoint traversability_radius,
      Point* last_traversable_point = nullptr) override {
    return isLineTraversableInActiveSubmap(
        start_point, end_point, traversability_radius, last_traversable_point);
  }
  bool lineIntersectsSurfaceInGlobalMap(const Point& start_point,
                                        const Point& end_point) override {
    return lineIntersectsSurfaceInActiveSubmap(start_point, end_point);
  }
  bool getDistanceInGlobalMap(const Point& position,
                              FloatingPoint* distance) override {
    return getDistanceInActiveSubmap(position, distance);
  }

  std::vector<SubmapId> getSubmapIdsAtPosition(
      const Point& position) const override {
    return std::vector<SubmapId>({0u});
  }
  std::vector<SubmapData> getAllSubmapData() override;

 protected:
  const Config config_;
  std::unique_ptr<ThreadsafeVoxbloxServer> server_;

  // cached constants
  FloatingPoint c_block_size_;
  FloatingPoint c_voxel_size_;

  static constexpr FloatingPoint kMaxLineTraversabilityCheckLength = 1e2;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_
