#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_MAP_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_MAP_H_

#include <memory>
#include <string>
#include <vector>

#include <glocal_exploration/3rd_party/config_utilities.hpp>
#include <glocal_exploration/mapping/map_base.h>

#include "glocal_exploration_ros/mapping/threadsafe_wrappers/threadsafe_voxblox_server.h"
#include "glocal_exploration_ros/mapping/threadsafe_wrappers/threadsafe_voxgraph_server.h"
#include "glocal_exploration_ros/mapping/voxgraph_local_area.h"
#include "glocal_exploration_ros/mapping/voxgraph_spatial_hash.h"

namespace glocal_exploration {
/**
 * Map class that just uses voxgraph as a monolithic map baseline.
 */
class VoxgraphMap : public MapBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    // Since this is a ros-class anyways we make it easy and just get the nh.
    std::string nh_private_namespace = "~";
    FloatingPoint traversability_radius = 0.3f;  // m
    FloatingPoint clearing_radius = 0.5f;        // m
    int verbosity = 1;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  explicit VoxgraphMap(const Config& config,
                       const std::shared_ptr<Communicator>& communicator);
  ~VoxgraphMap() override = default;

  /* General and Accessors */
  FloatingPoint getVoxelSize() const override { return c_voxel_size_; }
  FloatingPoint getTraversabilityRadius() const override {
    return config_.traversability_radius;
  }
  std::vector<WayPoint> getPoseHistory() const override;

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

  Point getVoxelCenterInLocalArea(const Point& position) const override {
    return (position / c_voxel_size_).array().round() * c_voxel_size_;
  }
  VoxelState getVoxelStateInLocalArea(const Point& position) override;

  /* Global planner */
  bool isObservedInGlobalMap(const Point& position) override;
  bool isTraversableInGlobalMap(
      const Point& position,
      const FloatingPoint traversability_radius) override;
  bool isLineTraversableInGlobalMap(
      const Point& start_point, const Point& end_point,
      const FloatingPoint traversability_radius,
      Point* last_traversable_point = nullptr) override;
  bool lineIntersectsSurfaceInGlobalMap(const Point& start_point,
                                        const Point& end_point) override;
  bool isOccupiedInGlobalMap(const Point& position) {
    FloatingPoint esdf_distance = 0.f;
    return getDistanceInGlobalMap(position, &esdf_distance) &&
           esdf_distance < c_voxel_size_;
  }

  bool getDistanceInGlobalMap(const Point& position,
                              FloatingPoint* min_esdf_distance);

  std::vector<voxgraph::SubmapID> getSubmapIdsAtPosition(
      const Point& position) const override {
    return voxgraph_spatial_hash_.getSubmapsAtPosition(position);
  }
  std::vector<SubmapData> getAllSubmapData() override;

 protected:
  const Config config_;

  std::unique_ptr<ThreadsafeVoxbloxServer> voxblox_server_;
  std::unique_ptr<ThreadsafeVoxgraphServer> voxgraph_server_;

  std::unique_ptr<VoxgraphLocalArea> local_area_;
  bool local_area_needs_update_;
  void updateLocalAreaIfNeeded();
  static constexpr FloatingPoint local_area_pruning_period_s_ = 10.f;
  ros::Timer local_area_pruning_timer_;
  ros::Publisher local_area_pub_;

  VoxgraphSpatialHash voxgraph_spatial_hash_;
  ros::Publisher voxgraph_spatial_hash_pub_;

  // cached constants
  FloatingPoint c_block_size_;
  FloatingPoint c_voxel_size_;

  static constexpr FloatingPoint kMaxLineTraversabilityCheckLength = 1e2;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_MAP_H_
