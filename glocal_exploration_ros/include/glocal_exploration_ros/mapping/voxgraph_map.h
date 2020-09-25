#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_MAP_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_MAP_H_

#include <memory>
#include <string>
#include <vector>

#include <glocal_exploration/mapping/map_base.h>
#include <glocal_exploration/3rd_party/config_utilities.hpp>

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
    double traversability_radius = 0.3;  // m
    double clearing_radius = 0.5;        // m
    int verbosity = 1;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  explicit VoxgraphMap(const Config& config,
                       const std::shared_ptr<Communicator>& communicator);
  ~VoxgraphMap() override = default;

  // MapBase overrides.
  double getVoxelSize() override { return c_voxel_size_; }
  std::vector<SubmapData> getAllSubmapData() override;

  bool isTraversableInActiveSubmap(const Point& position) override;
  bool isLineTraversableInActiveSubmap(
      const Point& start_point, const Point& end_point,
      Point* last_traversable_point = nullptr) override;
  bool getDistanceAndGradientAtPositionInActiveSubmap(const Point& position,
                                                      double* distance,
                                                      Point* gradient) override;

  Point getVoxelCenterInLocalArea(const Point& point) override;
  VoxelState getVoxelStateInLocalArea(const Point& position) override;

  bool isObservedInGlobalMap(const Point& position) override;
  bool isTraversableInGlobalMap(const Point& position) override;
  bool isLineTraversableInGlobalMap(
      const Point& start_point, const Point& end_point,
      Point* last_traversable_point = nullptr) override;
  bool getDistanceInGlobalMapAtPosition(const Point& position,
                                        double* min_esdf_distance);

  std::vector<voxgraph::SubmapID> getSubmapsAtPosition(
      const Point& position) const {
    return voxgraph_spatial_hash_.getSubmapsAtPosition(position);
  }

 protected:
  const Config config_;

  std::unique_ptr<ThreadsafeVoxbloxServer> voxblox_server_;
  std::unique_ptr<ThreadsafeVoxgraphServer> voxgraph_server_;

  std::unique_ptr<VoxgraphLocalArea> local_area_;
  bool local_area_needs_update_;
  void updateLocalAreaIfNeeded();
  static constexpr double local_area_pruning_period_s_ = 10.0;
  ros::Timer local_area_pruning_timer_;
  ros::Publisher local_area_pub_;

  VoxgraphSpatialHash voxgraph_spatial_hash_;
  ros::Publisher voxgraph_spatial_hash_pub_;

  // cached constants
  double c_block_size_;
  double c_voxel_size_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXGRAPH_MAP_H_
