#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_

#include <memory>
#include <string>
#include <vector>

#include <glocal_exploration/mapping/map_base.h>
#include <3rd_party/config_utilities.hpp>

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
    double traversability_radius = 0.3;  // m
    double clearing_radius = 0.5;        // m

    Config();
    void checkParams() const override;
    void fromRosParam() override;
  };

  explicit VoxbloxMap(const Config& config,
                      const std::shared_ptr<Communicator>& communicator);
  ~VoxbloxMap() override = default;

  double getVoxelSize() override;
  bool isTraversableInActiveSubmap(const Point& position) override;
  VoxelState getVoxelStateInLocalArea(const Point& position) override;
  Point getVoxelCenterInLocalArea(const Point& position) override;
  bool isObservedInGlobalMap(const Point& position) override;
  bool isTraversableInGlobalMap(const Point& position) override;
  void getAllSubmapData(std::vector<SubmapData>* data) override;

 protected:
  const Config config_;
  std::unique_ptr<ThreadsafeVoxbloxServer> server_;

  // cached constants
  double c_block_size_;
  double c_voxel_size_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_VOXBLOX_MAP_H_
