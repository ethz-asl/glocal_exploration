#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_LIDAR_MODEL_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_LIDAR_MODEL_H_

#include <memory>
#include <unordered_set>
#include <vector>

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>

#include "glocal_exploration/3rd_party/config_utilities.hpp"
#include "glocal_exploration/planning/local/sensor_model.h"

namespace glocal_exploration {

class LidarModel : public SensorModel {
 public:
  struct Config : public config_utilities::Config<Config> {
    FloatingPoint ray_length = 5.f;  // m
    // Total fields of view [deg], expected symmetric
    FloatingPoint vertical_fov = 45.f;
    // w.r.t. sensor facing direction
    FloatingPoint horizontal_fov = 360.f;
    int vertical_resolution = 64;
    int horizontal_resolution = 1024;
    FloatingPoint ray_step = 0.1f;  // m
    int num_yaw_samples = 4;
    // reduce the number of checks by this factor
    FloatingPoint downsampling_factor = 1.f;
    Transformation T_baselink_sensor;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  explicit LidarModel(const Config& config,
                      std::shared_ptr<Communicator> communicator);
  ~LidarModel() override = default;

  void getVisibleUnknownVoxels(const WayPoint& waypoint,
                               voxblox::LongIndexSet* voxels) override;
  void getVisibleUnknownVoxelsAndOptimalYaw(
      WayPoint* waypoint, voxblox::LongIndexSet* voxels) override;

 protected:
  const Config config_;

  // cached constants
  const FloatingPoint kFovX_;  // fov in rad
  const FloatingPoint kFovY_;
  const int kResolutionX_;  // factual resolution that is used for ray casting
  const int kResolutionY_;
  int c_n_sections_;  // number of ray duplications
  std::vector<FloatingPoint>
      c_split_distances_;            // distances where rays are duplicated
  std::vector<int> c_split_widths_;  // number of max distance rays that are
  // covered per split
  FloatingPoint c_voxel_size_inv_;

  // variables
  Eigen::ArrayXXi ray_table_;

  // methods
  void markNeighboringRays(int x, int y, int segment, int value);
  // x and y are cylindrical image coordinates scaled to [0, 1]
  void getDirectionVector(Point* result, FloatingPoint relative_x,
                          FloatingPoint relative_y) const;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_LIDAR_MODEL_H_
