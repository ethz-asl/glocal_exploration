#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_LIDAR_MODEL_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_LIDAR_MODEL_H_

#include <memory>
#include <vector>

#include "glocal_exploration/planning/local/sensor_model.h"

namespace glocal_exploration {

class LidarModel : public SensorModel {
 public:
  struct Config {
    double ray_length = 5.0;   // m
    double vertical_fov = 45;  // Total fields of view [deg], expected symmetric
    // w.r.t. sensor facing direction
    double horizontal_fov = 360;
    int vertical_resolution = 64;
    int horizontal_resolution = 1024;
    double ray_step = 0.0;  // m, use 0 to default to voxel size
    double downsampling_factor =
        1.0;  // reduce the number of checks by this factor
    Transformation T_baselink_sensor;

    Config isValid() const;
  };

  explicit LidarModel(const Config& config,
                      std::shared_ptr<Communicator> communicator);
  virtual ~LidarModel() = default;

  bool getVisibleVoxels(std::vector<Eigen::Vector3d>* centers,
                        std::vector<MapBase::VoxelState>* states,
                        const WayPoint& waypoint) override;

 protected:
  const Config config_;

  // cached constants
  const double kFovX_;  // fov in rad
  const double kFovY_;
  const int kResolutionX_;  // factual resolution that is used for ray casting
  const int kResolutionY_;
  int c_n_sections_;  // number of ray duplications
  std::vector<double>
      c_split_distances_;            // distances where rays are duplicated
  std::vector<int> c_split_widths_;  // number of max distance rays that are
  // covered per split

  // variables
  Eigen::ArrayXXi ray_table_;

  // methods
  void markNeighboringRays(int x, int y, int segment, int value);
  // x and y are cylindrical image coordinates scaled to [0, 1]
  void getDirectionVector(Eigen::Vector3d* result, double relative_x,
                          double relative_y) const;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_LIDAR_MODEL_H_
