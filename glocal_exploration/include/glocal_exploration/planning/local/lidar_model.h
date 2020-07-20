#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_LIDAR_MODEL_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_LIDAR_MODEL_H_

#include <memory>
#include <vector>

#include "glocal_exploration/planning/local/sensor_model.h"

namespace glocal_exploration {

class LidarModel : public SensorModel {
 public:
  struct Config : SensorModel::Config {
    double ray_length = 5.0;   // m
    double vertical_fov = 45;  // Total fields of view [deg], expected symmetric
                               // w.r.t. sensor facing direction
    double horizontal_fov = 360;
    int vertical_resolution = 64;
    int horizontal_resolution = 1024;
    double ray_step = 0;  // m, use 0 to use voxel size
    double downsampling_factor =
        1.0;  // reduce the number of checks by this factor
  };

  explicit LidarModel(std::shared_ptr<MapBase> map,
                      std::shared_ptr<StateMachine> state_machine);
  virtual ~LidarModel() = default;

  bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                        const WayPoint& waypoint) override;
  bool setupFromConfig(SensorModel::Config* config) override;

 protected:
  Config config_;

  // constants
  int c_res_x_;  // factual resolution that is used for ray casting
  int c_res_y_;
  double c_fov_x_;  // fov in rad
  double c_fov_y_;
  int c_n_sections_;  // number of ray duplications
  std::vector<double>
      c_split_distances_;            // distances where rays are duplicated
  std::vector<int> c_split_widths_;  // number of max distance rays that are
                                     // covered per split
  Eigen::Vector3d mounting_position_;
  Eigen::Quaterniond mounting_orientation_;

  // variables
  Eigen::ArrayXXi ray_table_;

  // methods
  void markNeighboringRays(int x, int y, int segment, int value);
  void getDirectionVector(Eigen::Vector3d* result, double relative_x,
                          double relative_y) const;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_LIDAR_MODEL_H_
