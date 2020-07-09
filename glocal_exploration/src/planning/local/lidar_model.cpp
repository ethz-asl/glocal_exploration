#include "glocal_exploration/planning/local/lidar_model.h"

namespace glocal_exploration {

LidarModel::LidarModel(std::shared_ptr<MapBase> map,
                       std::shared_ptr<StateMachine> state_machine)
    : SensorModel(std::move(map), std::move(state_machine)) {}

bool LidarModel::setupFromConfig(SensorModel::Config* config) {
  CHECK_NOTNULL(config);
  auto cfg = dynamic_cast<Config*>(config);
  if (!cfg) {
    LOG(ERROR)
        << "Failed to setup: config is not of type 'LidarModel::Config'.";
    return false;
  }
  config_ = *cfg;

  // params
  CHECK_GT(config_.vertical_fov, 0)
      << "The vertical field of view is expected > 0";
  CHECK_GT(config_.horizontal_fov, 0)
      << "The horizontal field of view is expected > 0";
  CHECK_GT(config_.vertical_resolution, 0)
      << "The vertical resolution is expected > 0";
  CHECK_GT(config_.horizontal_resolution, 0)
      << "The horizontal resolution is expected > 0";
  CHECK_GT(config_.ray_length, 0) << "The ray length is expected > 0";
  if (config_.ray_step <= 0.0) {
    config_.ray_step = map_->getVoxelSize();
  }

  // Downsample to voxel size resolution at max range
  c_fov_x_ = config_.horizontal_fov / 180.0 * M_PI;
  c_fov_y_ = config_.vertical_fov / 180.0 * M_PI;
  c_res_x_ =
      std::min((int)ceil(config_.ray_length * c_fov_x_ /
                         (map_->getVoxelSize() * config_.downsampling_factor)),
               config_.vertical_resolution);
  c_res_y_ =
      std::min((int)ceil(config_.ray_length * c_fov_y_ /
                         (map_->getVoxelSize() * config_.downsampling_factor)),
               config_.horizontal_resolution);
  ray_table_ = Eigen::ArrayXXi::Zero(c_res_x_, c_res_y_);
  mounting_position_ =
      Eigen::Vector3d(config_.mounting_position_x, config_.mounting_position_y,
                      config_.mounting_position_z);
  mounting_orientation_ = Eigen::Quaterniond(
      config_.mounting_orientation_w, config_.mounting_orientation_x,
      config_.mounting_orientation_y, config_.mounting_orientation_z);

  // Determine number of splits + split distances
  c_n_sections_ =
      (int)std::floor(std::log2(std::min((double)c_res_x_, (double)c_res_y_)));
  c_split_widths_.push_back(0);
  for (int i = 0; i < c_n_sections_; ++i) {
    c_split_widths_.push_back(std::pow(2, i));
    c_split_distances_.push_back(config_.ray_length / std::pow(2.0, (double)i));
  }
  c_split_distances_.push_back(0.0);
  std::reverse(c_split_distances_.begin(), c_split_distances_.end());
  std::reverse(c_split_widths_.begin(), c_split_widths_.end());
  return true;
}

bool LidarModel::getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                                  const WayPoint& waypoint) {
  // Setup ray table (contains at which segment to start, -1 if occluded
  ray_table_.setZero();

  // Ray-casting
  Eigen::Quaterniond orientation =
      Eigen::AngleAxisd(waypoint.yaw, Eigen::Vector3d::UnitZ()) *
      mounting_orientation_;
  Eigen::Vector3d position = waypoint.position() + mounting_position_;
  Eigen::Vector3d camera_direction;
  Eigen::Vector3d direction;
  Eigen::Vector3d current_position;
  double distance;
  bool cast_ray;
  for (int i = 0; i < c_res_x_; ++i) {
    for (int j = 0; j < c_res_y_; ++j) {
      int current_segment = ray_table_(i, j);  // get ray starting segment
      if (current_segment < 0) {
        continue;  // already occluded ray
      }
      LidarModel::getDirectionVector(&camera_direction,
                                     (double)i / ((double)c_res_x_ - 1.0),
                                     (double)j / ((double)c_res_y_ - 1.0));
      direction = orientation * camera_direction;
      distance = c_split_distances_[current_segment];
      cast_ray = true;
      while (cast_ray) {
        // iterate through all splits (segments)
        while (distance < c_split_distances_[current_segment + 1]) {
          current_position = position + distance * direction;
          distance += config_.ray_step;

          // Check voxel occupied
          if (map_->getVoxelStateInLocalArea(current_position) ==
                  MapBase::Occupied ||
              !state_machine_->pointInROI(current_position)) {
            // Occlusion, mark neighboring rays as occluded
            markNeighboringRays(i, j, current_segment, -1);
            cast_ray = false;
            break;
          }

          // add point
          result->push_back(map_->getVoxelCenterInLocalArea(current_position));
        }
        if (cast_ray) {
          current_segment++;
          if (current_segment >= c_n_sections_) {
            cast_ray = false;  // done
          } else {
            // update ray starts of neighboring rays
            markNeighboringRays(i, j, current_segment - 1, current_segment);
          }
        }
      }
    }
  }
  // remove duplicates (faster than looking these up during ray casting)
  result->erase(std::unique(result->begin(), result->end()), result->end());
  return true;
}

void LidarModel::markNeighboringRays(int x, int y, int segment, int value) {
  // Set all nearby (towards bottom right) ray starts, depending on the segment
  // depth, to a value.
  for (int i = x; i < std::min(c_res_x_, x + c_split_widths_[segment]); ++i) {
    for (int j = y; j < std::min(c_res_y_, y + c_split_widths_[segment]); ++j) {
      ray_table_(i, j) = value;
    }
  }
}

void LidarModel::getDirectionVector(Eigen::Vector3d* result, double relative_x,
                                    double relative_y) const {
  double phi = (0.5 - relative_x) * c_fov_x_;
  double theta = M_PI / 2.0 + (relative_y - 0.5) * c_fov_y_;
  *result =
      Eigen::Vector3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

}  // namespace glocal_exploration
