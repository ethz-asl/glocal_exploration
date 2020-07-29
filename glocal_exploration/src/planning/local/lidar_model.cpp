#include "glocal_exploration/planning/local/lidar_model.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "glocal_exploration/state/communicator.h"
#include "glocal_exploration/utility/config_checker.h"

namespace glocal_exploration {

bool LidarModel::Config::isValid() const {
  ConfigChecker checker("LidarModel");
  checker.check_gt(vertical_fov, 0.0, "vertical_fov");
  checker.check_gt(horizontal_fov, 0.0, "horizontal_fov");
  checker.check_gt(vertical_resolution, 0, "vertical_resolution");
  checker.check_gt(horizontal_resolution, 0, "horizontal_resolution");
  checker.check_gt(ray_length, 0.0, "ray_length");
  checker.check_gt(downsampling_factor, 0.0, "downsampling_factor");
  return checker.isValid();
}

LidarModel::Config LidarModel::Config::checkValid() const {
  CHECK(isValid());
  return Config(*this);
}

LidarModel::LidarModel(const Config& config,
                       std::shared_ptr<Communicator> communicator)
    : SensorModel(std::move(communicator)),
      config_(config.checkValid()),
      kFovX_(config_.horizontal_fov / 180.0 * M_PI),
      kFovY_(config_.vertical_fov / 180.0 * M_PI),
      kResolutionX_(std::min(
          static_cast<int>(std::ceil(
              config_.ray_length * kFovX_ /
              (comm_->map()->getVoxelSize() * config_.downsampling_factor))),
          config_.vertical_resolution)),
      kResolutionY_(std::min(
          static_cast<int>(std::ceil(
              config_.ray_length * kFovY_ /
              (comm_->map()->getVoxelSize() * config_.downsampling_factor))),
          config_.horizontal_resolution)) {
  // Determine number of splits + split distances
  c_n_sections_ = static_cast<int>(
      std::floor(std::log2(std::min(static_cast<double>(kResolutionX_),
                                    static_cast<double>(kResolutionY_)))));

  // Precompute the splits and ray table
  ray_table_ = Eigen::ArrayXXi::Zero(kResolutionX_, kResolutionY_);
  c_split_widths_.push_back(0);
  for (int i = 0; i < c_n_sections_; ++i) {
    c_split_widths_.push_back(std::pow(2, i));
    c_split_distances_.push_back(config_.ray_length /
                                 std::pow(2.0, static_cast<double>(i)));
  }
  c_split_distances_.push_back(0.0);
  std::reverse(c_split_distances_.begin(), c_split_distances_.end());
  std::reverse(c_split_widths_.begin(), c_split_widths_.end());
}

bool LidarModel::getVisibleVoxels(std::vector<Eigen::Vector3d>* centers,
                                  std::vector<MapBase::VoxelState>* states,
                                  const WayPoint& waypoint) {
  // Setup ray table (contains at which segment to start, -1 if occluded)
  ray_table_.setZero();

  // Ray-casting
  Eigen::Quaterniond orientation =
      Eigen::AngleAxisd(waypoint.yaw, Eigen::Vector3d::UnitZ()) *
      config_.T_baselink_sensor.getEigenQuaternion();
  Eigen::Vector3d position =
      waypoint.position() + config_.T_baselink_sensor.getPosition();
  Eigen::Vector3d camera_direction;
  Eigen::Vector3d direction;
  Eigen::Vector3d current_position;
  double distance;
  bool cast_ray;
  for (int i = 0; i < kResolutionX_; ++i) {
    for (int j = 0; j < kResolutionY_; ++j) {
      int current_segment = ray_table_(i, j);  // get ray starting segment
      if (current_segment < 0) {
        continue;  // already occluded ray
      }
      LidarModel::getDirectionVector(
          &camera_direction,
          static_cast<double>(i) / (static_cast<double>(kResolutionX_) - 1.0),
          static_cast<double>(j) / (static_cast<double>(kResolutionY_) - 1.0));
      direction = orientation * camera_direction;
      distance = c_split_distances_[current_segment];
      cast_ray = true;
      while (cast_ray) {
        // iterate through all splits (segments)
        while (distance < c_split_distances_[current_segment + 1]) {
          current_position = position + distance * direction;
          distance += config_.ray_step;

          // Check voxel occupied
          MapBase::VoxelState state =
              comm_->map()->getVoxelStateInLocalArea(current_position);
          if (state == MapBase::VoxelState::kOccupied ||
              !comm_->regionOfInterest()->contains(current_position)) {
            // Occlusion, mark neighboring rays as occluded
            markNeighboringRays(i, j, current_segment, -1);
            cast_ray = false;
            break;
          }

          // add point
          centers->push_back(
              comm_->map()->getVoxelCenterInLocalArea(current_position));
          states->push_back(state);
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
  // TODO(schmluk): Maybe check for duplicates here, double check theory
  return true;
}

void LidarModel::markNeighboringRays(int x, int y, int segment, int value) {
  // Set all nearby (towards bottom right) ray starts, depending on the segment
  // depth, to a value.
  for (int i = x; i < std::min(kResolutionX_, x + c_split_widths_[segment]);
       ++i) {
    for (int j = y; j < std::min(kResolutionY_, y + c_split_widths_[segment]);
         ++j) {
      ray_table_(i, j) = value;
    }
  }
}

void LidarModel::getDirectionVector(Eigen::Vector3d* result, double relative_x,
                                    double relative_y) const {
  double polar_angle = (0.5 - relative_x) * kFovX_;
  double azimuth_angle = M_PI / 2.0 + (relative_y - 0.5) * kFovY_;
  *result = Eigen::Vector3d(sin(azimuth_angle) * cos(polar_angle),
                            sin(azimuth_angle) * sin(polar_angle),
                            cos(azimuth_angle));
}

}  // namespace glocal_exploration
