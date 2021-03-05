#include "glocal_exploration/planning/local/lidar_model.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>

#include "glocal_exploration/state/communicator.h"

namespace glocal_exploration {

LidarModel::Config::Config() { setConfigName("LidarModel"); }

void LidarModel::Config::checkParams() const {
  checkParamGT(vertical_fov, 0.f, "vertical_fov");
  checkParamGT(horizontal_fov, 0.f, "horizontal_fov");
  checkParamGT(vertical_resolution, 0, "vertical_resolution");
  checkParamGT(horizontal_resolution, 0, "horizontal_resolution");
  checkParamGT(ray_length, 0.f, "ray_length");
  checkParamGT(ray_step, 0.f, "ray_step");
  checkParamGT(num_yaw_samples, 0, "num_yaw_samples");
  checkParamGT(downsampling_factor, 0.f, "downsampling_factor");
}

void LidarModel::Config::fromRosParam() {
  rosParam("vertical_fov", &vertical_fov);
  rosParam("horizontal_fov", &horizontal_fov);
  rosParam("vertical_resolution", &vertical_resolution);
  rosParam("horizontal_resolution", &horizontal_resolution);
  rosParam("ray_length", &ray_length);
  rosParam("ray_step", &ray_step);
  rosParam("num_yaw_samples", &num_yaw_samples);
  rosParam("downsampling_factor", &downsampling_factor);
  rosParam("T_baselink_sensor", &T_baselink_sensor);
}

void LidarModel::Config::printFields() const {
  printField("vertical_fov", vertical_fov);
  printField("horizontal_fov", horizontal_fov);
  printField("vertical_resolution", vertical_resolution);
  printField("horizontal_resolution", horizontal_resolution);
  printField("ray_length", ray_length);
  printField("ray_step", ray_step);
  printField("num_yaw_samples", num_yaw_samples);
  printField("downsampling_factor", downsampling_factor);
  printField("T_baselink_sensor", T_baselink_sensor);
}

LidarModel::LidarModel(const Config& config,
                       std::shared_ptr<Communicator> communicator)
    : SensorModel(std::move(communicator)),
      config_(config.checkValid()),
      kFovX_(config_.horizontal_fov / 180.f * M_PI),
      kFovY_(config_.vertical_fov / 180.f * M_PI),
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
  c_n_sections_ = static_cast<int>(std::floor(
      std::log2(std::min(static_cast<FloatingPoint>(kResolutionX_),
                         static_cast<FloatingPoint>(kResolutionY_)))));

  // Precompute the splits and ray table
  ray_table_ = Eigen::ArrayXXi::Zero(kResolutionX_, kResolutionY_);
  c_split_widths_.push_back(0);
  for (int i = 0; i < c_n_sections_; ++i) {
    c_split_widths_.push_back(std::pow(2, i));
    c_split_distances_.push_back(config_.ray_length /
                                 std::pow(2.f, static_cast<FloatingPoint>(i)));
  }
  c_split_distances_.push_back(0.f);
  std::reverse(c_split_distances_.begin(), c_split_distances_.end());
  std::reverse(c_split_widths_.begin(), c_split_widths_.end());
  c_voxel_size_inv_ = 1.f / comm_->map()->getVoxelSize();
}

void LidarModel::getVisibleUnknownVoxels(const WayPoint& waypoint,
                                         voxblox::LongIndexSet* voxels) {
  // NOTE(schmluk): This is a slightly more specialized version for gain
  // computation that is still independent of the map representation.

  // Setup ray table (contains at which segment to start, -1 if occluded)
  ray_table_.setZero();

  // Ray-casting
  Eigen::Quaternionf orientation =
      Eigen::AngleAxisf(waypoint.yaw, Point::UnitZ()) *
      config_.T_baselink_sensor.getEigenQuaternion();
  Point position = waypoint.position + config_.T_baselink_sensor.getPosition();
  Point camera_direction;
  Point direction;
  Point current_position;
  FloatingPoint distance;
  bool cast_ray;
  for (int i = 0; i < kResolutionX_; ++i) {
    for (int j = 0; j < kResolutionY_; ++j) {
      int current_segment = ray_table_(i, j);  // get ray starting segment
      if (current_segment < 0) {
        continue;  // already occluded ray
      }
      LidarModel::getDirectionVector(
          &camera_direction,
          static_cast<FloatingPoint>(i) /
              (static_cast<FloatingPoint>(kResolutionX_) - 1.f),
          static_cast<FloatingPoint>(j) /
              (static_cast<FloatingPoint>(kResolutionY_) - 1.f));
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
          } else if (state == MapBase::VoxelState::kUnknown) {
            // This should handle duplicates.
            auto idx = voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(
                (current_position), c_voxel_size_inv_);
            voxels->insert(idx);
          }
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
}

void LidarModel::getVisibleUnknownVoxelsAndOptimalYaw(
    WayPoint* waypoint, voxblox::LongIndexSet* voxels) {
  CHECK_NOTNULL(waypoint);
  CHECK_NOTNULL(voxels);

  FloatingPoint yaw_sample = waypoint->yaw;
  for (int yaw_sample_i = 0; yaw_sample_i < config_.num_yaw_samples;
       ++yaw_sample_i) {
    yaw_sample += 2.f * M_PI / config_.num_yaw_samples;
    const WayPoint waypoint_sample(waypoint->position, yaw_sample);

    voxblox::LongIndexSet visible_voxels;
    getVisibleUnknownVoxels(waypoint_sample, &visible_voxels);
    if (voxels->size() < visible_voxels.size()) {
      *waypoint = waypoint_sample;
      *voxels = visible_voxels;
    }
  }
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

void LidarModel::getDirectionVector(Point* result, FloatingPoint relative_x,
                                    FloatingPoint relative_y) const {
  FloatingPoint polar_angle = (0.5 - relative_x) * kFovX_;
  FloatingPoint azimuth_angle = M_PI / 2.f + (relative_y - 0.5) * kFovY_;
  *result = Point(sin(azimuth_angle) * cos(polar_angle),
                  sin(azimuth_angle) * sin(polar_angle), cos(azimuth_angle));
}

}  // namespace glocal_exploration
