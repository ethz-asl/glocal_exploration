#include "glocal_exploration_ros/conversions/ros_params.h"

namespace glocal_exploration {

VoxbloxMap::Config getVoxbloxMapConfigFromRos(const ros::NodeHandle& nh) {
  VoxbloxMap::Config config;
  config.nh_private_namespace = nh.getNamespace();
  nh.param("traversability_radius", config.traversability_radius,
           config.traversability_radius);
  nh.param("clearing_radius", config.clearing_radius, config.clearing_radius);
  return config;
}

LidarModel::Config getLidarModelConfigFromRos(const ros::NodeHandle& nh) {
  LidarModel::Config config;
  nh.param("ray_length", config.ray_length, config.ray_length);
  nh.param("vertical_fov", config.vertical_fov, config.vertical_fov);
  nh.param("horizontal_fov", config.horizontal_fov, config.horizontal_fov);
  nh.param("vertical_resolution", config.vertical_resolution,
           config.vertical_resolution);
  nh.param("horizontal_resolution", config.horizontal_resolution,
           config.horizontal_resolution);
  nh.param("ray_step", config.ray_step, config.ray_step);
  nh.param("downsampling_factor", config.downsampling_factor,
           config.downsampling_factor);

  // sensor model base
  nh.param("mounting_position_x", config.mounting_position_x,
           config.mounting_position_x);
  nh.param("mounting_position_y", config.mounting_position_y,
           config.mounting_position_y);
  nh.param("mounting_position_z", config.mounting_position_z,
           config.mounting_position_z);
  nh.param("mounting_orientation_x", config.mounting_orientation_x,
           config.mounting_orientation_x);
  nh.param("mounting_orientation_y", config.mounting_orientation_y,
           config.mounting_orientation_y);
  nh.param("mounting_orientation_z", config.mounting_orientation_z,
           config.mounting_orientation_z);
  nh.param("mounting_orientation_w", config.mounting_orientation_w,
           config.mounting_orientation_w);
  return config;
}

RHRRTStar::Config getRHRRTStarConfigFromRos(const ros::NodeHandle& nh) {
  RHRRTStar::Config config;
  nh.param("local_sampling_radius", config.local_sampling_radius,
           config.local_sampling_radius);
  nh.param("global_sampling_radius", config.global_sampling_radius,
           config.global_sampling_radius);
  nh.param("min_local_points", config.min_local_points,
           config.min_local_points);
  nh.param("min_path_length", config.min_path_length, config.min_path_length);
  nh.param("min_sampling_distance", config.min_sampling_distance,
           config.min_sampling_distance);
  nh.param("max_path_length", config.max_path_length, config.max_path_length);
  nh.param("path_cropping_length", config.path_cropping_length,
           config.path_cropping_length);
  nh.param("max_number_of_neighbors", config.max_number_of_neighbors,
           config.max_number_of_neighbors);
  nh.param("maximum_rewiring_iterations", config.maximum_rewiring_iterations,
           config.maximum_rewiring_iterations);
  config.lidar_config = getLidarModelConfigFromRos(nh);
  return config;
}

BoundingBox::Config getBoundingBoxConfigFromRos(const ros::NodeHandle& nh) {
  BoundingBox::Config config;
  nh.param("x_min", config.x_min, config.x_min);
  nh.param("y_min", config.y_min, config.y_min);
  nh.param("z_min", config.z_min, config.z_min);
  nh.param("x_max", config.x_max, config.x_max);
  nh.param("y_max", config.y_max, config.y_max);
  nh.param("z_max", config.z_max, config.z_max);
  return config;
}

}  // namespace glocal_exploration
