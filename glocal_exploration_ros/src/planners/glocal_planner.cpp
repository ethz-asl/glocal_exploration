#include "glocal_exploration_ros/planners/glocal_planner.h"

#include "glocal_exploration_ros/conversions/ros_params.h"

namespace glocal_exploration {

GlocalPlanner::GlocalPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {

  // setup the map
  ros::NodeHandle nh_mapping(nh_private_, "mapping");
  std::string map_type;
  nh_mapping.param("map_type", map_type, std::string("unspecified"));
  // TODO(schmluk): could also make this a factory if many map types
  if (map_type == "voxblox") {
    map_ = std::make_unique<VoxbloxMap>();
    VoxbloxMap::Config cfg = getVoxbloxMapConfigFromRos(nh_mapping);
    map_->setupFromConfig(&cfg);
  } else {
    LOG(ERROR) << "Unknown map type '" << map_type << "'.";
  }
}

} // namespace glocal_exploration