#include "glocal_exploration_ros/planners/glocal_planner.h"

namespace glocal_exploration {

GlocalPlanner::GlocalPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {};

} // namespace glocal_exploration