#include "glocal_exploration_ros/visualization/skeleton_visualizer.h"

#include <memory>
#include <utility>

#include <visualization_msgs/MarkerArray.h>

namespace glocal_exploration {

bool SkeletonVisualizer::Config::isValid() const {}

SkeletonVisualizer::Config SkeletonVisualizer::Config::checkValid() const {}

SkeletonVisualizer::SkeletonVisualizer(
    const Config& config, std::shared_ptr<Communicator> communicator)
    : config_(config.checkValid()),
      GlobalPlannerVisualizerBase(std::move(communicator)) {
  // reference planner
  planner_ = std::dynamic_pointer_cast<SkeletonPlanner>(comm_->localPlanner());
  if (!planner_) {
    LOG(FATAL) << "Can not setup 'SkeletonVisualizer' with a local planner "
                  "that is not of type 'SkeletonPlanner'.";
  }

  // ROS
  nh_ = ros::NodeHandle(config_.nh_namespace);
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization", 100);
}

void SkeletonVisualizer::visualize() {}

}  // namespace glocal_exploration
