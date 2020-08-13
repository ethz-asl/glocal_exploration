#include "glocal_exploration_ros/visualization/skeleton_visualizer.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

namespace glocal_exploration {

bool SkeletonVisualizer::Config::isValid() const { return true; }

SkeletonVisualizer::Config SkeletonVisualizer::Config::checkValid() const {
  CHECK(isValid());
  return Config(*this);
}

SkeletonVisualizer::SkeletonVisualizer(
    const Config& config, const std::shared_ptr<Communicator>& communicator)
    : config_(config.checkValid()), GlobalPlannerVisualizerBase(communicator) {
  // Reference planner.
  planner_ = std::dynamic_pointer_cast<SkeletonPlanner>(comm_->globalPlanner());
  if (!planner_) {
    LOG(FATAL) << "Can not setup 'SkeletonVisualizer' with a global planner "
                  "that is not of type 'SkeletonPlanner'.";
  }

  // Colors.
  if (config_.n_frontier_colors <= 1) {
    color_list_ = {voxblox::Color::Green()};
  } else {
    color_list_.reserve(config_.n_frontier_colors);
    for (int i = 0; i < config_.n_frontier_colors; ++i) {
      float h = static_cast<float>(i) /
                static_cast<float>(config_.n_frontier_colors - 1);
      color_list_.emplace_back(voxblox::rainbowColorMap(h));
      std::random_device random_device;
      std::shuffle(color_list_.begin(), color_list_.end(),
                   std::mt19937(random_device()));
    }
  }

  // ROS
  nh_ = ros::NodeHandle(config_.nh_namespace);
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization", 100);
}

void SkeletonVisualizer::visualize() {
  if (config_.visualize_frontiers) {
    // Note(schmluk): If submaps are frozen the id_tracker will ensure that
    // frontier colors persist. Otherwise the colors will be nicely spaced.
    unsigned int id_tracker = 0;
    std::vector<int> ids;
    ids.reserve(planner_->getFrontiers().size());
    for (const auto& id_frontiers_pair : planner_->getFrontiers()) {
      ids.push_back(id_frontiers_pair.first);
    }
    std::sort(ids.begin(), ids.end());

    // Erase previous visualizations
    auto msg = visualization_msgs::Marker();
    auto msg_array = visualization_msgs::MarkerArray();
    msg.header.frame_id = "mission";
    msg.header.stamp = ros::Time::now();
    msg.ns = frontier_ns;
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg_array.markers.push_back(msg);
    pub_.publish(msg_array);

    // Visualize all frontiers in all submaps.
    frontier_msg_id_ = 0;  // Tracker for each voxels id.
    for (const int& id : ids) {
      const FrontierCollection& frontiers = planner_->getFrontiers().at(id);
      for (const Frontier& frontier : frontiers) {
        pub_.publish(visualizeFrontier(frontier, &id_tracker));
      }
    }
  }
}

visualization_msgs::MarkerArray SkeletonVisualizer::visualizeFrontier(
    const Frontier& frontier, unsigned int* id) {
  auto result = visualization_msgs::MarkerArray();
  if (!frontier.isActive() && !config_.visualize_inactive_frontiers) {
    return result;
  }
  voxblox::Color color = color_list_[*id % color_list_.size()];
  for (const FrontierCandidate& point : frontier) {
    if (!point.is_active && !config_.visualize_inactive_frontiers) {
      continue;
    }
    auto msg = visualization_msgs::Marker();
    msg.header.frame_id = "mission";
    msg.header.stamp = ros::Time::now();
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.type = visualization_msgs::Marker::CUBE;
    msg.ns = frontier_ns;
    msg.id = frontier_msg_id_++;
    double scale = comm_->map()->getVoxelSize();
    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;
    msg.pose.position.x = point.position.x();
    msg.pose.position.y = point.position.y();
    msg.pose.position.z = point.position.z();
    msg.color.r = color.r;
    msg.color.g = color.g;
    msg.color.b = color.b;
    if (point.is_active) {
      msg.color.a = 1.0;
    } else {
      msg.color.a = 0.1;
    }
    result.markers.push_back(msg);
  }
  (*id)++;
  return result;
}

}  // namespace glocal_exploration
