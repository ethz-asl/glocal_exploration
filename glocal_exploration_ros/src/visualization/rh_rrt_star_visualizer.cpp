#include "glocal_exploration_ros/visualization/rh_rrt_star_visualizer.h"

#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>

namespace glocal_exploration {

RHRRTStarVisualizer::RHRRTStarVisualizer(const ros::NodeHandle &nh, const std::shared_ptr<LocalPlannerBase> &planner)
    : LocalPlannerVisualizerBase(nh, planner),
      num_previous_msgs_(0),
      visualize_gain_(true),
      visualize_value_(true) {
  planner_ = std::dynamic_pointer_cast<RHRRTStar>(planner);
  if (!planner) {
    LOG(FATAL) << "Can not setup 'RHRRTStarVisualizer' with planner that is not of type 'RHRRTStar'";
  }

  // params
  nh.param("visualize_gain", visualize_gain_, visualize_gain_);
  nh.param("visualize_value", visualize_value_, visualize_value_);

  // ROS
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization", 100);
}

void RHRRTStarVisualizer::visualize() {
  if (!visualize_gain_ && !visualize_value_) { return; }
  auto &points = planner_->getTreeData().points;
  ros::Time now = ros::Time::now();
  std::string frame_id = "world";
  // Display all trajectories in the input and erase previous ones
  double max_value = points[0]->value;
  double min_value = points[0]->value;
  double max_gain = points[0]->gain;
  double min_gain = points[0]->gain;
  for (size_t i = 1; i < points.size(); ++i) {
    if (points[i]->value >= max_value) {
      max_value = points[i]->value;
    }
    if (points[i]->value < min_value) {
      min_value = points[i]->value;
    }
    if (points[i]->gain > max_gain) {
      max_gain = points[i]->gain;
    }
    if (points[i]->gain < min_gain) {
      min_gain = points[i]->gain;
    }
  }

  visualization_msgs::MarkerArray value_markers, gain_markers, goal_markers;
  visualization_msgs::Marker msg;
  for (int i = 0; i < points.size(); ++i) {
    // visualize value
    if (visualize_value_) {
      // Setup marker message
      msg = visualization_msgs::Marker();
      msg.header.frame_id = frame_id;
      msg.header.stamp = now;
      msg.pose.orientation.w = 1.0;
      msg.type = visualization_msgs::Marker::LINE_STRIP;
      msg.id = i;
      msg.ns = "candidate_trajectories";
      msg.scale.x = 0.08;
      msg.color.a = 1;
      msg.action = visualization_msgs::Marker::ADD;

      if (!points[i]->is_root) {
        // Color according to relative value (blue when indifferent)
        if (max_value != min_value) {
          double frac = (points[i]->value - min_value) / (max_value - min_value);
          msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
          msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
          msg.color.b = 0.0;
        } else {
          msg.color.r = 0.3;
          msg.color.g = 0.3;
          msg.color.b = 1.0;
        }

        // points
        geometry_msgs::Point pt;
        const Eigen::Vector3d &start = points[i]->pose.position();
        pt.x = start.x();
        pt.y = start.y();
        pt.z = start.z();
        msg.points.push_back(pt);
        const Eigen::Vector3d &end = points[i]->getConnectedViewPoint(points[i]->active_connection)->pose.position();
        pt.x = end.x();
        pt.y = end.y();
        pt.z = end.z();
        msg.points.push_back(pt);
      } else {
        msg.points.push_back(geometry_msgs::Point()); // suppress rviz warnings
        msg.points.push_back(geometry_msgs::Point());
      }
      value_markers.markers.push_back(msg);
    }

    // visualize gain
    if (visualize_gain_) {
      msg = visualization_msgs::Marker();
      msg.header.frame_id = frame_id;
      msg.header.stamp = now;
      msg.type = visualization_msgs::Marker::ARROW;
      msg.action = visualization_msgs::Marker::ADD;
      msg.id = i;
      msg.ns = "canidate_gains";
      msg.scale.x = 0.2;
      msg.scale.y = 0.1;
      msg.scale.z = 0.1;
      msg.pose.position.x = points[i]->pose.x;
      msg.pose.position.y = points[i]->pose.y;
      msg.pose.position.z = points[i]->pose.z;
      tf2::Quaternion q;
      q.setRPY(0, 0, points[i]->pose.yaw);
      msg.pose.orientation.w = q.w();
      msg.pose.orientation.x = q.x();
      msg.pose.orientation.y = q.y();
      msg.pose.orientation.z = q.z();

      // Color according to relative value (blue when indifferent)
      if (min_gain != max_gain) {
        double frac = (points[i]->gain - min_gain) / (max_gain - min_gain);
        msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
        msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
        msg.color.b = 0.0;
      } else {
        msg.color.r = 0.3;
        msg.color.g = 0.3;
        msg.color.b = 1.0;
      }
      msg.color.a = 1.0;
      gain_markers.markers.push_back(msg);
    }
  }

  // clear previous vis
  for (size_t i = points.size(); i < num_previous_msgs_; ++i) {
    msg = visualization_msgs::Marker();
    msg.id = i;
    msg.ns = "candidate_trajectories";
    msg.color.a = 0.0;
    msg.action = visualization_msgs::Marker::ADD;
    msg.header.frame_id = frame_id;
    msg.header.stamp = now;
    value_markers.markers.push_back(msg);

    if (visualize_gain_) {
      msg = visualization_msgs::Marker();
      msg.action = visualization_msgs::Marker::ADD;
      msg.header.frame_id = frame_id;
      msg.header.stamp = now;
      msg.id = i;
      msg.ns = "canidate_gains";
      msg.color.a = 0.0;
      gain_markers.markers.push_back(msg);
    }
  }
  num_previous_msgs_ = points.size();

  // visualize
  pub_.publish(value_markers);
  pub_.publish(gain_markers);
}

} // namespace glocal_exploration