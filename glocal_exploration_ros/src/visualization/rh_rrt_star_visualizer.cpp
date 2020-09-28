#include "glocal_exploration_ros/visualization/rh_rrt_star_visualizer.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>

namespace glocal_exploration {

RHRRTStarVisualizer::Config::Config() { setConfigName("RHRRTStarVisualizer"); }

void RHRRTStarVisualizer::Config::checkParams() const {}

void RHRRTStarVisualizer::Config::fromRosParam() {
  rosParam("visualize_gain", &visualize_gain);
  rosParam("visualize_text", &visualize_text);
  rosParam("visualize_visible_voxels", &visualize_visible_voxels);
  rosParam("visualize_tree", &visualize_tree);
  rosParam("visualize_executed_path", &visualize_executed_path);
  nh_namespace = rosParamNameSpace();
}

RHRRTStarVisualizer::RHRRTStarVisualizer(
    const Config& config, const std::shared_ptr<Communicator>& communicator)
    : LocalPlannerVisualizerBase(communicator), config_(config.checkValid()) {
  // reference planner
  planner_ = std::dynamic_pointer_cast<RHRRTStar>(comm_->localPlanner());
  if (!planner_) {
    LOG(FATAL) << "Can not setup 'RHRRTStarVisualizer' with a local planner "
                  "that is not of type 'RHRRTStar'.";
  }

  // ROS
  nh_ = ros::NodeHandle(config_.nh_namespace);
  value_pub_ = nh_.advertise<visualization_msgs::Marker>("tree", 100);
  gain_pub_ = nh_.advertise<visualization_msgs::Marker>("gain", 100);
  text_pub_ = nh_.advertise<visualization_msgs::Marker>("text", 100);
  voxel_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("visible_voxels", 100);
  path_pub_ = nh_.advertise<visualization_msgs::Marker>("executed_path", 100);
}

void RHRRTStarVisualizer::visualize() {
  // Visualize only if a new waypoint was requested.
  if (!comm_->newWayPointIsRequested() &&
      comm_->stateMachine()->currentState() ==
          StateMachine::State::kLocalPlanning) {
    return;
  }

  // Executed path, local planning is in dark blue.
  if (config_.visualize_executed_path) {
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = timestamp_;
    msg.pose.orientation.w = 1.0;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.id = executed_path_id_++;
    msg.scale.x = 0.08;
    msg.color.a = 1;
    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 0.8;
    msg.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point pt;
    tf::pointEigenToMsg(comm_->getPreviousWayPoint().position.cast<double>(),
                        pt);
    msg.points.push_back(pt);
    tf::pointEigenToMsg(comm_->getRequestedWayPoint().position.cast<double>(),
                        pt);
    msg.points.push_back(pt);
    path_pub_.publish(msg);
  }

  // initialize data
  auto& points = planner_->getTreeData().points;

  // cached headers for all msgs
  timestamp_ = ros::Time::now();

  // Find min and max gain and value and other required data.
  FloatingPoint max_value = std::numeric_limits<FloatingPoint>::min();
  FloatingPoint min_value = std::numeric_limits<FloatingPoint>::max();
  FloatingPoint max_gain = std::numeric_limits<FloatingPoint>::min();
  FloatingPoint min_gain = std::numeric_limits<FloatingPoint>::max();
  for (const auto& point : points) {
    if (point->value >= max_value) {
      max_value = point->value;
    }
    if (point->value < min_value) {
      min_value = point->value;
    }
    if (point->gain > max_gain) {
      max_gain = point->gain;
    }
    if (point->gain < min_gain) {
      min_gain = point->gain;
    }
  }

  // Visualizations.
  if (config_.visualize_tree && value_pub_.getNumSubscribers() > 0) {
    auto msg = visualization_msgs::Marker();
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = timestamp_;
    value_pub_.publish(msg);

    if (comm_->stateMachine()->currentState() ==
        StateMachine::State::kLocalPlanning) {
      for (int i = 0; i < points.size(); ++i) {
        visualizeValue(*(points[i]), min_value, max_value, i);
      }
    }
  }

  if (config_.visualize_gain && gain_pub_.getNumSubscribers() > 0) {
    auto msg = visualization_msgs::Marker();
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = timestamp_;
    gain_pub_.publish(msg);
    if (comm_->stateMachine()->currentState() ==
        StateMachine::State::kLocalPlanning) {
      for (int i = 0; i < points.size(); ++i) {
        visualizeGain(*(points[i]), min_gain, max_gain, i);
      }
    }
  }

  if (config_.visualize_text && text_pub_.getNumSubscribers() > 0) {
    auto msg = visualization_msgs::Marker();
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = timestamp_;
    text_pub_.publish(msg);
    if (comm_->stateMachine()->currentState() ==
        StateMachine::State::kLocalPlanning) {
      for (int i = 0; i < points.size(); ++i) {
        visualizeText(*(points[i]), i);
      }
    }
  }

  if (config_.visualize_visible_voxels && voxel_pub_.getNumSubscribers() > 0) {
    if (comm_->stateMachine()->currentState() ==
        StateMachine::State::kLocalPlanning) {
      // Display only the gain of the next selected viewpoint.
      RHRRTStar::ViewPoint* next_point =
          std::find_if(points.begin(), points.end(),
                       [](const std::unique_ptr<RHRRTStar::ViewPoint>& p) {
                         return p->is_root;
                       })
              ->get();
      if (next_point) {
        visualizeVisibleVoxels(*next_point);
      } else {
        LOG(WARNING) << "Could not find a point labeled root to visualize.";
      }
    } else {
      // Clear the previous visualization.
      auto msg = visualization_msgs::Marker();
      msg.action = visualization_msgs::Marker::DELETEALL;
      msg.header.frame_id = frame_id_;
      msg.header.stamp = timestamp_;
      auto array_msg = visualization_msgs::MarkerArray();
      array_msg.markers.push_back(msg);
      voxel_pub_.publish(array_msg);
    }
  }
}

void RHRRTStarVisualizer::visualizeValue(const RHRRTStar::ViewPoint& point,
                                         FloatingPoint min_value,
                                         FloatingPoint max_value, int id) {
  // Setup marker message
  auto msg = visualization_msgs::Marker();
  msg.header.frame_id = frame_id_;
  msg.header.stamp = timestamp_;
  msg.pose.orientation.w = 1.0;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.id = id;
  msg.scale.x = 0.08;
  msg.color.a = 1;
  msg.action = visualization_msgs::Marker::ADD;

  if (!point.is_root) {
    // Color according to relative value (blue when indifferent)
    if (max_value != min_value) {
      FloatingPoint frac = (point.value - min_value) / (max_value - min_value);
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
    pt.x = point.pose.position.x();
    pt.y = point.pose.position.y();
    pt.z = point.pose.position.z();
    msg.points.push_back(pt);
    RHRRTStar::ViewPoint* viewpoint_end = point.getActiveViewPoint();
    if (viewpoint_end) {
      tf::pointEigenToMsg(viewpoint_end->pose.position.cast<double>(), pt);
    } else {
      LOG(WARNING) << "Tried to visualize a view point without valid "
                      "connected view point.";
      tf::pointEigenToMsg(Eigen::Vector3d::Zero(), pt);
    }
    msg.points.push_back(pt);
  } else {
    msg.points.push_back(geometry_msgs::Point());  // suppress rviz warnings
    msg.points.push_back(geometry_msgs::Point());
  }
  value_pub_.publish(msg);
}

void RHRRTStarVisualizer::visualizeGain(const RHRRTStar::ViewPoint& point,
                                        FloatingPoint min_gain,
                                        FloatingPoint max_gain, int id) {
  auto msg = visualization_msgs::Marker();
  msg.header.frame_id = frame_id_;
  msg.header.stamp = timestamp_;
  msg.type = visualization_msgs::Marker::ARROW;
  msg.action = visualization_msgs::Marker::ADD;
  msg.id = id;
  msg.scale.x = 0.2;
  msg.scale.y = 0.1;
  msg.scale.z = 0.1;
  tf::pointEigenToMsg(point.pose.position.cast<double>(), msg.pose.position);
  tf2::Quaternion q;
  q.setRPY(0, 0, point.pose.yaw);
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();

  // Color according to relative value (blue when indifferent)
  if (min_gain != max_gain) {
    FloatingPoint frac = (point.gain - min_gain) / (max_gain - min_gain);
    msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
    msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
    msg.color.b = 0.0;
  } else {
    msg.color.r = 0.3;
    msg.color.g = 0.3;
    msg.color.b = 1.0;
  }
  msg.color.a = 1.0;
  gain_pub_.publish(msg);
}

void RHRRTStarVisualizer::visualizeText(const RHRRTStar::ViewPoint& point,
                                        int id) {
  auto msg = visualization_msgs::Marker();
  msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  msg.id = id;
  msg.header.stamp = timestamp_;
  msg.header.frame_id = frame_id_;
  msg.scale.z = 0.3;
  msg.color.r = 0.0f;
  msg.color.g = 0.0f;
  msg.color.b = 0.0f;
  msg.color.a = 1.0;
  tf::pointEigenToMsg(point.pose.position.cast<double>(), msg.pose.position);
  FloatingPoint g = point.gain;
  FloatingPoint c;
  const RHRRTStar::Connection* active_connection = point.getActiveConnection();
  if (active_connection) {
    c = active_connection->cost;
  } else {
    LOG(WARNING) << "Tried to visualize a view point without valid "
                    "active connection.";
    c = -1.f;
  }
  FloatingPoint v = point.value;
  std::stringstream stream;
  stream << "g: " << std::fixed << std::setprecision(1)
         << (g > 1000 ? g / 1000 : g) << (g > 1000 ? "k" : "")
         << ", c: " << std::fixed << std::setprecision(1)
         << (c > 1000 ? c / 1000 : c) << (c > 1000 ? "k" : "")
         << ", v: " << std::fixed << std::setprecision(1)
         << (v > 1000 ? v / 1000 : v) << (v > 1000 ? "k" : "");
  msg.text = stream.str();
  msg.action = visualization_msgs::Marker::ADD;
  text_pub_.publish(msg);
}

void RHRRTStarVisualizer::visualizeVisibleVoxels(
    const RHRRTStar::ViewPoint& point) {
  // NOTE(schmluk): This could also be a single message of type cube array but
  // that won't display properly on my rviz.
  auto result = visualization_msgs::MarkerArray();
  std::vector<Point> voxels, colors;
  FloatingPoint scale;
  planner_->visualizeGain(point.pose, &voxels, &colors, &scale);

  // add voxels
  for (size_t i = 0; i < voxels.size(); ++i) {
    auto msg = visualization_msgs::Marker();
    msg.header.frame_id = frame_id_;
    msg.header.stamp = timestamp_;
    msg.pose.orientation.w = 1.0;
    msg.type = visualization_msgs::Marker::CUBE;
    msg.id = i;
    msg.action = visualization_msgs::Marker::ADD;
    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;
    msg.pose.position.x = voxels[i].x();
    msg.pose.position.y = voxels[i].y();
    msg.pose.position.z = voxels[i].z();
    msg.color.r = colors[i].x();
    msg.color.g = colors[i].y();
    msg.color.b = colors[i].z();
    msg.color.a = 0.5;
    result.markers.push_back(msg);
  }
  voxel_pub_.publish(result);
}

}  // namespace glocal_exploration
