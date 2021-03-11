#include "glocal_exploration_ros/visualization/skeleton_visualizer.h"

#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <voxblox/utils/color_maps.h>
#include <voxblox_skeleton/ros/skeleton_vis.h>

namespace glocal_exploration {

SkeletonVisualizer::Config::Config() { setConfigName("SkeletonVisualizer"); }

void SkeletonVisualizer::Config::checkParams() const {}

void SkeletonVisualizer::Config::fromRosParam() {
  rosParam("visualize_frontiers", &visualize_frontiers);
  rosParam("visualize_executed_path", &visualize_executed_path);
  rosParam("visualize_candidate_goals", &visualize_candidate_goals);
  rosParam("visualize_planned_path", &visualize_planned_path);
  rosParam("visualize_frontier_text", &visualize_frontier_text);
  rosParam("visualize_inactive_frontiers", &visualize_inactive_frontiers);
  rosParam("keep_visualizations", &keep_visualizations);
  nh_namespace = rosParamNameSpace();
}

SkeletonVisualizer::SkeletonVisualizer(
    const Config& config, const std::shared_ptr<Communicator>& communicator)
    : GlobalPlannerVisualizerBase(communicator), config_(config.checkValid()) {
  // Reference planner.
  planner_ = std::dynamic_pointer_cast<SkeletonPlanner>(comm_->globalPlanner());
  if (!planner_) {
    LOG(FATAL) << "Can not setup 'SkeletonVisualizer' with a global planner "
                  "that is not of type 'SkeletonPlanner'.";
  }

  // ROS
  nh_ = ros::NodeHandle(config_.nh_namespace);
  executed_path_pub_ =
      nh_.advertise<visualization_msgs::Marker>("executed_path", queue_size_);
  planned_path_pub_ =
      nh_.advertise<visualization_msgs::Marker>("planned_path", queue_size_);
  path_search_pub_ =
      nh_.advertise<visualization_msgs::Marker>("path_search", queue_size_);
  frontier_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("frontiers", queue_size_);
  goals_pub_ =
      nh_.advertise<visualization_msgs::Marker>("goal_points", queue_size_);
  frontier_text_pub_ =
      nh_.advertise<visualization_msgs::Marker>("frontier_text", queue_size_);
  inactive_frontiers_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "inactive_frontiers", queue_size_);
  skeleton_submaps_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "skeleton_submaps", queue_size_);
}

void SkeletonVisualizer::visualize() {
  // Paths.
  if (config_.visualize_executed_path &&
      executed_path_pub_.getNumSubscribers() > 0) {
    visualizeExecutedPath();
  }
  if (config_.visualize_planned_path &&
      planned_path_pub_.getNumSubscribers() > 0) {
    visualizePlannedPath();
  }
  if (config_.visualize_path_search &&
      path_search_pub_.getNumSubscribers() > 0) {
    visualizePathSearch();
  }

  // Frontiers.
  if (config_.visualize_frontiers && frontier_pub_.getNumSubscribers() > 0) {
    visualizeFrontiers();
  }
  if (config_.visualize_frontier_text &&
      frontier_pub_.getNumSubscribers() > 0) {
    visualizeFrontierText();
  }
  if (config_.visualize_candidate_goals && goals_pub_.getNumSubscribers() > 0) {
    visualizeGoalPoints();
  }
  if (config_.visualize_inactive_frontiers &&
      inactive_frontiers_pub_.getNumSubscribers() > 0) {
    visualizeInactiveFrontiers();
  }

  // Skeleton submaps.
  if (config_.visualize_skeleton_submaps &&
      skeleton_submaps_pub_.getNumSubscribers() > 0) {
    visualizeSkeletonSubmaps();
  }

  // Visualization stat tracking.
  planner_->visualizationData().frontiers_have_changed = false;
}

void SkeletonVisualizer::visualizePlannedPath() {
  if (!comm_->newWayPointIsRequested()) {
    // Only visualize after each waypoint.
    return;
  }

  // Clear previous messages.
  auto msg = visualization_msgs::Marker();
  msg.action = visualization_msgs::Marker::DELETEALL;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();
  planned_path_pub_.publish(msg);

  if (!planner_->visualizationData().execution_finished) {
    if (planner_->getWayPoints().empty()) {
      return;
    }
    // Setup common data.
    msg.pose.orientation.w = 1.0;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.scale.x = 0.08;
    msg.color.a = 1;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    msg.action = visualization_msgs::Marker::ADD;

    // Publish segments.
    msg.id = 0;
    geometry_msgs::Point pt;
    tf::pointEigenToMsg(comm_->getRequestedWayPoint().position.cast<double>(),
                        pt);
    msg.points.push_back(pt);
    tf::pointEigenToMsg(planner_->getWayPoints()[0].position.cast<double>(),
                        pt);
    msg.points.push_back(pt);
    planned_path_pub_.publish(msg);
    for (int i = 1; i < planner_->getWayPoints().size(); ++i) {
      msg.id = i;
      msg.points.clear();
      tf::pointEigenToMsg(
          planner_->getWayPoints()[i - 1].position.cast<double>(), pt);
      msg.points.push_back(pt);
      tf::pointEigenToMsg(planner_->getWayPoints()[i].position.cast<double>(),
                          pt);
      msg.points.push_back(pt);
      planned_path_pub_.publish(msg);
    }
  }
}

void SkeletonVisualizer::visualizeExecutedPath() {
  if (!comm_->newWayPointIsRequested()) {
    // Only visualize after each waypoint.
    return;
  }

  // Executed path, global planning is in teal.
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();
  msg.pose.orientation.w = 1.0;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.id = executed_path_id_++;
  msg.scale.x = 0.08;
  msg.color.a = 1;
  msg.color.r = 0.0;
  msg.color.g = 0.8;
  msg.color.b = 0.8;
  msg.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point pt;
  tf::pointEigenToMsg(comm_->getPreviousWayPoint().position.cast<double>(), pt);
  msg.points.push_back(pt);
  tf::pointEigenToMsg(comm_->getRequestedWayPoint().position.cast<double>(),
                      pt);
  msg.points.push_back(pt);
  executed_path_pub_.publish(msg);
}

void SkeletonVisualizer::visualizePathSearch() {
  SkeletonAStar::VisualizationEdges visualization_edges =
      planner_->getVisualizationEdges();
  // Setup the marker
  visualization_msgs::Marker marker_msg;
  marker_msg.header.frame_id = "odom";
  marker_msg.header.stamp = ros::Time();
  marker_msg.ns = "a_star_search";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::LINE_LIST;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.pose.position.x = 0.0;
  marker_msg.pose.position.y = 0.0;
  marker_msg.pose.position.z = 0.0;
  marker_msg.pose.orientation.x = 0.0;
  marker_msg.pose.orientation.y = 0.0;
  marker_msg.pose.orientation.z = 0.0;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.scale.x = 0.08;
  marker_msg.color.a = 1.0;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;

  // Add the parent to child edges
  std_msgs::ColorRGBA parent_child_edge_color;
  parent_child_edge_color.r = 0.0;
  parent_child_edge_color.g = 1.0;
  parent_child_edge_color.b = 0.0;
  parent_child_edge_color.a = 1.0;
  for (const auto& parent_child_edge : visualization_edges.parent_map_) {
    geometry_msgs::Point parent_position_msg, child_position_msg;
    if (getVertexPositionMsg(parent_child_edge.first, &parent_position_msg) &&
        getVertexPositionMsg(parent_child_edge.second, &child_position_msg)) {
      marker_msg.points.emplace_back(parent_position_msg);
      marker_msg.colors.emplace_back(parent_child_edge_color);
      marker_msg.points.emplace_back(child_position_msg);
      marker_msg.colors.emplace_back(parent_child_edge_color);
    }
  }

  // Add the intraversable edges
  std_msgs::ColorRGBA intraversable_edge_color;
  intraversable_edge_color.r = 1.0;
  intraversable_edge_color.g = 0.0;
  intraversable_edge_color.b = 0.0;
  intraversable_edge_color.a = 1.0;
  for (const auto& intraversable_edge :
       visualization_edges.intraversable_edge_map_) {
    geometry_msgs::Point start_position_msg, end_position_msg;
    if (getVertexPositionMsg(intraversable_edge.first, &start_position_msg) &&
        getVertexPositionMsg(intraversable_edge.second, &end_position_msg)) {
      marker_msg.points.emplace_back(start_position_msg);
      marker_msg.colors.emplace_back(intraversable_edge_color);
      marker_msg.points.emplace_back(end_position_msg);
      marker_msg.colors.emplace_back(intraversable_edge_color);
    }
  }

  path_search_pub_.publish(marker_msg);
}

void SkeletonVisualizer::visualizeGoalPoints() {
  if (planner_->visualizationData().frontiers_have_changed ||
      planner_->visualizationData().execution_finished) {
    // Clear previous messages.
    auto msg = visualization_msgs::Marker();
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = ros::Time::now();
    goals_pub_.publish(msg);

    if (!planner_->visualizationData().execution_finished) {
      // Common data.
      msg = visualization_msgs::Marker();
      msg.header.frame_id = frame_id_;
      msg.header.stamp = ros::Time::now();
      msg.type = visualization_msgs::Marker::SPHERE;
      msg.action = visualization_msgs::Marker::ADD;
      msg.scale.x = 0.7;
      msg.scale.y = 0.7;
      msg.scale.z = 0.7;
      msg.pose.orientation.w = 1.0;
      msg.color.a = 1.0;
      if (planner_->visualizationData().execution_finished) {
        msg.lifetime = failed_timeout_;
      }

      // Go through all goal points.
      int id = 0;
      for (const auto& goal : planner_->getFrontierSearchData()) {
        tf::pointEigenToMsg(goal.centroid.cast<double>(), msg.pose.position);
        msg.id = id++;

        switch (goal.reachability) {
          case SkeletonPlanner::FrontierSearchData::kReachable: {
            msg.color.r = 0.0;
            msg.color.g = 1.0;
            msg.color.b = 0.0;
            break;
          }
          case SkeletonPlanner::FrontierSearchData::kUnreachable: {
            msg.color.r = 1.0;
            msg.color.g = 0.0;
            msg.color.b = 0.0;
            break;
          }
          case SkeletonPlanner::FrontierSearchData::kUnchecked: {
            msg.color.r = 1.0;
            msg.color.g = 1.0;
            msg.color.b = 0.0;
            break;
          }
          case SkeletonPlanner::FrontierSearchData::kInvalidGoal: {
            msg.color.r = 1.0;
            msg.color.g = 0.0;
            msg.color.b = 1.0;
            break;
          }
        }
        goals_pub_.publish(msg);
      }
    }
  }
}

void SkeletonVisualizer::visualizeFrontierText() {
  if (planner_->visualizationData().frontiers_have_changed ||
      planner_->visualizationData().execution_finished) {
    // Clear previous messages.
    auto msg = visualization_msgs::Marker();
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = ros::Time::now();
    frontier_text_pub_.publish(msg);

    if (!planner_->visualizationData().execution_finished) {
      // Common data.
      msg.header.stamp = ros::Time::now();
      msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      msg.action = visualization_msgs::Marker::ADD;
      msg.scale.z = 0.5;
      msg.color.r = 0.0f;
      msg.color.g = 0.0f;
      msg.color.b = 0.0f;
      msg.color.a = 1.0;
      if (planner_->visualizationData().execution_finished) {
        msg.lifetime = failed_timeout_;
      }

      // Go through all goal points.
      int id = 0;
      for (const auto& frontier : planner_->getFrontierSearchData()) {
        msg.pose.position.x = frontier.centroid.x();
        msg.pose.position.y = frontier.centroid.y();
        msg.pose.position.z = frontier.centroid.z();
        msg.id = id++;
        std::stringstream ss;
        ss << "Path: " << frontierTextFormat(frontier.path_distance)
           << "\nDistance: " << frontierTextFormat(frontier.euclidean_distance)
           << "\nState: ";
        switch (frontier.reachability) {
          case SkeletonPlanner::FrontierSearchData::kReachable: {
            ss << "Reachable";
            break;
          }
          case SkeletonPlanner::FrontierSearchData::kUnreachable: {
            ss << "Unreachable";
            break;
          }
          case SkeletonPlanner::FrontierSearchData::kUnchecked: {
            ss << "Suboptimal";
            break;
          }
          case SkeletonPlanner::FrontierSearchData::kInvalidGoal: {
            ss << "InvalidGoal";
            break;
          }
        }
        ss << "\nPoints: " << frontier.num_points
           << "\nClusters: " << frontier.clusters;
        msg.text = ss.str();
        frontier_text_pub_.publish(msg);
      }
    }
  }
}

void SkeletonVisualizer::visualizeFrontiers() {
  if (planner_->visualizationData().frontiers_have_changed ||
      planner_->visualizationData().execution_finished) {
    pcl::PointCloud<pcl::PointXYZRGB> frontier_points;
    if (!planner_->visualizationData().execution_finished) {
      int color_id = 0;
      voxblox::ExponentialOffsetIdColorMap color_map;
      for (const auto& frontier : planner_->getActiveFrontiers()) {
        // All points on one frontier share the same color
        voxblox::Color frontier_color = color_map.colorLookup(color_id++);
        for (const Point& point : frontier) {
          pcl::PointXYZRGB frontier_point_msg;

          frontier_point_msg.x = static_cast<float>(point.x());
          frontier_point_msg.y = static_cast<float>(point.y());
          frontier_point_msg.z = static_cast<float>(point.z());
          frontier_point_msg.r = frontier_color.r;
          frontier_point_msg.g = frontier_color.g;
          frontier_point_msg.b = frontier_color.b;

          frontier_points.push_back(frontier_point_msg);
        }
      }
    }
    // NOTE: In case the planner did not finish successfully, an empty
    //       pointcloud will still be published to overwrite the previous one
    sensor_msgs::PointCloud2 frontier_points_msg;
    pcl::toROSMsg(frontier_points, frontier_points_msg);
    frontier_points_msg.header.frame_id = frame_id_;
    frontier_points_msg.header.stamp = ros::Time::now();
    frontier_pub_.publish(frontier_points_msg);
  }
}

void SkeletonVisualizer::visualizeInactiveFrontiers() {
  if (planner_->visualizationData().frontiers_have_changed ||
      planner_->visualizationData().execution_finished) {
    pcl::PointCloud<pcl::PointXYZRGB> frontier_points;
    if (!planner_->visualizationData().execution_finished) {
      // Visualize all inactive frontiers.
      const voxblox::Color inactive_color(50, 50, 50);

      for (const Point& point : planner_->getInactiveFrontiers()) {
        pcl::PointXYZRGB frontier_point_msg;
        frontier_point_msg.x = static_cast<float>(point.x());
        frontier_point_msg.y = static_cast<float>(point.y());
        frontier_point_msg.z = static_cast<float>(point.z());
        frontier_point_msg.r = inactive_color.r;
        frontier_point_msg.g = inactive_color.g;
        frontier_point_msg.b = inactive_color.b;
        frontier_points.push_back(frontier_point_msg);
      }
    }
    // NOTE: In case the planner did not finish successfully, an empty
    //       pointcloud will still be published to overwrite the previous one.
    sensor_msgs::PointCloud2 frontier_points_msg;
    pcl::toROSMsg(frontier_points, frontier_points_msg);
    frontier_points_msg.header.frame_id = frame_id_;
    frontier_points_msg.header.stamp = ros::Time::now();
    inactive_frontiers_pub_.publish(frontier_points_msg);
  }
}

void SkeletonVisualizer::visualizeSkeletonSubmaps() {
  voxblox::ExponentialOffsetIdColorMap submap_id_color_map;
  visualization_msgs::MarkerArray marker_array;
  for (const auto& submap_ptr :
       planner_->getSkeletonSubmapCollection().getSubmapConstPtrs()) {
    // Generate the graph markers
    visualization_msgs::MarkerArray submap_marker_array;
    std::string submap_frame_id = submap_ptr->getFrameId();
    voxblox::visualizeSkeletonGraph(submap_ptr->getSkeletonGraph(),
                                    submap_frame_id, &submap_marker_array);

    // Namespace and recolor by ID
    const voxblox::Color submap_color =
        submap_id_color_map.colorLookup(submap_ptr->getId());
    const voxblox::Color submap_vertex_color = voxblox::Color::blendTwoColors(
        submap_color, 0.7f, voxblox::Color::Black(), 0.3f);
    for (auto& marker : submap_marker_array.markers) {
      if (marker.ns == "vertices") {
        voxblox::colorVoxbloxToMsg(submap_vertex_color, &marker.color);
        marker.colors.clear();
      } else if (marker.ns == "edges") {
        voxblox::colorVoxbloxToMsg(submap_color, &marker.color);
      }
      marker.ns = submap_frame_id + "_" + marker.ns;
    }

    // Concatenate
    marker_array.markers.insert(
        marker_array.markers.end(),
        std::make_move_iterator(submap_marker_array.markers.begin()),
        std::make_move_iterator(submap_marker_array.markers.end()));
  }
  skeleton_submaps_pub_.publish(marker_array);
}

std::string SkeletonVisualizer::frontierTextFormat(FloatingPoint value) const {
  if (value == std::numeric_limits<FloatingPoint>::max()) {
    return "-";
  }
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << value;
  return ss.str();
}

bool SkeletonVisualizer::getVertexPositionMsg(
    const GlobalVertexId& global_vertex_id,
    geometry_msgs::Point* position_msg) {
  CHECK_NOTNULL(position_msg);
  SkeletonSubmap::ConstPtr submap_ptr =
      planner_->getSkeletonSubmapCollection().getSubmapConstPtrById(
          global_vertex_id.submap_id);
  if (submap_ptr) {
    const Point t_submap_vertex = submap_ptr->getSkeletonGraph()
                                      .getVertex(global_vertex_id.vertex_id)
                                      .point;
    const Point t_odom_vertex = submap_ptr->getPose() * t_submap_vertex;
    tf::pointEigenToMsg(t_odom_vertex.cast<double>(), *position_msg);
    return true;
  } else {
    if (global_vertex_id.submap_id != RelativeWayPoint::kOdomFrameId) {
      LOG(WARNING) << "Could not get pointer to submap with ID: "
                   << global_vertex_id.submap_id;
    }
    return false;
  }
}

}  // namespace glocal_exploration
