#include "glocal_exploration_ros/visualization/skeleton_visualizer.h"

#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <visualization_msgs/MarkerArray.h>
#include <voxblox/utils/color_maps.h>

namespace glocal_exploration {

SkeletonVisualizer::Config::Config() { setConfigName("SkeletonVisualizer"); }

void SkeletonVisualizer::Config::checkParams() const {}

void SkeletonVisualizer::Config::fromRosParam() {
  rosParam("visualize_frontiers", &visualize_frontiers);
  rosParam("visualize_inactive_frontiers", &visualize_inactive_frontiers);
  rosParam("visualize_executed_path", &visualize_executed_path);
  rosParam("visualize_candidate_goals", &visualize_candidate_goals);
  rosParam("visualize_planned_path", &visualize_planned_path);
  rosParam("visualize_frontier_text", &visualize_frontier_text);
  nh_namespace = rosParamNameSpace();
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

  // ROS
  nh_ = ros::NodeHandle(config_.nh_namespace);
  executed_path_pub_ =
      nh_.advertise<visualization_msgs::Marker>("executed_path", queue_size_);
  planned_path_pub_ =
      nh_.advertise<visualization_msgs::Marker>("planned_path", queue_size_);
  frontier_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("frontiers", queue_size_);
  goals_pub_ =
      nh_.advertise<visualization_msgs::Marker>("goal_points", queue_size_);
  frontier_text_pub_ =
      nh_.advertise<visualization_msgs::Marker>("frontier_text", queue_size_);
}

void SkeletonVisualizer::visualize() {
  if (!comm_->newWayPointIsRequested()) {
    // Only visualize after each waypoint.
    return;
  }
  // Paths.
  if (config_.visualize_executed_path &&
      executed_path_pub_.getNumSubscribers() > 0) {
    visualizeExecutedPath();
  }
  if (config_.visualize_planned_path &&
      planned_path_pub_.getNumSubscribers() > 0) {
    visualizePlannedPath();
  }

  // Only visualize frontiers if these were updated. Check for changes in the
  // number of goal points and number of frontier points. This is extremely
  // unlikely to be identical after recomputation.
  int goals = planner_->getFrontierSearchData().size();
  int points = 0;
  for (const auto& frontier : planner_->getFrontierSearchData()) {
    points += frontier.num_points;
  }
  frontiers_have_changed =
      !(goals == num_prev_goals && points == num_prev_points);
  num_prev_goals = goals;
  num_prev_points = points;

  // Frontiers.
  if (config_.visualize_frontiers &&
      frontier_pub_.getNumSubscribers() > 0) {
    visualizeFrontiers();
  }
  if (config_.visualize_frontier_text &&
      frontier_pub_.getNumSubscribers() > 0) {
    visualizeFrontierText();
  }
  if (config_.visualize_candidate_goals &&
      goals_pub_.getNumSubscribers() > 0) {
    visualizeGoalPoints();
  }

  // After one visualization pass set changed to false.
  frontiers_have_changed = false;
}

void SkeletonVisualizer::visualizePlannedPath() {
  // Clear previous messages.
  auto msg = visualization_msgs::Marker();
  msg.action = visualization_msgs::Marker::DELETEALL;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();
  planned_path_pub_.publish(msg);

  if (comm_->stateMachine()->currentState() ==
      StateMachine::State::kGlobalPlanning) {
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
    pt.x = comm_->getRequestedWayPoint().x;
    pt.y = comm_->getRequestedWayPoint().y;
    pt.z = comm_->getRequestedWayPoint().z;
    msg.points.push_back(pt);
    pt.x = planner_->getWayPoints()[0].x;
    pt.y = planner_->getWayPoints()[0].y;
    pt.z = planner_->getWayPoints()[0].z;
    msg.points.push_back(pt);
    planned_path_pub_.publish(msg);
    for (int i = 1; i < planner_->getWayPoints().size(); ++i) {
      msg.id = i;
      msg.points.clear();
      pt.x = planner_->getWayPoints()[i - 1].x;
      pt.y = planner_->getWayPoints()[i - 1].y;
      pt.z = planner_->getWayPoints()[i - 1].z;
      msg.points.push_back(pt);
      pt.x = planner_->getWayPoints()[i].x;
      pt.y = planner_->getWayPoints()[i].y;
      pt.z = planner_->getWayPoints()[i].z;
      msg.points.push_back(pt);
      planned_path_pub_.publish(msg);
    }
  }
}

void SkeletonVisualizer::visualizeExecutedPath() {
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
  pt.x = comm_->getPreviousWayPoint().x;
  pt.y = comm_->getPreviousWayPoint().y;
  pt.z = comm_->getPreviousWayPoint().z;
  msg.points.push_back(pt);
  pt.x = comm_->getRequestedWayPoint().x;
  pt.y = comm_->getRequestedWayPoint().y;
  pt.z = comm_->getRequestedWayPoint().z;
  msg.points.push_back(pt);
  executed_path_pub_.publish(msg);
}

void SkeletonVisualizer::visualizeGoalPoints() {
  if (frontiers_have_changed) {
    // Clear previous messages.
    auto msg = visualization_msgs::Marker();
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = ros::Time::now();
    goals_pub_.publish(msg);

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

    // Go through all goal points.
    int id = 0;
    for (const auto& goal : planner_->getFrontierSearchData()) {
      msg.pose.position.x = goal.centroid.x();
      msg.pose.position.y = goal.centroid.y();
      msg.pose.position.z = goal.centroid.z();
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

void SkeletonVisualizer::visualizeFrontierText() {
  if (frontiers_have_changed) {
    // Clear previous messages.
    auto msg = visualization_msgs::Marker();
    msg.action = visualization_msgs::Marker::DELETEALL;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = ros::Time::now();
    frontier_text_pub_.publish(msg);

    // Common data.
    msg.header.stamp = ros::Time::now();
    msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    msg.action = visualization_msgs::Marker::ADD;
    msg.scale.z = 0.5;
    msg.color.r = 0.0f;
    msg.color.g = 0.0f;
    msg.color.b = 0.0f;
    msg.color.a = 1.0;

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

std::string SkeletonVisualizer::frontierTextFormat(double value) const {
  if (value == std::numeric_limits<double>::max()) {
    return "-";
  }
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << value;
  return ss.str();
}

void SkeletonVisualizer::visualizeFrontiers() {
  if (frontiers_have_changed) {
    // Erase previous visualizations
    auto msg = visualization_msgs::Marker();
    msg.header.frame_id = frame_id_;
    msg.header.stamp = ros::Time::now();
    msg.action = visualization_msgs::Marker::DELETEALL;
    auto array_msg = visualization_msgs::MarkerArray();
    array_msg.markers.push_back(msg);
    frontier_pub_.publish(array_msg);

    frontier_msg_id_ = 0;
    // Visualize all active frontiers.
    int color_id = 0;
    for (const auto& frontier_collection : planner_->getUpdatedCollections()) {
      for (const auto& frontier :
           frontier_collection.second.getActiveFrontiers()) {
        visualizeFrontier(*frontier, false, color_id);
        color_id++;
      }
    }

    // Visualize all inactive frontiers.
    if (config_.visualize_inactive_frontiers) {
      for (const auto& frontier_collection :
           planner_->getCandidateCollections()) {
        for (const auto& frontier :
             frontier_collection.second.getActiveFrontiers()) {
          // Color ids are the submap ids.
          visualizeFrontier(*frontier, true, frontier_collection.first);
        }
      }
    }
  }
}

void SkeletonVisualizer::visualizeFrontier(const Frontier& frontier,
                                           bool show_inactive_points,
                                           int color_id) {
  auto result = visualization_msgs::MarkerArray();
  if (frontier.isActive() == show_inactive_points) {
    return;
  }

  voxblox::ExponentialOffsetColorMap color_map;
  voxblox::Color color = color_map.colorLookup(color_id);
  for (const FrontierCandidate& point : frontier) {
    if (point.is_active == show_inactive_points) {
      continue;
    }
    auto msg = visualization_msgs::Marker();
    msg.header.frame_id = frame_id_;
    msg.header.stamp = ros::Time::now();
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.orientation.w = 1.0;
    msg.type = visualization_msgs::Marker::CUBE;
    msg.id = frontier_msg_id_++;
    double scale = comm_->map()->getVoxelSize();
    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;
    msg.pose.position.x = point.position.x();
    msg.pose.position.y = point.position.y();
    msg.pose.position.z = point.position.z();
    msg.color.r = color.r / 255.0;
    msg.color.g = color.g / 255.0;
    msg.color.b = color.b / 255.0;
    if (point.is_active) {
      msg.color.a = 1.0;
    } else {
      msg.color.a = 0.2;
    }
    result.markers.push_back(msg);
  }
  frontier_pub_.publish(result);
}

}  // namespace glocal_exploration
