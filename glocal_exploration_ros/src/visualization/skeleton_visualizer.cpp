#include "glocal_exploration_ros/visualization/skeleton_visualizer.h"

#include <memory>

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
      nh_.advertise<visualization_msgs::Marker>("executed_path", 100);
  planned_path_pub_ =
      nh_.advertise<visualization_msgs::Marker>("planned_path", 100);
  frontier_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 100);
  goals_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_points", 100);
}

void SkeletonVisualizer::visualize() {
  if (config_.visualize_executed_path && comm_->newWayPointIsRequested() &&
      executed_path_pub_.getNumSubscribers() > 0) {
    visualizeExecutedPath();
  }
  if (config_.visualize_frontiers &&
      planner_->visualizationInfo().frontiers_changed &&
      frontier_pub_.getNumSubscribers() > 0) {
    visualizeFrontiers();
  }
  if (config_.visualize_planned_path && comm_->newWayPointIsRequested() &&
      planned_path_pub_.getNumSubscribers() > 0) {
    visualizePlannedPath();
  }
  if (config_.visualize_candidate_goals &&
      planner_->visualizationInfo().goals_changed &&
      goals_pub_.getNumSubscribers() > 0) {
    visualizeGoalPoints();
  }
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
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
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

void SkeletonVisualizer::visualizeGoalPoints() {
  planner_->visualizationInfo().goals_changed = false;
  // Clear previous messages.
  auto msg = visualization_msgs::Marker();
  msg.action = visualization_msgs::Marker::DELETEALL;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();
  goals_pub_.publish(msg);

  if (comm_->stateMachine()->currentState() ==
      StateMachine::State::kGlobalPlanning) {
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

    // Go through all points.
    int id = 0;
    for (const auto& reachability_point_pair :
         planner_->visualizationInfo().goal_points) {
      msg.pose.position.x = reachability_point_pair.second.x();
      msg.pose.position.y = reachability_point_pair.second.y();
      msg.pose.position.z = reachability_point_pair.second.z();
      msg.id = id++;

      switch (reachability_point_pair.first) {
        case SkeletonPlanner::VisualizationInfo::kReachable: {
          msg.color.r = 0.0;
          msg.color.g = 1.0;
          msg.color.b = 0.0;
          break;
        }
        case SkeletonPlanner::VisualizationInfo::kUnreachable: {
          msg.color.r = 1.0;
          msg.color.g = 0.0;
          msg.color.b = 0.0;
          break;
        }
        case SkeletonPlanner::VisualizationInfo::kUnchecked: {
          msg.color.r = 1.0;
          msg.color.g = 1.0;
          msg.color.b = 0.0;
          break;
        }
        case SkeletonPlanner::VisualizationInfo::kInvalidGoal: {
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

void SkeletonVisualizer::visualizeFrontiers() {
  planner_->visualizationInfo().frontiers_changed = false;

  // Erase previous visualizations
  auto msg = visualization_msgs::Marker();
  msg.header.frame_id = frame_id_;
  msg.header.stamp = ros::Time::now();
  msg.action = visualization_msgs::Marker::DELETEALL;
  auto array_msg = visualization_msgs::MarkerArray();
  array_msg.markers.push_back(msg);
  frontier_pub_.publish(array_msg);

  if (comm_->stateMachine()->currentState() ==
      StateMachine::State::kGlobalPlanning) {
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
