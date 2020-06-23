#ifndef GLOCAL_EXPLORATION_PLANNERS_GLOCAL_PLANNER_H_
#define GLOCAL_EXPLORATION_PLANNERS_GLOCAL_PLANNER_H_

#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>

#include "glocal_exploration/planning/state_machine.h"
#include "glocal_exploration/mapping/map_base.h"
#include "glocal_exploration/planning/local_planner/local_planner_base.h"

namespace glocal_exploration {

class GlocalPlanner {
 public:
  struct Config{
    double replan_position_threshold = 0.2;   // m
    double replan_yaw_threshold = 10;  // deg
  };

  // Constructor
  GlocalPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~GlocalPlanner() = default;

  // ROS callbacks
  void odomCallback(const nav_msgs::Odometry &msg);
  bool runSrvCallback(std_srvs::SetBool::Request &req,
                      std_srvs::SetBool::Response &res);

  // spinning is managed explicitly, run this to start the planner
  void planningLoop();

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers and publishers
  ros::Subscriber odom_sub_;
  ros::Publisher target_pub_;
  ros::ServiceServer run_srv_;

  // Components
  Config config_;
  const std::shared_ptr<StateMachine> state_machine_;
  std::shared_ptr<MapBase> map_;
  std::unique_ptr<LocalPlannerBase> local_planner_;

  // methods
  void loopIteration();
  void readParamsFromRos();

  // variables
  Eigen::Vector3d current_position_;    // current and goal poses are in odom frame
  Eigen::Quaterniond current_orientation_;
  Eigen::Vector3d target_position_;
  double target_yaw_;   // rad
};

} // namespace glocal_exploration

#endif //GLOCAL_EXPLORATION_PLANNERS_GLOCAL_PLANNER_H_
