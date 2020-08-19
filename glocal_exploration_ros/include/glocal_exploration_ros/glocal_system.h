#ifndef GLOCAL_EXPLORATION_ROS_GLOCAL_SYSTEM_H_
#define GLOCAL_EXPLORATION_ROS_GLOCAL_SYSTEM_H_

#include <memory>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <3rd_party/config_utilities.hpp>

#include <glocal_exploration/state/communicator.h>

#include "glocal_exploration_ros/visualization/global_planner_visualizer_base.h"
#include "glocal_exploration_ros/visualization/local_planner_visualizer_base.h"

namespace glocal_exploration {

class GlocalSystem {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
    double replan_position_threshold = 0.2;  // m
    double replan_yaw_threshold = 10.0;      // deg
    double waypoint_timeout = 0.0;           // s

    Config();
    void checkParams() const override;
    void fromRosParam() override;
  };

  // Constructors.
  GlocalSystem(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  GlocalSystem(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
               const Config& config);
  virtual ~GlocalSystem() = default;

  // ROS callbacks.
  void odomCallback(const nav_msgs::Odometry& msg);
  bool runSrvCallback(std_srvs::SetBool::Request& req,    // NOLINT
                      std_srvs::SetBool::Response& res);  // NOLINT

  // spinning is managed explicitly, run this to start the planner.
  void mainLoop();

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers and publishers
  ros::Subscriber odom_sub_;
  ros::Publisher target_pub_;
  ros::ServiceServer run_srv_;

  // Components
  const Config config_;
  std::shared_ptr<Communicator> comm_;
  std::shared_ptr<LocalPlannerVisualizerBase> local_planner_visualizer_;
  std::shared_ptr<GlobalPlannerVisualizerBase> global_planner_visualizer_;

  // methods
  void buildComponents(const ros::NodeHandle& nh);
  bool startExploration();
  void loopIteration();
  void publishTargetPose();

  // variables
  Eigen::Vector3d current_position_;  // current/goal poses are in odom frame.
  Eigen::Quaterniond current_orientation_;
  Eigen::Vector3d target_position_;
  double target_yaw_;                  // rad
  ros::Time time_last_waypoint_started_;  // track waypoint timeout.
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_GLOCAL_SYSTEM_H_
