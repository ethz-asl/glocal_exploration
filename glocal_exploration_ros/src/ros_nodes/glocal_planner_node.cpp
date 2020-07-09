#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "glocal_exploration_ros/planning/glocal_planner.h"

int main(int argc, char** argv) {
  // Setup logging
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup ROS
  ros::init(argc, argv, "glocal_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Setup multi-threaded spinning
  int n_threads;
  nh_private.param("n_threads", n_threads,
                   0);  // 0 defaults to #physical cores available
  ros::AsyncSpinner spinner(n_threads);
  spinner.start();

  // Run node
  glocal_exploration::GlocalPlanner planner(nh, nh_private);
  planner.planningLoop();  // the planner manages spinning

  return 0;
}