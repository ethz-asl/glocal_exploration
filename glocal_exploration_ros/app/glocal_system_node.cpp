#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include <glocal_exploration/3rd_party/config_utilities.hpp>

#include "glocal_exploration_ros/glocal_system.h"

int main(int argc, char** argv) {
  // Setup logging.
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--alsologtostderr", "--colorlogtostderr"});
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Setup ROS.
  ros::init(argc, argv, "glocal_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  // Setup multi-threaded spinning.
  int n_threads;
  nh_private.param("n_threads", n_threads,
                   0);  // 0 defaults to #physical cores available
  ros::AsyncSpinner spinner(n_threads);
  spinner.start();

  // Run node.
  glocal_exploration::GlocalSystem glocal(nh, nh_private);
  glocal.mainLoop();  // the planner manages spinning

  return 0;
}
