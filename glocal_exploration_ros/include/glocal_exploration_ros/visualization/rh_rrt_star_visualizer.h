#ifndef GLOCAL_EXPLORATION_VISUALIZATION_RH_RRT_STAR_VISUALIZER_
#define GLOCAL_EXPLORATION_VISUALIZATION_RH_RRT_STAR_VISUALIZER_

#include "glocal_exploration/planning/local/rh_rrt_star.h"
#include "glocal_exploration_ros/visualization/local_planner_visualizer_base.h"

namespace glocal_exploration {

class RHRRTStarVisualizer : public LocalPlannerVisualizerBase {
 public:
  RHRRTStarVisualizer(const ros::NodeHandle& nh,
                      const std::shared_ptr<LocalPlannerBase>& planner);

  void visualize() override;

 protected:
  std::shared_ptr<RHRRTStar> planner_;
  ros::Publisher pub_;

  // params
  bool visualize_gain_;
  bool visualize_text_;
  bool visualize_visible_voxels_;
  bool visualize_value_;

  // variables
  int num_previous_msgs_;
  int num_previous_visible_voxels_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_VISUALIZATION_RH_RRT_STAR_VISUALIZER_
