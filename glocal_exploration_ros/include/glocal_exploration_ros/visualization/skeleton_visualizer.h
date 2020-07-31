#ifndef GLOCAL_EXPLORATION_ROS_VISUALIZATION_SKELETON_VISUALIZER_H_
#define GLOCAL_EXPLORATION_ROS_VISUALIZATION_SKELETON_VISUALIZER_H_

#include <memory>
#include <string>

#include "glocal_exploration_ros/planning/global/skeleton_planner.h"
#include "glocal_exploration_ros/visualization/global_planner_visualizer_base.h"

namespace glocal_exploration {

class SkeletonVisualizer : public GlobalPlannerVisualizerBase {
 public:
  struct Config {
    std::string nh_namespace = "";
    bool visualize_gain = true;
    bool visualize_text = true;
    bool visualize_visible_voxels = true;
    bool visualize_value = true;

    [[nodiscard]] bool isValid() const;
    [[nodiscard]] Config checkValid() const;
  };

  SkeletonVisualizer(const Config& config,
                     std::shared_ptr<Communicator> communicator);
  ~SkeletonVisualizer() override = default;

  void visualize() override;

 protected:
  const Config config_;
  std::shared_ptr<SkeletonPlanner> planner_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_VISUALIZATION_SKELETON_VISUALIZER_H_
