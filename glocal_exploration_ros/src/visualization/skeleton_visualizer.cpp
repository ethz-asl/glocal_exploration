#include "glocal_exploration_ros/visualization/skeleton_visualizer.h"

#include <memory>
#include <utility>

namespace glocal_exploration {

bool SkeletonVisualizer::Config::isValid() const {}

SkeletonVisualizer::Config SkeletonVisualizer::Config::checkValid() const {}

SkeletonVisualizer::SkeletonVisualizer(
    const Config& config, std::shared_ptr<Communicator> communicator)
    : config_(config.checkValid()),
      GlobalPlannerVisualizerBase(std::move(communicator)) {}

void SkeletonVisualizer::visualize() {}

}  // namespace glocal_exploration
