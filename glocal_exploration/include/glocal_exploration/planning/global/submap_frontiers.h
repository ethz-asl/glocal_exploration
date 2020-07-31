#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIERS_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIERS_H_

#include <memory>

#include "glocal_exploration/planning/global/global_planner_base.h"

namespace glocal_exploration {
/**
 * A class that tracks and updates frontiers on tsdf-(sub)-maps for target
 * selection in a global planner.
 */
class SubmapFrontiers : public GlobalPlannerBase {
 public:
  struct Config {
    [[nodiscard]] bool isValid() const;
    [[nodiscard]] Config checkValid() const;
  };

  SubmapFrontiers(const Config& config,
                  std::shared_ptr<Communicator> communicator);
  ~SubmapFrontiers() override = default;

  // NOTE(schmluk): these are curently exposed in the base class for simplicity.
  void computeFrontiersForSubmap(
      const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer,
      int submap_id) override;
  void updateFrontiers() override;

 private:
  const Config config_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SUBMAP_FRONTIERS_H_
