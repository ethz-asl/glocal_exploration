#include "glocal_exploration/planning/global/submap_frontiers.h"

#include <utility>
#include <memory>

namespace glocal_exploration {

bool SubmapFrontiers::Config::isValid() const { return true; }

SubmapFrontiers::Config SubmapFrontiers::Config::checkValid() const {
  CHECK(isValid());
  return Config(*this);
}

SubmapFrontiers::SubmapFrontiers(const Config& config,
                                 std::shared_ptr<Communicator> communicator)
    : config_(config.checkValid()),
      GlobalPlannerBase(std::move(communicator)) {}

void SubmapFrontiers::computeFrontiersForSubmap(
    const voxblox::Layer<voxblox::TsdfVoxel>& tsdf_layer, int submap_id) {}

void SubmapFrontiers::updateFrontiers() {}

}  // namespace glocal_exploration
