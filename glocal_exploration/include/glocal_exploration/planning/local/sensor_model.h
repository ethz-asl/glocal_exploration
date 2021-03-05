#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_SENSOR_MODEL_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_SENSOR_MODEL_H_

#include <memory>
#include <unordered_set>
#include <utility>

#include "glocal_exploration/mapping/map_base.h"
#include "glocal_exploration/state/region_of_interest.h"
#include "glocal_exploration/state/waypoint.h"

namespace glocal_exploration {

class SensorModel {
 public:
  explicit SensorModel(std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~SensorModel() = default;

  virtual void getVisibleUnknownVoxels(const WayPoint& waypoint,
                                       voxblox::LongIndexSet* voxels) = 0;
  virtual void getVisibleUnknownVoxelsAndOptimalYaw(
      WayPoint* waypoint, voxblox::LongIndexSet* voxels) = 0;

 protected:
  std::shared_ptr<Communicator> comm_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_SENSOR_MODEL_H_
