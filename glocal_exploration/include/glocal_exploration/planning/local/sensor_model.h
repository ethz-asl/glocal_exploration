#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_SENSOR_MODEL_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_SENSOR_MODEL_H_

#include <memory>
#include <utility>
#include <vector>

#include "glocal_exploration/mapping/map_base.h"
#include "glocal_exploration/planning/waypoint.h"
#include "glocal_exploration/state/region_of_interest.h"

namespace glocal_exploration {

class SensorModel {
 public:
  explicit SensorModel(std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~SensorModel() = default;

  // Return the voxel centers of all visible voxels for that viewpoint
  virtual bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                                const WayPoint& waypoint) = 0;

 protected:
  std::shared_ptr<Communicator> comm_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_SENSOR_MODEL_H_
