#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_SENSOR_MODEL_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_SENSOR_MODEL_H_

#include <memory>
#include <utility>
#include <vector>

#include "glocal_exploration/mapping/map_base.h"
#include "glocal_exploration/planning/waypoint.h"

namespace glocal_exploration {

class SensorModel {
 public:
  struct Config {
    virtual ~Config() = default;
  };

  explicit SensorModel(std::shared_ptr<MapBase> map,
                       std::shared_ptr<StateMachine> state_machine)
      : map_(std::move(map)), state_machine_(std::move(state_machine)) {}
  virtual ~SensorModel() = default;

  // Return the voxel centers of all visible voxels for that viewpoint
  virtual bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                                const WayPoint& waypoint) = 0;

  // setup from a config, these are also allowed to be derived configs
  virtual bool setupFromConfig(Config* config) = 0;

 protected:
  std::shared_ptr<MapBase> map_;
  std::shared_ptr<StateMachine> state_machine_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_SENSOR_MODEL_H_
