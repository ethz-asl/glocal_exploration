#ifndef GLOCAL_EXPLORATION_MAPPING_MAP_INTERFACE_H_
#define GLOCAL_EXPLORATION_MAPPING_MAP_INTERFACE_H_

#include <memory>

#include "glocal_exploration/common.h"
#include "glocal_exploration/planning/state_machine.h"

namespace glocal_exploration {
/**
 * Defines the interface of a map module that is needed by the planner.
 */

class MapBase {
 public:
  // Defines a baseclass for map configurations
  struct Config {
    virtual ~Config() = default;
  };

  enum VoxelState { Unknown, Occupied, Free };

  // Constructors
  explicit MapBase(std::shared_ptr<StateMachine> state_machine) : state_machine_(std::move(state_machine)) {}
  virtual ~MapBase() = default;

  /* Setup */
  // Can pass derived configs here by base pointer to setup the map.
  virtual bool setupFromConfig(Config *config) = 0;

  /* General and Accessors */
  virtual double getVoxelSize() = 0;

  /* Local planner */
  virtual bool isTraversableInActiveSubmap(const Eigen::Vector3d &position,
                                           const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity()) = 0;
  // sets *center to the center of the voxel that contains point. Voxels are referred in the planner by their centers.
  virtual bool getVoxelCenterInLocalArea(Eigen::Vector3d *center, const Eigen::Vector3d &point) = 0;
  virtual VoxelState getVoxelStateInLocalArea(const Eigen::Vector3d &point) = 0;

 protected:
  const std::shared_ptr<StateMachine> state_machine_;
};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_MAPPING_MAP_INTERFACE_H_
