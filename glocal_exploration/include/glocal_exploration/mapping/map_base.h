#ifndef GLOCAL_EXPLORATION_MAPPING_MAP_BASE_H_
#define GLOCAL_EXPLORATION_MAPPING_MAP_BASE_H_

#include <memory>
#include <utility>

#include "glocal_exploration/common.h"

namespace glocal_exploration {
/**
 * Defines the interface of a map module that is needed by the planner.
 */

class Communicator;

class MapBase {
 public:
  // Defines a baseclass for map configurations
  struct Config {
    virtual ~Config() = default;
  };

  enum VoxelState { kUnknown, kOccupied, kFree };

  // Constructors
  explicit MapBase(std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~MapBase() = default;

  /* Setup */
  // Can pass derived configs here by base pointer to setup the map.
  virtual bool setupFromConfig(Config* config) = 0;

  /* General and Accessors */
  virtual double getVoxelSize() = 0;

  /* Local planner */
  virtual bool isTraversableInActiveSubmap(
      const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation =
                                           Eigen::Quaterniond::Identity()) = 0;

  // Voxels are referred in the planner by their center points.
  virtual Eigen::Vector3d getVoxelCenterInLocalArea(
      const Eigen::Vector3d& point) = 0;

  virtual VoxelState getVoxelStateInLocalArea(const Eigen::Vector3d& point) = 0;

 protected:
  const std::shared_ptr<Communicator> comm_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_MAPPING_MAP_BASE_H_
