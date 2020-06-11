#ifndef GLOCAL_EXPLORATION_MAPPING_MAP_INTERFACE_H_
#define GLOCAL_EXPLORATION_MAPPING_MAP_INTERFACE_H_

#include "glocal_exploration/common.h"

namespace glocal_exploration {
/**
 * Defines the interface of a map module that is needed by the planner.
 */
class MapInterface {
 public:
  // Defines a baseclass for map configurations
  struct Config {
    virtual ~Config() = default;
  };
  MapInterface() = default;
  virtual ~MapInterface() = default;

  /* Setup */
  // Can pass derived configs here by base pointer to setup the map.
  virtual bool setupFromConfig(Config *config) = 0;

  /* General and Accessors */
  virtual double getVoxelSize() = 0;

  /* Local planner */
  virtual bool isTraversableInActiveSubmap(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation) = 0;
  // sets *center to the center of the voxel that contains point. Voxels are referred in the planner by their centers.
  virtual bool getVoxelCenterInLocalArea(Eigen::Vector3d *center, const Eigen::Vector3d &point) = 0;
  virtual bool isObservedInLocalArea(const Eigen::Vector3d &point) = 0;
};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_MAPPING_MAP_INTERFACE_H_
