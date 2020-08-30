#ifndef GLOCAL_EXPLORATION_MAPPING_MAP_BASE_H_
#define GLOCAL_EXPLORATION_MAPPING_MAP_BASE_H_

#include <memory>
#include <utility>
#include <vector>

#include <voxblox/core/layer.h>

#include "glocal_exploration/common.h"

namespace glocal_exploration {

class Communicator;

/**
 * Defines the interface of a map module that is needed by the planner.
 */
class MapBase {
 public:
  enum class VoxelState { kUnknown, kOccupied, kFree };

  struct SubmapData {
    int id;
    Transformation T_M_S;
    std::shared_ptr<const voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;
  };

  explicit MapBase(std::shared_ptr<Communicator> communicator)
      : comm_(std::move(communicator)) {}
  virtual ~MapBase() = default;

  /* General and Accessors */
  virtual double getVoxelSize() = 0;

  /* Local planner */
  virtual bool isTraversableInActiveSubmap(const Point& position) = 0;

  // Voxels are referred in the planner by their center points.
  virtual Point getVoxelCenterInLocalArea(const Point& position) = 0;

  virtual VoxelState getVoxelStateInLocalArea(const Point& position) = 0;

  /* Global planner */
  virtual bool isObservedInGlobalMap(const Point& position) = 0;

  virtual bool isTraversableInGlobalMap(const Point& position) = 0;

  virtual std::vector<SubmapData> getAllSubmapData() = 0;

 protected:
  const std::shared_ptr<Communicator> comm_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_MAPPING_MAP_BASE_H_
