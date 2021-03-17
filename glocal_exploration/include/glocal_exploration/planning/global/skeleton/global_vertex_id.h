#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_GLOBAL_VERTEX_ID_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_GLOBAL_VERTEX_ID_H_

#include <voxblox_skeleton/skeleton.h>

#include "glocal_exploration/common.h"

namespace glocal_exploration {
// NOTE: The VertexIdElement type must match voxblox::SkeletonVertex::vertex_id
using VertexIdElement = int64_t;

struct GlobalVertexId {
  SubmapId submap_id = -1;
  VertexIdElement vertex_id = -1;

  friend bool operator==(const GlobalVertexId& lhs, const GlobalVertexId& rhs) {
    return (lhs.submap_id == rhs.submap_id) && (lhs.vertex_id == rhs.vertex_id);
  }

  friend bool operator<(const GlobalVertexId& lhs, const GlobalVertexId& rhs) {
    if (lhs.submap_id < rhs.submap_id) {
      return true;
    } else if (lhs.submap_id == rhs.submap_id) {
      return lhs.vertex_id < rhs.vertex_id;
    } else {
      return false;
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const GlobalVertexId& rhs) {
    return os << "(" << rhs.submap_id << ", " << rhs.vertex_id << ")";
  }
};

struct GlobalVertexIdHash {
  static constexpr size_t sl = 17191;

  std::size_t operator()(const GlobalVertexId& index) const {
    return static_cast<unsigned int>(index.vertex_id + index.submap_id * sl);
  }
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_GLOBAL_VERTEX_ID_H_
