#ifndef GLOCAL_EXPLORATION_MAPPING_NEIGHBORHOOD_OFFSETS_H_
#define GLOCAL_EXPLORATION_MAPPING_NEIGHBORHOOD_OFFSETS_H_

#include <algorithm>
#include <vector>

#include "glocal_exploration/common.h"

namespace glocal_exploration {

struct NeighborhoodOffsets {
  NeighborhoodOffsets() {
    for (int x = -1; x <= 1; ++x) {
      for (int y = -1; y <= 1; ++y) {
        for (int z = -1; z <= 1; ++z) {
          const Point offset(x, y, z);
          if (!offset.isZero()) {
            vertices_.emplace_back(offset);
          }
        }
      }
    }
    std::sort(vertices_.begin(), vertices_.end(),
              [](const Point& lhs, const Point& rhs) {
                return lhs.norm() < rhs.norm();
              });
  }

  const std::vector<Point>& getVertices() const { return vertices_; }

 protected:
  std::vector<Point> vertices_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_MAPPING_NEIGHBORHOOD_OFFSETS_H_
