#ifndef GLOCAL_EXPLORATION_STATE_REGION_OF_INTEREST_H_
#define GLOCAL_EXPLORATION_STATE_REGION_OF_INTEREST_H_

#include "glocal_exploration/common.h"

namespace glocal_exploration {

/**
 * Computes whether a point is in a region of interest.
 */
class RegionOfInterest {
 public:
  RegionOfInterest() = default;
  virtual ~RegionOfInterest() = default;

  virtual bool contains(const Eigen::Vector3d& point) = 0;
};

/**
 * Simple Bounding Box implementation
 */
class BoundingBox : public RegionOfInterest {
 public:
  struct Config {
    double x_min = 0.0;
    double y_min = 0.0;
    double z_min = 0.0;
    double x_max = 0.0;
    double y_max = 0.0;
    double z_max = 0.0;

    bool isValid() const;
    Config checkValid() const;
  };

  explicit BoundingBox(const Config& config);
  virtual ~BoundingBox() = default;

  bool contains(const Eigen::Vector3d& point) override;

 protected:
  const Config config_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_STATE_REGION_OF_INTEREST_H_
