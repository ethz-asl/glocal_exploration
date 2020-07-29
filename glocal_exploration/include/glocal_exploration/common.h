#ifndef GLOCAL_EXPLORATION_COMMON_H_
#define GLOCAL_EXPLORATION_COMMON_H_

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Geometry>

namespace glocal_exploration {

// floating point accuracy
using FloatingPoint = double;

// Vector types
using Point = Eigen::Matrix<FloatingPoint, 3, 1>;
using Transformation =
    kindr::minimal::QuatTransformationTemplate<FloatingPoint>;

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_COMMON_H_
