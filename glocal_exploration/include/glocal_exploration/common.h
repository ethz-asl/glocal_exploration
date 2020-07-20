#ifndef GLOCAL_EXPLORATION_COMMON_H_
#define GLOCAL_EXPLORATION_COMMON_H_

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Geometry>

namespace glocal_exploration {

// floating point accuracy
typedef double FloatingPoint;

// Vector types
typedef Eigen::Matrix<FloatingPoint, 3, 1> Point;
typedef kindr::minimal::QuatTransformationTemplate<FloatingPoint>
    Transformation;

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_COMMON_H_
