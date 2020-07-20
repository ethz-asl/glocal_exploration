#ifndef GLOCAL_EXPLORATION_COMMON_H_
#define GLOCAL_EXPLORATION_COMMON_H_

#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Geometry>

namespace glocal_exploration {

// transformations
using Transformation = kindr::minimal::QuatTransformationTemplate<double>;

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_COMMON_H_
