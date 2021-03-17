#ifndef GLOCAL_EXPLORATION_COMMON_H_
#define GLOCAL_EXPLORATION_COMMON_H_

#include <Eigen/Geometry>
#include <glog/logging.h>
#include <kindr/minimal/quat-transformation.h>
#include <voxblox/core/common.h>

namespace glocal_exploration {

// floating point accuracy
using FloatingPoint = voxblox::FloatingPoint;

// Vector types
using Point = voxblox::Point;
using Transformation = voxblox::Transformation;

// Submapping related types
using SubmapId = unsigned int;  // NOTE: This must match cblox's SubmapID type

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_COMMON_H_
