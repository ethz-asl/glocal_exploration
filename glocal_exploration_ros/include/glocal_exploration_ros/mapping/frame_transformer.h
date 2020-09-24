#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_FRAME_TRANSFORMER_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_FRAME_TRANSFORMER_H_

#include <string>

#include <glocal_exploration/common.h>
#include <kindr/minimal/quat-transformation.h>
#include <voxblox/core/common.h>

namespace glocal_exploration {
class FrameTransformer {
 public:
  explicit FrameTransformer(const std::string& fixed_frame_id)
      : fixed_frame_id_(fixed_frame_id) {}

  template <typename InputFloatingPointType>
  void update(
      const kindr::minimal::QuatTransformationTemplate<InputFloatingPointType>&
          T_O_F) {
    T_F_O_ = T_O_F.inverse().template cast<FloatingPoint>();
  }

  template <typename InputFloatingPointType>
  voxblox::Point transformFromOdomToFixedFrame(
      const Eigen::Matrix<InputFloatingPointType, 3, 1>& t_O_position) const {
    return T_F_O_.cast<voxblox::FloatingPoint>() *
           t_O_position.template cast<voxblox::FloatingPoint>();
  }
  template <typename InputFloatingPointType>
  voxblox::Transformation transformFromOdomToFixedFrame(
      const kindr::minimal::QuatTransformationTemplate<InputFloatingPointType>&
          T_O_i) const {
    return T_F_O_.cast<voxblox::FloatingPoint>() *
           T_O_i.template cast<voxblox::FloatingPoint>();
  }

  const std::string& getFixedFrameId() { return fixed_frame_id_; }

 private:
  Transformation T_F_O_;
  const std::string fixed_frame_id_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_FRAME_TRANSFORMER_H_
