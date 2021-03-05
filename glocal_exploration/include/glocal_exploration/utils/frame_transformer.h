#ifndef GLOCAL_EXPLORATION_UTILS_FRAME_TRANSFORMER_H_
#define GLOCAL_EXPLORATION_UTILS_FRAME_TRANSFORMER_H_

#include <string>

#include "glocal_exploration/common.h"

namespace glocal_exploration {
class FrameTransformer {
 public:
  explicit FrameTransformer(const std::string& fixed_frame_id)
      : fixed_frame_id_(fixed_frame_id) {}

  void update(const Transformation& T_O_F) { T_F_O_ = T_O_F.inverse(); }

  Point transformFromOdomToFixedFrame(const Point& t_O_position) const {
    return T_F_O_ * t_O_position;
  }
  Transformation transformFromOdomToFixedFrame(
      const Transformation& T_O_i) const {
    return T_F_O_ * T_O_i;
  }

  const std::string& getFixedFrameId() { return fixed_frame_id_; }

 private:
  Transformation T_F_O_;
  const std::string fixed_frame_id_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_UTILS_FRAME_TRANSFORMER_H_
