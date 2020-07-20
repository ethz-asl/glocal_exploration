#ifndef GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_NODE_HANDLES_H_
#define GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_NODE_HANDLES_H_

namespace glocal_exploration {
inline ros::NodeHandle changeNodeHandleCallbackQueue(
    const ros::NodeHandle& nh, ros::CallbackQueue* callback_queue_ptr) {
  ros::NodeHandle changed_nh(nh);
  changed_nh.setCallbackQueue(callback_queue_ptr);
  return changed_nh;
}
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_CONVERSIONS_ROS_NODE_HANDLES_H_
