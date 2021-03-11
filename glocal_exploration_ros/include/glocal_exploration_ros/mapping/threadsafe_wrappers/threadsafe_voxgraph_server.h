#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXGRAPH_SERVER_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXGRAPH_SERVER_H_

#include <utility>

#include <voxgraph/frontend/voxgraph_mapper.h>

#include "glocal_exploration_ros/conversions/ros_node_handles.h"

namespace glocal_exploration {
class ThreadsafeVoxgraphServer : public voxgraph::VoxgraphMapper {
 public:
  using Function = std::function<void()>;

  template <typename... Args>
  ThreadsafeVoxgraphServer(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private, Args&&... args)
      : voxgraph::VoxgraphMapper(
            changeNodeHandleCallbackQueue(nh, &callback_queue_),
            changeNodeHandleCallbackQueue(nh_private, &callback_queue_),
            std::forward<Args>(args)...),
        spinner_(1, &callback_queue_) {
    // Start processing callbacks
    spinner_.start();
  }

  bool submapCallback(
      const voxblox_msgs::LayerWithTrajectory& submap_msg) override {
    const bool submap_added_successfully =
        voxgraph::VoxgraphMapper::submapCallback(submap_msg);

    // Call the external callback, if it has been set
    if (submap_added_successfully && external_new_submap_callback_) {
      external_new_submap_callback_();
    }

    return submap_added_successfully;
  }

  void setExternalNewSubmapCallback(Function callback) {
    external_new_submap_callback_ = std::move(callback);
  }

 protected:
  Function external_new_submap_callback_;

  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXGRAPH_SERVER_H_
