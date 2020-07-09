#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXGRAPH_SERVER_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXGRAPH_SERVER_H_

#include <utility>

#include <voxgraph/frontend/voxgraph_mapper.h>

#include "glocal_exploration_ros/conversions/ros_node_handles.h"

namespace glocal_exploration {
class ThreadsafeVoxgraphServer : public voxgraph::VoxgraphMapper {
 public:
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

  // TODO(victorr): Add the local area update here

 protected:
  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXGRAPH_SERVER_H_
