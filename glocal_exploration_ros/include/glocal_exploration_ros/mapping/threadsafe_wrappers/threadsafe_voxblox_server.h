#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXBLOX_SERVER_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXBLOX_SERVER_H_

#include <functional>
#include <utility>

#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/ros_params.h>

#include "glocal_exploration_ros/conversions/ros_node_handles.h"

namespace glocal_exploration {
class ThreadsafeVoxbloxServer : public voxblox::EsdfServer {
 public:
  using Function = std::function<void()>;

  template <typename... Args>
  ThreadsafeVoxbloxServer(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private, Args&&... args)
      : voxblox::EsdfServer(
            changeNodeHandleCallbackQueue(nh, &callback_queue_),
            changeNodeHandleCallbackQueue(nh_private, &callback_queue_),
            std::forward<Args>(args)...),
        spinner_(1, &callback_queue_) {
    // Set up the thread-safe ESDF map copy
    safe_esdf_map_.reset(new voxblox::EsdfMap(
        voxblox::getEsdfMapConfigFromRosParam(nh_private_)));
    // Start processing callbacks
    spinner_.start();
  }

  // TODO(victorr): Also make sure all other thread-unsafe base class methods
  //                are no longer accessible, and see if there's a cleaner
  //                alternative to base method hiding.
  inline voxblox::EsdfMap::Ptr getEsdfMapPtr() { return safe_esdf_map_; }

  void newPoseCallback(const voxblox::Transformation& T_G_C) override {
    // TODO(victorr): Block getEsdfMapPtr() during this method, preferably
    //                without a mutex though since that method gets called at a
    //                very high rate.
    *safe_esdf_map_->getEsdfLayerPtr() = esdf_map_->getEsdfLayer();
    voxblox::EsdfServer::newPoseCallback(T_G_C);

    external_new_pose_callback_();
  }

  void setExternalNewPoseCallback(Function callback) {
    external_new_pose_callback_ = callback;
  }

 protected:
  voxblox::EsdfMap::Ptr safe_esdf_map_;

  Function external_new_pose_callback_;

  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXBLOX_SERVER_H_
