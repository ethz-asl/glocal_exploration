#ifndef GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXBLOX_SERVER_H_
#define GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXBLOX_SERVER_H_

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <minkindr_conversions/kindr_msg.h>
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
  std::shared_ptr<voxblox::EsdfMap> getEsdfMapPtr() override {
    return safe_esdf_map_;
  }
  std::shared_ptr<const voxblox::EsdfMap> getEsdfMapPtr() const override {
    return safe_esdf_map_;
  }

  void updateEsdf() override {
    voxblox::EsdfServer::updateEsdf();
    *safe_esdf_map_->getEsdfLayerPtr() = esdf_map_->getEsdfLayer();

    // Call the external callback, if it has been set
    if (external_new_esdf_callback_) {
      external_new_esdf_callback_();
    }
  }
  void updateEsdfBatch(bool full_euclidean = false) override {
    voxblox::EsdfServer::updateEsdfBatch();
    *safe_esdf_map_->getEsdfLayerPtr() = esdf_map_->getEsdfLayer();

    // Call the external callback, if it has been set
    if (external_new_esdf_callback_) {
      external_new_esdf_callback_();
    }
  }

  void newPoseCallback(const voxblox::Transformation& T_G_C) override {
    voxblox::EsdfServer::newPoseCallback(T_G_C);

    // Call the external callback, if it has been set
    if (external_new_pose_callback_) {
      external_new_pose_callback_();
    }
  }

  void setExternalNewPoseCallback(Function callback) {
    external_new_pose_callback_ = std::move(callback);
  }
  void setExternalNewEsdfCallback(Function callback) {
    external_new_esdf_callback_ = std::move(callback);
  }

  std::vector<geometry_msgs::PoseStamped> getPoseHistory() {
    std::vector<geometry_msgs::PoseStamped> pose_history;
    for (const auto& item : pointcloud_deintegration_queue_) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = item.timestamp;
      tf::poseKindrToMsg(item.T_G_C.cast<double>(), &pose_msg.pose);
      pose_history.emplace_back(pose_msg);
    }
    return pose_history;
  }

 protected:
  voxblox::EsdfMap::Ptr safe_esdf_map_;

  Function external_new_pose_callback_;
  Function external_new_esdf_callback_;

  ros::CallbackQueue callback_queue_;
  ros::AsyncSpinner spinner_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_MAPPING_THREADSAFE_WRAPPERS_THREADSAFE_VOXBLOX_SERVER_H_
