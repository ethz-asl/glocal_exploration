#ifndef GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_SUBMAP_H_
#define GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_SUBMAP_H_

#include <memory>
#include <string>
#include <utility>

#include <voxblox_skeleton/skeleton.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <voxgraph/common.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>

namespace glocal_exploration {
class SkeletonSubmap {
 public:
  using Ptr = std::shared_ptr<SkeletonSubmap>;
  using ConstPtr = std::shared_ptr<const SkeletonSubmap>;

  SkeletonSubmap(voxgraph::VoxgraphSubmap::ConstPtr submap_ptr,
                 const float traversability_radius)
      : submap_ptr_(std::move(submap_ptr)),
        traversability_radius_(traversability_radius),
        skeleton_generator_(&submap_ptr_->getEsdfMap().getEsdfLayer()) {}

  const voxblox::SparseSkeletonGraph& getSkeletonGraph() const {
    if (skeleton_generator_.getSparseGraph().getVertexMap().empty()) {
      generateSkeleton();
    }

    return skeleton_generator_.getSparseGraph();
  }

  voxgraph::SubmapID getId() const { return submap_ptr_->getID(); }
  std::string getFrameId() const {
    return "submap_" + std::to_string(submap_ptr_->getID());
  }

 private:
  voxgraph::VoxgraphSubmap::ConstPtr submap_ptr_;

  const float traversability_radius_;
  mutable voxblox::SkeletonGenerator skeleton_generator_;
  void generateSkeleton() const {
    CHECK_NOTNULL(submap_ptr_);
    LOG(INFO) << "Generating skeleton for submap: " << submap_ptr_->getID();
    skeleton_generator_.setMinGvdDistance(traversability_radius_);
    skeleton_generator_.setGenerateByLayerNeighbors(true);
    skeleton_generator_.generateSkeleton();
    skeleton_generator_.generateSparseGraph();
  }
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_SUBMAP_H_
