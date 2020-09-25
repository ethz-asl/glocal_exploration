#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_SUBMAP_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_SUBMAP_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cblox/core/tsdf_esdf_submap.h>
#include <voxblox_skeleton/skeleton.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <voxblox_skeleton/sparse_graph_planner.h>

namespace glocal_exploration {
class SkeletonSubmap {
 public:
  using Ptr = std::shared_ptr<SkeletonSubmap>;
  using ConstPtr = std::shared_ptr<const SkeletonSubmap>;

  SkeletonSubmap(cblox::TsdfEsdfSubmap::ConstPtr submap_ptr,
                 const float traversability_radius)
      : submap_ptr_(CHECK_NOTNULL(submap_ptr)),
        traversability_radius_(traversability_radius),
        graph_(generateSparseSkeletonGraph(*submap_ptr, traversability_radius)),
        kd_tree_adapter_(graph_.getVertexMap()),
        kd_tree_(
            kDTreeDim, kd_tree_adapter_,
            voxblox::nanoflann::KDTreeSingleIndexAdaptorParams(kDTreeMaxLeaf)) {
    kd_tree_.buildIndex();
  }

  const voxblox::SparseSkeletonGraph& getSkeletonGraph() const {
    return graph_;
  }
  size_t getNClosestVertices(const voxblox::Point& point, int num_vertices,
                             std::vector<int64_t>* vertex_inds) const {
    CHECK_NOTNULL(vertex_inds);
    vertex_inds->clear();
    vertex_inds->reserve(num_vertices);

    std::vector<size_t> ret_index(num_vertices);
    std::vector<FloatingPoint> out_dist_sqr(num_vertices);

    voxblox::nanoflann::SearchParams params;  // Defaults are fine.
    size_t num_results = kd_tree_.knnSearch(point.data(), num_vertices,
                                            &ret_index[0], &out_dist_sqr[0]);
    for (size_t i = 0; i < num_results; i++) {
      vertex_inds->push_back(ret_index[i]);
    }
    return num_results;
  }

  SubmapId getId() const { return submap_ptr_->getID(); }
  std::string getFrameId() const {
    return "submap_" + std::to_string(submap_ptr_->getID());
  }

  Transformation getPose() const { return submap_ptr_->getPose(); }

 private:
  cblox::TsdfEsdfSubmap::ConstPtr submap_ptr_;
  const float traversability_radius_;

  voxblox::SparseSkeletonGraph graph_;
  static voxblox::SparseSkeletonGraph generateSparseSkeletonGraph(
      const cblox::TsdfEsdfSubmap& submap, const float traversability_radius) {
    voxblox::SkeletonGenerator skeleton_generator(
        &submap.getEsdfMap().getEsdfLayer());
    LOG(INFO) << "Generating skeleton for submap: " << submap.getID();
    skeleton_generator.setMinGvdDistance(traversability_radius);
    skeleton_generator.setGenerateByLayerNeighbors(true);
    skeleton_generator.generateSkeleton();
    skeleton_generator.generateSparseGraph();
    return skeleton_generator.getSparseGraph();
  }

  const int kDTreeDim = 3;
  const int kDTreeMaxLeaf = 10;
  voxblox::DirectSkeletonVertexMapAdapter kd_tree_adapter_;
  voxblox::SparseGraphPlanner::VertexGraphKdTree kd_tree_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_SUBMAP_H_
