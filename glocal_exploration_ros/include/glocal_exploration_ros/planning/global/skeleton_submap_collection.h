#ifndef GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_SUBMAP_COLLECTION_H_
#define GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_SUBMAP_COLLECTION_H_

#include <list>
#include <map>
#include <memory>

#include "glocal_exploration_ros/planning/global/skeleton_submap.h"

namespace glocal_exploration {
class SkeletonSubmapCollection {
 public:
  void addSubmap(voxgraph::VoxgraphSubmap::ConstPtr submap_ptr,
                 const float traversability_radius) {
    CHECK_NOTNULL(submap_ptr);
    skeleton_submaps_.emplace(
        submap_ptr->getID(),
        std::make_shared<SkeletonSubmap>(submap_ptr, traversability_radius));
  }

  SkeletonSubmap::ConstPtr getSubmapConstPtrById(
      const voxgraph::SubmapID submap_id) const {
    auto it = skeleton_submaps_.find(submap_id);
    if (it == skeleton_submaps_.end()) {
      return nullptr;
    } else {
      return it->second;
    }
  }

  std::list<SkeletonSubmap::ConstPtr> getSubmapConstPtrs() const {
    std::list<SkeletonSubmap::ConstPtr> submap_list;
    for (const auto& submap_kv : skeleton_submaps_) {
      submap_list.emplace_back(submap_kv.second);
    }
    return submap_list;
  }

 private:
  std::map<voxgraph::SubmapID, SkeletonSubmap::Ptr> skeleton_submaps_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_ROS_PLANNING_GLOBAL_SKELETON_SUBMAP_COLLECTION_H_
