#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_SUBMAP_COLLECTION_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_SUBMAP_COLLECTION_H_

#include <list>
#include <map>
#include <memory>

#include "glocal_exploration/planning/global/skeleton/skeleton_submap.h"

namespace glocal_exploration {
class SkeletonSubmapCollection {
 public:
  void addSubmap(cblox::TsdfEsdfSubmap::ConstPtr submap_ptr,
                 const float traversability_radius) {
    CHECK_NOTNULL(submap_ptr);
    skeleton_submaps_.emplace(
        submap_ptr->getID(),
        std::make_shared<SkeletonSubmap>(submap_ptr, traversability_radius));
  }

  const SkeletonSubmap& getSubmapById(const SubmapId submap_id) const {
    auto it = skeleton_submaps_.find(submap_id);
    CHECK(it != skeleton_submaps_.end())
        << "Could not find skeleton submap with ID " << submap_id;
    return *it->second;
  }

  SkeletonSubmap::ConstPtr getSubmapConstPtrById(
      const SubmapId submap_id) const {
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
  std::map<SubmapId, SkeletonSubmap::Ptr> skeleton_submaps_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_SUBMAP_COLLECTION_H_
