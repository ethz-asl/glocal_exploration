#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_A_STAR_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_A_STAR_H_

#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <cblox/core/tsdf_esdf_submap.h>

#include "glocal_exploration/planning/global/skeleton/global_vertex_id.h"
#include "glocal_exploration/planning/global/skeleton/skeleton_submap_collection.h"
#include "glocal_exploration/state/communicator.h"
#include "glocal_exploration/state/waypoint.h"

namespace glocal_exploration {
class SkeletonAStar {
 public:
  struct Config : public config_utilities::Config<Config> {
    FloatingPoint traversability_radius = 1.1f;
    int max_num_start_vertex_candidates = 5;
    int max_num_end_vertex_candidates = 30;
    int linking_num_nearest_neighbors = 3;
    FloatingPoint linking_max_distance = 2.f;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  SkeletonAStar(const Config& config,
                std::shared_ptr<Communicator> communicator)
      : config_(config.checkValid()), comm_(std::move(communicator)) {}

  bool planPath(const Point& start_point, const Point& goal_point,
                std::vector<WayPoint>* way_points);

  void addSubmap(cblox::TsdfEsdfSubmap::ConstPtr submap_ptr,
                 const float traversability_radius) {
    skeleton_submap_collection_.addSubmap(std::move(submap_ptr),
                                          traversability_radius);
  }
  const SkeletonSubmapCollection& getSkeletonSubmapCollection() {
    return skeleton_submap_collection_;
  }

  FloatingPoint getTraversabilityRadius() {
    return config_.traversability_radius;
  }

 protected:
  const Config config_;

  std::shared_ptr<Communicator> comm_;
  std::shared_ptr<MapBase> map_;
  SkeletonSubmapCollection skeleton_submap_collection_;

  const GlobalVertexId kGoalVertexId{-1u, -1u};

  static constexpr size_t kMaxNumAStarIterations = 5e3;

  bool getPathBetweenVertices(
      const std::vector<GlobalVertexId>& start_vertex,
      const std::vector<GlobalVertexId>& end_vertex_candidates,
      const voxblox::Point& start_point, const voxblox::Point& goal_point,
      std::vector<GlobalVertexId>* vertex_path) const;
  static void getSolutionVertexPath(
      GlobalVertexId end_vertex_id,
      const std::map<GlobalVertexId, GlobalVertexId>& parent_map,
      std::vector<GlobalVertexId>* vertex_path);
  void convertVertexToWaypointPath(
      const std::vector<GlobalVertexId>& vertex_path, const Point& goal_point,
      std::vector<WayPoint>* way_points) const;

  static GlobalVertexId popSmallestFromOpen(
      const std::map<GlobalVertexId, FloatingPoint>& f_score_map,
      std::set<GlobalVertexId>* open_set);

  std::vector<GlobalVertexId> searchNClosestReachableSkeletonVertices(
      const Point& point, const int n_closest,
      const std::function<bool(const Point& start_point,
                               const Point& end_point)>&
          traversability_function) const;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_A_STAR_H_
