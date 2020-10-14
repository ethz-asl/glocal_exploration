#ifndef GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_A_STAR_H_
#define GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_A_STAR_H_

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <cblox/core/tsdf_esdf_submap.h>

#include "glocal_exploration/planning/global/skeleton/global_vertex_id.h"
#include "glocal_exploration/planning/global/skeleton/relative_waypoint.h"
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
    int linking_max_num_submaps = 20;
    int linking_max_num_links = 30;
    FloatingPoint linking_max_distance = 2.f;
    int max_num_a_star_iterations = 5e3;

    Config();
    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
  };

  struct VisualizationEdges {
    std::map<GlobalVertexId, GlobalVertexId> parent_map_;
    std::map<GlobalVertexId, GlobalVertexId> intraversable_edge_map_;
    void clear() {
      parent_map_.clear();
      intraversable_edge_map_.clear();
    }
  };

  SkeletonAStar(const Config& config,
                std::shared_ptr<Communicator> communicator)
      : config_(config.checkValid()), comm_(std::move(communicator)) {}

  const Config& getConfig() const { return config_; }
  FloatingPoint getTraversabilityRadius() const {
    return config_.traversability_radius;
  }

  bool planPath(const Point& start_point, const Point& goal_point,
                std::vector<RelativeWayPoint>* way_points);

  std::vector<GlobalVertexId> searchClosestReachableSkeletonVertices(
      const Point& point, const int n_closest,
      const std::function<bool(const Point& point,
                               const Point& skeleton_vertex_point)>&
          traversability_function) const;
  bool getPathBetweenVertices(
      const std::vector<GlobalVertexId>& start_vertex,
      const std::vector<GlobalVertexId>& end_vertex_candidates,
      const voxblox::Point& start_point, const voxblox::Point& goal_point,
      std::vector<GlobalVertexId>* vertex_path) const;
  void convertVertexToWaypointPath(
      const std::vector<GlobalVertexId>& vertex_path, const Point& goal_point,
      std::vector<RelativeWayPoint>* way_points) const;

  const SkeletonSubmapCollection& getSkeletonSubmapCollection() const {
    return skeleton_submap_collection_;
  }
  void addSubmap(cblox::TsdfEsdfSubmap::ConstPtr submap_ptr,
                 const float traversability_radius) {
    skeleton_submap_collection_.addSubmap(std::move(submap_ptr),
                                          traversability_radius);
  }

  VisualizationEdges getVisualizationEdges() const {
    std::lock_guard<std::mutex> lock_guard(visualization_data_mutex_);
    return visualization_edges_;
  }

 protected:
  const Config config_;

  std::shared_ptr<Communicator> comm_;
  SkeletonSubmapCollection skeleton_submap_collection_;

  const GlobalVertexId kGoalVertexId{RelativeWayPoint::kOdomFrameId, -1u};

  static void getSolutionVertexPath(
      GlobalVertexId end_vertex_id,
      const std::map<GlobalVertexId, GlobalVertexId>& parent_map,
      std::vector<GlobalVertexId>* vertex_path);

  static GlobalVertexId popSmallestFromOpen(
      const std::map<GlobalVertexId, FloatingPoint>& f_score_map,
      std::set<GlobalVertexId>* open_set);

  // Persistent only for visualization purposes.
  // Only used within getPathBetweenVertices().
  mutable VisualizationEdges visualization_edges_;
  mutable std::mutex visualization_data_mutex_;
};
}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_GLOBAL_SKELETON_SKELETON_A_STAR_H_
