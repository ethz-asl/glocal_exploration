#include "glocal_exploration/planning/global/skeleton/skeleton_a_star.h"

#include <limits>
#include <list>
#include <map>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

#include "glocal_exploration/utils/execute_on_scope_exit.h"

namespace glocal_exploration {

SkeletonAStar::Config::Config() { setConfigName("SkeletonAStarPlanner"); }

void SkeletonAStar::Config::checkParams() const {
  checkParamGE(traversability_radius, 0.f, "traversability_radius");
  checkParamGT(max_num_start_vertex_candidates, 0,
               "max_num_start_vertex_candidates");
  checkParamGT(max_num_end_vertex_candidates, 0,
               "max_num_end_vertex_candidates");
  checkParamGT(linking_num_nearest_neighbors, 0,
               "linking_num_nearest_neighbors");
  checkParamGT(linking_max_num_submaps, 0, "linking_max_num_submaps");
  checkParamGT(linking_max_distance, 0.f, "linking_max_distance");
  checkParamGT(max_num_a_star_iterations, 0, "max_num_a_star_iterations");
}

void SkeletonAStar::Config::fromRosParam() {
  rosParam("traversability_radius", &traversability_radius);
  rosParam("max_num_start_vertex_candidates", &max_num_start_vertex_candidates);
  rosParam("max_num_end_vertex_candidates", &max_num_end_vertex_candidates);
  rosParam("linking_num_nearest_neighbors", &linking_num_nearest_neighbors);
  rosParam("linking_max_num_submaps", &linking_max_num_submaps);
  rosParam("linking_max_distance", &linking_max_distance);
  rosParam("max_num_a_star_iterations", &max_num_a_star_iterations);
}

void SkeletonAStar::Config::printFields() const {
  printField("traversability_radius", traversability_radius);
  printField("max_num_start_vertex_candidates",
             max_num_start_vertex_candidates);
  printField("max_num_end_vertex_candidates", max_num_end_vertex_candidates);
  printField("linking_num_nearest_neighbors", linking_num_nearest_neighbors);
  printField("linking_max_num_submaps", linking_max_num_submaps);
  printField("linking_max_distance", linking_max_distance);
  printField("max_num_a_star_iterations", max_num_a_star_iterations);
}

bool SkeletonAStar::planPath(const Point& start_point, const Point& goal_point,
                             std::vector<RelativeWayPoint>* way_points) {
  // Search the nearest reachable start vertex on the skeleton graphs
  if (!comm_->map()->isTraversableInActiveSubmap(
          start_point, config_.traversability_radius)) {
    LOG(INFO) << "Start point is not traversable in active submap ("
              << start_point.transpose()
              << "). Will not be able to connect to skeleton graphs.";
    return false;
  }
  const std::vector<GlobalVertexId> start_vertex_candidates =
      searchClosestReachableSkeletonVertices(
          start_point, config_.max_num_start_vertex_candidates,
          [this](const Point& start_point, const Point& end_point) {
            return comm_->map()->isLineTraversableInActiveSubmap(
                start_point, end_point, config_.traversability_radius);
          });
  if (start_vertex_candidates.empty()) {
    LOG(INFO)
        << "Could not find any reachable skeleton vertices near start point ("
        << start_point.transpose() << ")";
    return false;
  }

  // Search the N closest reachable end vertices on the skeleton graph
  if (!comm_->map()->isTraversableInGlobalMap(goal_point,
                                              config_.traversability_radius)) {
    LOG(INFO) << "Goal point is not traversable in global map ("
              << goal_point.transpose()
              << "). Will not be able to connect to skeleton graphs.";
    return false;
  }
  std::vector<GlobalVertexId> end_vertex_candidates =
      searchClosestReachableSkeletonVertices(
          goal_point, config_.max_num_end_vertex_candidates,
          [this](const Point& start_point, const Point& end_point) {
            return comm_->map()->isLineTraversableInGlobalMap(
                start_point, end_point, config_.traversability_radius);
          });
  if (end_vertex_candidates.empty()) {
    LOG(INFO)
        << "Could not find any reachable skeleton vertices near goal point ("
        << goal_point.transpose() << ")";
    return false;
  }

  // Plan path along the skeleton
  std::vector<GlobalVertexId> vertex_path;
  if (!getPathBetweenVertices(start_vertex_candidates, end_vertex_candidates,
                              start_point, goal_point, &vertex_path)) {
    LOG(INFO) << "Could not find global path from start point ("
              << start_point.transpose() << ") to goal point ("
              << goal_point.transpose() << ")";
    return false;
  }

  // Convert the path from vertex IDs to waypoints
  convertVertexToWaypointPath(vertex_path, goal_point, way_points);

  return true;
}

std::vector<GlobalVertexId>
SkeletonAStar::searchClosestReachableSkeletonVertices(
    const Point& point, const int n_closest,
    const std::function<bool(const Point& point,
                             const Point& skeleton_vertex_point)>&
        traversability_function) const {
  struct CandidateVertex {
    GlobalVertexId global_vertex_id;
    Point t_O_point = Point::Zero();
    float distance = -1.f;
  };
  std::list<CandidateVertex> candidate_start_vertices;
  for (const SubmapId submap_id : comm_->map()->getSubmapIdsAtPosition(point)) {
    SkeletonSubmap::ConstPtr skeleton_submap =
        skeleton_submap_collection_.getSubmapConstPtrById(submap_id);
    if (!skeleton_submap) {
      LOG(ERROR) << "Couldn't get pointer to skeleton submap with ID "
                 << submap_id;
      continue;
    }
    for (const auto& vertex_kv :
         skeleton_submap->getSkeletonGraph().getVertexMap()) {
      CandidateVertex candidate_vertex;
      candidate_vertex.global_vertex_id.submap_id = submap_id;
      candidate_vertex.global_vertex_id.vertex_id = vertex_kv.second.vertex_id;
      candidate_vertex.t_O_point =
          skeleton_submap->getPose() * vertex_kv.second.point;
      candidate_vertex.distance = (candidate_vertex.t_O_point - point).norm();
      candidate_start_vertices.emplace_back(std::move(candidate_vertex));
    }
  }

  candidate_start_vertices.sort(
      [](const CandidateVertex& lhs, const CandidateVertex& rhs) {
        return lhs.distance < rhs.distance;
      });

  int num_candidates_unreachable = 0;
  std::vector<GlobalVertexId> reachable_skeleton_vertices;
  for (const CandidateVertex& candidate_start_vertex :
       candidate_start_vertices) {
    if (traversability_function(point, candidate_start_vertex.t_O_point)) {
      reachable_skeleton_vertices.emplace_back(
          candidate_start_vertex.global_vertex_id);
      if (n_closest <= reachable_skeleton_vertices.size()) {
        break;
      }
    } else {
      ++num_candidates_unreachable;
    }
  }

  return reachable_skeleton_vertices;
}

bool SkeletonAStar::getPathBetweenVertices(
    const std::vector<GlobalVertexId>& start_vertex_candidates,
    const std::vector<GlobalVertexId>& end_vertex_candidates,
    const voxblox::Point& start_point, const voxblox::Point& goal_point,
    std::vector<GlobalVertexId>* vertex_path) const {
  CHECK_NOTNULL(vertex_path);
  CHECK(!start_vertex_candidates.empty());
  CHECK(!end_vertex_candidates.empty());

  std::map<GlobalVertexId, FloatingPoint> g_score_map;
  std::map<GlobalVertexId, FloatingPoint> f_score_map;
  std::map<GlobalVertexId, GlobalVertexId> parent_map;

  std::set<GlobalVertexId> open_set;
  std::set<GlobalVertexId> closed_set;

  // Auto copy the visualization data once this method returns.
  // NOTE: We copy them s.t. it's safe to read them from a separate thread.
  std::map<GlobalVertexId, GlobalVertexId> intraversable_edge_map;
  ExecuteOnScopeExit auto_copy_visuals([&]() {
    std::lock_guard<std::mutex> lock_guard(visualization_data_mutex_);
    visualization_edges_.parent_map_ = parent_map;
    visualization_edges_.intraversable_edge_map_ = intraversable_edge_map;
  });

  // Initialize the search with vertices that can be used as graph entry points
  // i.e. vertices that are closest to the start_point and reachable
  for (const GlobalVertexId& current_vertex_id : start_vertex_candidates) {
    const SkeletonSubmap& current_submap =
        skeleton_submap_collection_.getSubmapById(current_vertex_id.submap_id);
    const voxblox::SparseSkeletonGraph& current_graph =
        current_submap.getSkeletonGraph();
    const voxblox::SkeletonVertex& current_vertex =
        current_graph.getVertex(current_vertex_id.vertex_id);

    const voxblox::Point t_odom_current_vertex =
        current_submap.getPose() * current_vertex.point;
    g_score_map[current_vertex_id] =
        (t_odom_current_vertex - start_point).norm();
    f_score_map[current_vertex_id] =
        (goal_point - t_odom_current_vertex).norm();
    open_set.insert(current_vertex_id);
  }

  // Indicate which vertices can be used as graph exit points
  // i.e. vertices that are close to the end point and that can reach it
  std::unordered_set<GlobalVertexId, GlobalVertexIdHash>
      end_vertex_candidate_set;
  for (const GlobalVertexId& end_vertex_candidate : end_vertex_candidates) {
    end_vertex_candidate_set.insert(end_vertex_candidate);
  }

  // Run the Astar search
  size_t iteration_counter = 0u;
  SubmapId previous_submap_id = -1;
  const SkeletonSubmap* current_submap = nullptr;
  const voxblox::SparseSkeletonGraph* current_graph = nullptr;
  while (!open_set.empty()) {
    if (config_.max_num_a_star_iterations <= ++iteration_counter) {
      LOG(WARNING) << "Aborting skeleton planning. Exceeded maximum number of "
                      "iterations ("
                   << iteration_counter << ").";
      return false;
    }

    // Find the smallest f-value in the open set.
    const GlobalVertexId current_vertex_id =
        popSmallestFromOpen(f_score_map, &open_set);

    // Check if we have reached the goal
    if (current_vertex_id == kGoalVertexId) {
      LOG(INFO) << "Found skeleton path to goal in " << iteration_counter
                << " iterations.";
      getSolutionVertexPath(kGoalVertexId, parent_map, vertex_path);
      return true;
    }

    // Get vertex's submap and graph
    if (current_vertex_id.submap_id != previous_submap_id) {
      current_submap = &skeleton_submap_collection_.getSubmapById(
          current_vertex_id.submap_id);
      current_graph = &current_submap->getSkeletonGraph();
    }
    previous_submap_id = current_vertex_id.submap_id;
    closed_set.insert(current_vertex_id);

    // If this vertex is an exit point candidate,
    // hallucinate an edge to the goal
    const voxblox::SkeletonVertex& current_vertex =
        current_graph->getVertex(current_vertex_id.vertex_id);
    if (end_vertex_candidate_set.count(current_vertex_id)) {
      //      LOG(WARNING) << "Found exit point candidate vertex";
      if (open_set.count(kGoalVertexId) == 0) {
        open_set.insert(kGoalVertexId);
      }
      FloatingPoint tentative_g_score =
          g_score_map[current_vertex_id] +
          (goal_point - current_vertex.point).norm();
      if (g_score_map.count(kGoalVertexId) == 0 ||
          g_score_map[kGoalVertexId] < tentative_g_score) {
        g_score_map[kGoalVertexId] = tentative_g_score;
        f_score_map[kGoalVertexId] = tentative_g_score;
        parent_map[kGoalVertexId] = current_vertex_id;
      }
      continue;
    }

    // Unless this vertex already has many neighbors, try to connect to a
    // neighboring skeleton submap
    const Point t_odom_current_vertex =
        current_submap->getPose() * current_vertex.point;
    if (current_vertex.edge_list.size() <= 3) {
      int num_linked_submaps = 0;
      int num_links_total = 0;
      for (const SubmapId submap_id :
           comm_->map()->getSubmapIdsAtPosition(t_odom_current_vertex)) {
        // Avoid linking the current vertex against vertices of its own submap
        if (submap_id == current_vertex_id.submap_id) {
          continue;
        }

        SkeletonSubmap::ConstPtr nearby_submap =
            skeleton_submap_collection_.getSubmapConstPtrById(submap_id);
        if (!nearby_submap) {
          continue;
        }

        // Limit the maximum number of submaps that can be linked to
        if (config_.linking_max_num_submaps < num_linked_submaps ||
            config_.linking_max_num_links < num_links_total) {
          break;
        }

        voxblox::Point t_nearby_submap_current_vertex =
            (nearby_submap->getPose().inverse() * t_odom_current_vertex);
        std::vector<VertexIdElement> nearest_vertex_ids;
        nearby_submap->getNClosestVertices(
            t_nearby_submap_current_vertex,
            config_.linking_num_nearest_neighbors, &nearest_vertex_ids);

        bool linked_submap = false;
        for (const VertexIdElement& nearby_vertex_id : nearest_vertex_ids) {
          const GlobalVertexId nearby_vertex_global_id{submap_id,
                                                       nearby_vertex_id};
          const Point t_odom_nearby_vertex =
              nearby_submap->getPose() * nearby_submap->getSkeletonGraph()
                                             .getVertex(nearby_vertex_id)
                                             .point;
          const float distance_current_to_nearby_vertex =
              (t_odom_current_vertex - t_odom_nearby_vertex).norm();
          if (distance_current_to_nearby_vertex <
              config_.linking_max_distance) {
            if (comm_->map()->isLineTraversableInGlobalMap(
                    t_odom_current_vertex, t_odom_nearby_vertex,
                    config_.traversability_radius)) {
              ++num_links_total;
              linked_submap = true;
              if (closed_set.count(nearby_vertex_global_id) > 0) {
                continue;
              }
              if (open_set.count(nearby_vertex_global_id) == 0) {
                open_set.insert(nearby_vertex_global_id);
              }

              FloatingPoint tentative_g_score =
                  g_score_map[current_vertex_id] +
                  (t_odom_nearby_vertex - t_odom_current_vertex).norm();
              if (g_score_map.count(nearby_vertex_global_id) == 0 ||
                  g_score_map[nearby_vertex_global_id] < tentative_g_score) {
                g_score_map[nearby_vertex_global_id] = tentative_g_score;
                f_score_map[nearby_vertex_global_id] =
                    tentative_g_score +
                    (goal_point - t_odom_nearby_vertex).norm();
                parent_map[nearby_vertex_global_id] = current_vertex_id;
              }
            } else {
              intraversable_edge_map[current_vertex_id] =
                  nearby_vertex_global_id;
            }
          }
        }
        if (linked_submap) {
          ++num_linked_submaps;
        }
      }
    }

    // Evaluate the vertex's neighbors
    for (int64_t edge_id : current_vertex.edge_list) {
      const voxblox::SkeletonEdge& edge = current_graph->getEdge(edge_id);
      GlobalVertexId neighbor_vertex_id = current_vertex_id;
      if (edge.start_vertex == current_vertex_id.vertex_id) {
        neighbor_vertex_id.vertex_id = edge.end_vertex;
      } else {
        neighbor_vertex_id.vertex_id = edge.start_vertex;
      }

      if (closed_set.count(neighbor_vertex_id) > 0) {
        // This neighbor has already been checked
        continue;
      }

      // Check if this neighbor is reachable from the current vertex
      const voxblox::SkeletonVertex& neighbor_vertex =
          current_graph->getVertex(neighbor_vertex_id.vertex_id);
      const voxblox::Point t_odom_neighbor_vertex =
          current_submap->getPose() * neighbor_vertex.point;
      if (!comm_->map()->isLineTraversableInGlobalMap(
              t_odom_current_vertex, t_odom_neighbor_vertex,
              config_.traversability_radius)) {
        intraversable_edge_map[current_vertex_id] = neighbor_vertex_id;
        continue;
      }

      if (open_set.count(neighbor_vertex_id) == 0) {
        open_set.insert(neighbor_vertex_id);
      }

      FloatingPoint tentative_g_score =
          g_score_map[current_vertex_id] +
          (neighbor_vertex.point - current_vertex.point).norm();
      // NOTE: Since the vertex and its neighbor are already in the same
      //       (submap) frame, we can directly compute their distance above
      if (g_score_map.count(neighbor_vertex_id) == 0 ||
          g_score_map[neighbor_vertex_id] < tentative_g_score) {
        g_score_map[neighbor_vertex_id] = tentative_g_score;
        f_score_map[neighbor_vertex_id] =
            tentative_g_score + (goal_point - t_odom_neighbor_vertex).norm();
        parent_map[neighbor_vertex_id] = current_vertex_id;
      }
    }
  }

  return false;
}

void SkeletonAStar::convertVertexToWaypointPath(
    const std::vector<GlobalVertexId>& vertex_path, const Point& goal_point,
    std::vector<RelativeWayPoint>* way_points) const {
  CHECK_NOTNULL(way_points);
  way_points->clear();
  for (const GlobalVertexId& global_vertex_id : vertex_path) {
    if (global_vertex_id == kGoalVertexId) {
      way_points->emplace_back(RelativeWayPoint(goal_point));
    } else {
      SkeletonSubmap::ConstPtr submap_ptr =
          skeleton_submap_collection_.getSubmapConstPtrById(
              global_vertex_id.submap_id);
      const voxblox::SkeletonVertex& vertex =
          submap_ptr->getSkeletonGraph().getVertex(global_vertex_id.vertex_id);
      const Point t_submap_vertex = vertex.point;
      way_points->emplace_back(RelativeWayPoint(submap_ptr, t_submap_vertex));
    }
  }
}

void SkeletonAStar::getSolutionVertexPath(
    GlobalVertexId end_vertex_id,
    const std::map<GlobalVertexId, GlobalVertexId>& parent_map,
    std::vector<GlobalVertexId>* vertex_path) {
  CHECK_NOTNULL(vertex_path);
  vertex_path->clear();
  vertex_path->push_back(end_vertex_id);
  GlobalVertexId vertex_id = end_vertex_id;
  while (parent_map.count(vertex_id) > 0) {
    vertex_id = parent_map.at(vertex_id);
    vertex_path->push_back(vertex_id);
  }
  std::reverse(vertex_path->begin(), vertex_path->end());
}

GlobalVertexId SkeletonAStar::popSmallestFromOpen(
    const std::map<GlobalVertexId, FloatingPoint>& f_score_map,
    std::set<GlobalVertexId>* open_set) {
  FloatingPoint min_distance = std::numeric_limits<FloatingPoint>::max();
  auto min_iter = open_set->cbegin();

  for (auto iter = open_set->cbegin(); iter != open_set->cend(); ++iter) {
    FloatingPoint distance = f_score_map.at(*iter);
    if (distance < min_distance) {
      min_distance = distance;
      min_iter = iter;
    }
  }

  GlobalVertexId return_val = *min_iter;
  open_set->erase(min_iter);
  return return_val;
}

}  // namespace glocal_exploration
