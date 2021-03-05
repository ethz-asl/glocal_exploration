#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_RH_RRT_STAR_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_RH_RRT_STAR_H_

#include <chrono>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "glocal_exploration/3rd_party/config_utilities.hpp"
#include "glocal_exploration/3rd_party/nanoflann.hpp"
#include "glocal_exploration/common.h"
#include "glocal_exploration/planning/local/lidar_model.h"
#include "glocal_exploration/planning/local/local_planner_base.h"
#include "glocal_exploration/planning/local/sensor_model.h"

namespace glocal_exploration {

class RHRRTStar : public LocalPlannerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;

    // Pathing.
    FloatingPoint min_path_length = 0.5f;        // m, min length of connections
    FloatingPoint max_path_length = 2.f;         // m
    FloatingPoint min_sampling_distance = 0.5f;  // m, minimum distance between
                                                 // viewpoints
    FloatingPoint path_cropping_length = 0.2f;   // m, distance until cropped
                                                 // paths become infeasible
    FloatingPoint traversability_radius = 1.f;

    // Behavior.
    int maximum_rewiring_iterations = 100;
    FloatingPoint sampling_range = 10.f;  // m
    int max_number_of_neighbors = 20;
    FloatingPoint reconsideration_time = 2.f;  // s, extra time taken before
                                               // switching to global or
                                               // reversing a path.

    // Termination.
    int terminaton_min_tree_size = 5;
    FloatingPoint termination_max_gain = 100.f;

    int DEBUG_number_of_iterations = -1;  // Only used if>0, use for debugging.

    // sensor model (currently just use lidar)
    LidarModel::Config lidar_config;

    void checkParams() const override;
    void fromRosParam() override;
    void printFields() const override;
    Config();
  };

  // setup
  RHRRTStar(const Config& config, std::shared_ptr<Communicator> communicator);
  ~RHRRTStar() override = default;

  // planning
  void executePlanningIteration() override;
  void resetPlanner(const WayPoint& new_origin) override;

  struct Connection;
  // View points are the vertices in the tree.
  struct ViewPoint {
   public:
    friend Connection;
    WayPoint pose;
    FloatingPoint gain = 0.f;
    FloatingPoint value = 0.f;
    bool is_root = false;
    bool is_connected_to_root = false;

    // access
    void setActiveConnection(size_t index);

    // helper methods
    Connection* addConnection(ViewPoint* target, MapBase* map);
    void removeConnection(const Connection* connection);
    Connection* getActiveConnection();
    const Connection* getActiveConnection() const;
    ViewPoint* getConnectedViewPoint(size_t index) const;
    Connection* getConnection(size_t index) const;
    ViewPoint* getActiveViewPoint() const;
    std::vector<ViewPoint*> getChildViewPoints() const;
    const std::vector<std::pair<bool, std::shared_ptr<Connection>>>&
    getConnections() const {
      return connections;
    }
    size_t getActiveConnectionIndex() const { return active_connection; }

   private:
    std::vector<std::pair<bool, std::shared_ptr<Connection>>>
        connections;  // true: this ViewPoint is the parent
    size_t active_connection = 0;
  };

  // connections are the edges in the tree
  struct Connection {
   public:
    friend ViewPoint;
    FloatingPoint cost;

    const ViewPoint* getParent() const { return parent; }
    const ViewPoint* getTarget() const { return target; }

   private:
    ViewPoint* parent;
    ViewPoint* target;
  };

  // Nanoflann KD-tree implementation
  struct TreeData {
    std::vector<std::unique_ptr<ViewPoint>> points;

    // Nanoflann functionality (this is required s.t. nanoflann can run).
    inline std::size_t kdtree_get_point_count() const { return points.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
      if (dim == 0)
        return points[idx]->pose.position.x();
      else if (dim == 1)
        return points[idx]->pose.position.y();
      else
        return points[idx]->pose.position.z();
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
      return false;
    }
  };
  typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
      nanoflann::L2_Simple_Adaptor<FloatingPoint, TreeData>, TreeData, 3>
      KDTree;

  // accessors for visualization
  const Config& getConfig() const { return config_; }
  const TreeData& getTreeData() const { return tree_data_; }
  void visualizeGain(const WayPoint& pose, std::vector<Point>* voxels,
                     std::vector<Point>* colors, FloatingPoint* scale) const;

 protected:
  /* components */
  const Config config_;
  TreeData tree_data_;
  std::unique_ptr<KDTree> kdtree_;
  std::unique_ptr<SensorModel> sensor_model_;

  /* methods */
  // general
  bool findNearestNeighbors(const Point& position, std::vector<size_t>* result,
                            int n_neighbors = 1);

  // tree building.
  void expandTree();
  bool sampleNewPoint(ViewPoint* point);
  bool connectViewPoint(ViewPoint* view_point);

  // compute gains.
  void evaluateViewPoint(ViewPoint* view_point);
  FloatingPoint computeCost(const Connection& connection);

  // extract best viewpoint.
  bool selectNextBestWayPoint(WayPoint* next_waypoint);
  bool optimizeTreeAndFindBestGoal(size_t* next_root_idx);
  bool selectBestConnection(ViewPoint* view_point);
  void computeValue(ViewPoint* view_point);
  static FloatingPoint computeGNVStep(ViewPoint* view_point, FloatingPoint gain,
                                      FloatingPoint cost,
                                      std::unordered_set<ViewPoint*>* visited);

  // updating.
  void updateCollision();
  void updateGains();
  void computePointsConnectedToRoot(bool count_only_active_connections);

  // termination
  void sampleReconsideration();
  bool isTerminationCriterionMet();

  /* variables */
  bool gain_update_needed_;
  ViewPoint* root_;  // root pointer so it does not need to be searched for all
  // the time.
  ViewPoint* previous_view_point_;  // Track this to detect for reversing.
  Connection* current_connection_;  // the connection currently being executed.
  bool reconsidered_;               // true: reverse/switch to global anyways.
  int number_of_executed_waypoints_;

  // stats
  int pruned_points_;
  int new_points_;
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_RH_RRT_STAR_H_
