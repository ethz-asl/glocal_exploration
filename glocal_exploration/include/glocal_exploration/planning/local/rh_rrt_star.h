#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_RH_RRT_STAR_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_RH_RRT_STAR_H_

#include <memory>
#include <utility>
#include <vector>

#include <3rd_party/nanoflann.hpp>

#include "glocal_exploration/common.h"
#include "glocal_exploration/planning/local/lidar_model.h"
#include "glocal_exploration/planning/local/local_planner_base.h"
#include "glocal_exploration/planning/local/sensor_model.h"

namespace glocal_exploration {

class RHRRTStar : public LocalPlannerBase {
 public:
  struct Config {
    // sampling
    double local_sampling_radius = 1.5;   // m
    double global_sampling_radius = 100;  // m
    int min_local_points = 5;

    // path
    double min_path_length = 0.5;  // m, determines the length of connections
    double min_sampling_distance =
        0.5;  // m, determines the minimum path length when sampling
    double max_path_length = 2.0;  // m
    double path_cropping_length =
        0.2;  // m, distance until cropped paths become infeasible
    int max_number_of_neighbors = 20;

    // behavior
    int maximum_rewiring_iterations = 100;

    // sensor model (currently just use lidar)
    LidarModel::Config lidar_config;
    Config isValid() const;
  };

  // setup
  explicit RHRRTStar(const Config& config,
                     std::shared_ptr<Communicator> communicator);
  virtual ~RHRRTStar() = default;

  // planning
  void planningIteration() override;

  struct Connection;
  // View points are the vertices in the tree
  struct ViewPoint {
    WayPoint pose;
    double gain = 0;
    double value = 0;
    bool is_root = false;
    bool is_connected_to_root = false;
    std::vector<std::pair<bool, std::shared_ptr<Connection>>>
        connections;  // true: this ViewPoint is the parent
    size_t active_connection = 0;

    // helper methods
    bool tryAddConnection(ViewPoint* target, MapBase* map);
    Connection* getActiveConnection();
    Connection const* getActiveConnection() const;
    ViewPoint* getConnectedViewPoint(size_t index);
  };

  // connections are the edges in the tree
  struct Connection {
    ViewPoint* parent;
    ViewPoint* target;
    std::vector<Eigen::Vector3d> path_points;
    double cost;
  };

  // Nanoflann KD-tree implementation
  struct TreeData {
    std::vector<std::unique_ptr<ViewPoint>> points;

    // nanoflann functionality (This is required s.t. nanoflann can run)
    inline std::size_t kdtree_get_point_count() const { return points.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
      if (dim == 0)
        return points[idx]->pose.x;
      else if (dim == 1)
        return points[idx]->pose.y;
      else
        return points[idx]->pose.z;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
      return false;
    }
  };
  typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
      nanoflann::L2_Simple_Adaptor<double, TreeData>, TreeData, 3>
      KDTree;

  // accessors for visualization
  const Config& getConfig() const { return config_; }
  const TreeData& getTreeData() const { return tree_data_; }
  void visualizeGain(std::vector<Eigen::Vector3d>* voxels,
                     std::vector<Eigen::Vector3d>* colors, double* scale,
                     const WayPoint& pose) const;

 protected:
  /* components */
  const Config config_;
  TreeData tree_data_;
  std::unique_ptr<KDTree> kdtree_;
  std::unique_ptr<SensorModel> sensor_model_;

  /* methods */
  // general
  void resetPlanner(const WayPoint& origin);
  bool findNearestNeighbors(Eigen::Vector3d position,
                            std::vector<size_t>* result, int n_neighbors = 1);

  // tree building
  void expandTree();
  bool sampleNewPoint(ViewPoint* point);
  bool connectViewPoint(ViewPoint* view_point);

  // compute gains
  void evaluateViewPoint(ViewPoint* view_point);
  double computeGain(const std::vector<Eigen::Vector3d>& visible_voxels);
  double computeCost(const Connection& connection);

  // extract best viewpoint
  bool selectNextBestWayPoint(WayPoint* next_waypoint);
  bool selectBestConnection(ViewPoint* view_point);
  void computeValue(ViewPoint* view_point);
  static double computeGNVStep(ViewPoint* view_point, double gain, double cost);

  // updating
  void updateTree();
  void updateCollision();
  void updateGains();
  void computePointsConnectedToRoot(bool count_only_active_connections);

  /* variables */
  int local_sampled_points_;
  int num_previous_points_;
  int next_root_index_;  // -1 if there is no next root, >=0 if tree should
                         // update
  ViewPoint* root_;  // root pointer so it does not need to be searched for all
                     // the time
};

}  // namespace glocal_exploration

#endif  // GLOCAL_EXPLORATION_PLANNING_LOCAL_RH_RRT_STAR_H_
