#ifndef GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_RH_RRT_STAR_H_
#define GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_RH_RRT_STAR_H_

#include <memory>
#include <utility>

#include <3rd_party/nanoflann.hpp>

#include "glocal_exploration/common.h"
#include "glocal_exploration/planning/local_planner/sensor_model.h"
#include "glocal_exploration/planning/local_planner/lidar_model.h"
#include "glocal_exploration/planning/local_planner/local_planner_base.h"

namespace glocal_exploration {

class RHRRTStar : public LocalPlannerBase {
 public:
  struct Config : LocalPlannerBase::Config {
    // sampling
    double local_sampling_radius = 1.5; // m
    double global_sampling_radius = 100; // m
    int min_local_points = 5;

    // path
    double min_path_length = 0.5;   // m
    double max_path_length = 2.0;   // m
    double path_cropping_length = 0.2;  // m, distance until cropped paths become infeasible
    int max_number_of_neighbors = 20;

    // behavior
    int maximum_rewiring_iterations = 100;

    // sensor model (currently just use lidar)
    LidarModel::Config lidar_config;
  };

  // setup
  RHRRTStar(std::shared_ptr<MapBase> map, std::shared_ptr<StateMachine> state_machine);
  virtual ~RHRRTStar() = default;
  bool setupFromConfig(LocalPlannerBase::Config *config) override;

  // planning
  void planningIteration() override;

 protected:
  struct Connection;
  // View points are the vertices in the tree
  struct ViewPoint {
    WayPoint pose;
    double gain = 0;
    double value = 0;
    bool is_root = false;
    std::vector<std::pair<bool, std::shared_ptr<Connection>>> connections;   // true: this ViewPoint is the parent
    size_t active_connection;

    // helper methods
    bool tryAddConnection(ViewPoint *target, const std::shared_ptr<MapBase> &map);
    Connection* getActiveConnection();
    ViewPoint* getConnectedViewPoint(size_t index);
  };

  // connections are the edges in the tree
  struct Connection {
    ViewPoint *parent;
    ViewPoint *target;
    std::vector<Eigen::Vector3d> path_points;
    double cost;
  };

  // Nanoflann KD-tree implementation
  struct TreeData {
    std::vector<ViewPoint> points;

    // nanoflann functionality (This is required s.t. nanoflann can run)
    inline std::size_t kdtree_get_point_count() const { return points.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
      if (dim == 0)
        return points[idx].pose.x;
      else if (dim == 1)
        return points[idx].pose.y;
      else
        return points[idx].pose.z;
    }

    template<class BBOX>
    bool kdtree_get_bbox(BBOX & /* bb */) const {
      return false;
    }
  };
  typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, TreeData>, TreeData, 3>
      KDTree;

  /* components */
  Config config_;
  TreeData tree_data_;
  std::unique_ptr<KDTree> kdtree_;
  std::unique_ptr<SensorModel> sensor_model_;

  /* methods */
  // general
  void resetPlanner(const WayPoint &origin);
  bool findNearestNeighbors(Eigen::Vector3d position, std::vector<ViewPoint *> *result, int n_neighbors = 1);

  // tree building
  void expandTree();
  bool sampleNewPoint(ViewPoint *point);
  void evaluateViewPoint(ViewPoint *view_point);
  bool connectViewPoint(ViewPoint *view_point);

  // extract best viewpoint
  void selectNextBestWayPoint(WayPoint *way_point);
  bool selectBestConnection(ViewPoint *view_point);
  void computeValue(ViewPoint *view_point);
  double computeGNVStep(ViewPoint* view_point, double gain, double cost);

  // updating
  void updateTree();
  void updateCollision();
  void updateGains();

  /* variables */
  int local_sampled_points_;

};

} // namespace glocal_exploration

#endif // GLOCAL_EXPLORATION_PLANNING_LOCAL_PLANNER_RH_RRT_STAR_H_
