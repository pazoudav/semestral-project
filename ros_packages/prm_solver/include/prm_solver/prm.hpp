#ifndef PRM_H
#define PRM_H

#include <mrs_lib/batch_visualizer.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <vector>
#include <queue>
#include <functional>
#include <octomap_planner_utils/utils.hpp>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/transforms.h>


// struct PointXYZIDX
// {
//   PCL_ADD_POINT4D;                  // quad-word XYZ
//   uint32_t idx;

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // required for Eigen alignment
// } EIGEN_ALIGN16;                    // enforce SSE padding

// POINT_CLOUD_REGISTER_POINT_STRUCT(
//     PointXYZIDX,
//     (float, x, x)
//     (float, y, y)
//     (float, z, z)
//     (uint32_t, idx, idx)
// )

namespace prm_solver
{

typedef struct node_t node_t;

struct node_t{
  octomap::point3d position;
  std::vector<std::weak_ptr<node_t>> neighbors;
  unsigned long idx;
  bool isViewpoint=false;
  bool valid = true;
  int age = 0;
};

struct path_t{
  std::vector<node_t> nodes;
  float length;
};



struct a_start_node_t{
  unsigned long idx;
  float f;
};

// min-heap comparator for the A* open set (lowest f-score first)
struct customLess
{
    bool operator()(a_start_node_t a, a_start_node_t b) const { return a.f > b.f; }
} ;



class PRM
{
private:
  std::vector<std::shared_ptr<node_t>> nodes_;
  float resample_factor_ ;
  int node_cnt_;
  float max_cost_;
  float node_max_age_;
  float neighbor_overlap_;
  double free_space_diameter_;
  int max_neighbors_;
  double min_neighbor_distance_;
  double max_neighbor_distance_;
  double min_node_distance_;
  Eigen::MatrixXd cost_matrix_;
  std::shared_ptr<mrs_lib::BatchVisualizer> bv_prm_;
  std::shared_ptr<octomap::OcTree> tree_;
  std::vector<bool> frozen_path_;

  // void removeNode(node_t* node);
  // sweeps nodes marked invalid (dead neighbor refs pruned first), compacting the vector in place
  void removeInvalidNodes();
  // void addNode(octomap::point3d position);
  // returns nodes within radius r of point, sorted nearest-first
  std::vector<node_t> findCloseNodes(octomap::point3d point, double r);
  // A* search over the roadmap graph between two existing nodes, straight-line distance heuristic
  path_t findNodePath(node_t start, node_t goal);


public:
  PRM(std::shared_ptr<mrs_lib::BatchVisualizer> bv_planner,
      double                                    free_space_diameter,
      double                                    overlap_coefficient,
      double                                    resample_factor,
      int                                       node_max_age,
      int                                       max_neighbors,
      double                                    min_neighbor_distance,
      double                                    max_neighbor_distance,
      double                                    min_node_distance);
  PRM();
  ~PRM();

  // inserts a new roadmap node at position (unless too close to an existing one) and wires it to nearby nodes with a clear line-of-free-space between them
  void addNode(octomap::point3d position);
  // ages/invalidates/removes roadmap nodes inside zone and resamples new ones to keep node density in the zone up to the resample budget; call periodically and on every new map
  void updateZone(const std::shared_ptr<octomap::OcTree>& tree, octomap_planner_utils::AABB zone, bool map_update);
  // finds nearest roadmap nodes to start/goal and returns an A* path between them (start/goal appended as endpoints); empty if unreachable or goal not in free space
  std::vector<octomap::point3d> findPath(octomap::point3d start, octomap::point3d goal, octomath::Vector3 velocity);
  // findPath() followed by a forward+reverse simplifyFreeSpacePath() pass to shortcut the roadmap path where free space allows
  std::vector<octomap::point3d> findSimplifiedPath(octomap::point3d start, octomap::point3d goal, octomath::Vector3 velocity);
  // dead/unused: shortcuts a path wherever consecutive waypoints have an unoccupied raycast between them (line-of-sight only, ignores drone size); not called anywhere
  std::vector<octomap::point3d> simplifyRaycastPath(std::vector<octomap::point3d> path);
  // shortcuts a path by merging waypoints whenever the straight segment between them stays in free space for the drone's size (free_space_diameter_)
  std::vector<octomap::point3d> simplifyFreeSpacePath(std::vector<octomap::point3d> path);
  // dead/unused: total length of a simplified findPath() route between start and goal; not called anywhere
  double distance(octomap::point3d start, octomap::point3d goal);
  // dead/unused: computes path length/height-change/angle-change/velocity-change stats along a simplified findPath() route; not called anywhere
  octomap_planner_utils::path_info_t extraDistance(octomap::point3d start, octomap::point3d goal);
  // void freezePath();
};
}




#endif
