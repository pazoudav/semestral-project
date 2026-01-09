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
#include "utils.hpp"

namespace mrs_octomap_planner
{
    
typedef struct node_t node_t;

struct node_t{
  octomap::point3d position;
  std::vector<std::weak_ptr<node_t>> neighbors;
  unsigned long idx;
  bool isVievpoint=false;
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
  Eigen::MatrixXd cost_matrix_;
  std::shared_ptr<mrs_lib::BatchVisualizer> bv_prm_;
  std::shared_ptr<octomap::OcTree> tree_;
  std::vector<bool> frozen_path_;

  // void removeNode(node_t* node);
  void removeInvalidNodes();
  // void addNode(octomap::point3d position);
  std::vector<node_t> findCloseNodes(octomap::point3d point, double r);
  path_t findNodePath(node_t start, node_t goal);
  

public:
  PRM(std::shared_ptr<mrs_lib::BatchVisualizer> bv_planner, 
      double                                    free_space_diameter, 
      double                                    overlap_coefficient, 
      double                                    resample_factor,
      int                                       node_max_age,
      int                                       max_neighbors, 
      double                                    min_neighbor_distance,
      double                                    max_neighbor_distance);
  PRM();
  ~PRM();
  
  void addNode(octomap::point3d position);
  void updateZone(const std::shared_ptr<octomap::OcTree>& tree, AABB zone, bool map_update);
  std::vector<octomap::point3d> findPath(octomap::point3d start, octomap::point3d goal, octomath::Vector3 velocity);
  std::vector<octomap::point3d> simplifyRaycastPath(std::vector<octomap::point3d> path);
  std::vector<octomap::point3d> simplifyFreeSpacePath(std::vector<octomap::point3d> path);
  // void freezePath();
};
}






#endif