#ifndef FRONTIER_MANAGER_H
#define FRONTIER_MANAGER_H


#include <mrs_lib/batch_visualizer.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <vector>
#include <queue>

#include "fis.hpp"


namespace mrs_octomap_planner
{


class FrontierManager
{
private:
    /* data */
  
  std::shared_ptr<mrs_lib::BatchVisualizer> bv_frontiers_;

  AABB zone_;
  octomap::OcTreeKey min_key_;
  octomap::OcTreeKey max_key_;
  long dx_; 
  long dy_;
  long dz_;

  std::shared_ptr<octomap::OcTree> tree_;
  std::vector<bool> global_closed_;
  
  unsigned long frontier_id_;

  double  free_space_diameter_;
  int     min_frontier_size_;
  int     min_svd_eigen_value_;
  double  size_decrease_ratio_;
  int     viewpoint_sample_attempts_;
  double  viewpoint_sample_radius_;
  double  viewpoint_sample_height_;
  double  viewpoint_max_distance_;
  double  viewpoint_max_angle_;
  int     max_viewpoints_per_fr_;
  int     min_viewpoints_per_fr_;
  double  resample_probability_; 
  int     min_coverage_;

  void removeFrontiers();
  void addFrontier(const frontier_t& frontier);
  bool isStillFrontier(const frontier_t& frontier);
  bool isFrontierCell(octomap::point3d cell_pos);
  long keyToClosedIdx(octomap::OcTreeKey start_key);
  void setZone(AABB zone);
  bool canBeProcessed(long key_idx, const std::vector<bool>& closed);
  void makeViewpoints(FIS* fis);

  
    
public:
  std::vector<std::unique_ptr<FIS>> fis_c_;
  int viewable_frontier_cnt_;
  std::vector<int> removed_frontiers_;
  std::vector<int> added_frontiers_idx_;

  FrontierManager(const std::shared_ptr<mrs_lib::BatchVisualizer>&  bv_frontiers,
                  double                                            free_space_diameter,
                  int                                               min_frontier_size,
                  int                                               min_svd_eigen_value,
                  double                                            size_decrease_ratio,
                  int                                               viewpoint_sample_attempts,
                  double                                            viewpoint_sample_radius,
                  double                                            viewpoint_sample_height,
                  double                                            viewpoint_max_distance,
                  double                                            viewpoint_max_angle,
                  int                                               max_viewpoints_per_fr,
                  int                                               min_viewpoints_per_fr,
                  double                                            resample_probability, 
                  int                                               min_coverage);
  ~FrontierManager();

  void processNewMap(const std::shared_ptr<octomap::OcTree>& tree, AABB region, octomap::OcTreeKey start_key);
};

} // namespace mrs_octomap_planner


#endif