#ifndef FRONTIER_DETECTION_FIS_H
#define FRONTIER_DETECTION_FIS_H

#include <vector>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <algorithm>
#include <math.h>
#include <octomap_planner_utils/utils.hpp>
#include "frontier_detection/utils.hpp"

namespace frontier_detection
{

// a candidate observation point for a frontier, with how many of the frontier's cells are visible from it
struct viewpoint_t{
  octomap::point3d position;
  int coverage;
};

// Frontier Information Structure: one clustered set of frontier cells, its bounding box/center, and its sampled viewpoints
class FIS
{
private:
    double sample_r_;
    double sample_h_;
    int sample_cnt_;

public:
  FIS();
  FIS(frontier_t cells, unsigned long id);
  ~FIS();

  unsigned long id_;
  bool valid_;

  // randomly samples a candidate viewpoint within horizontal radius r and vertical offset h around the frontier's center
  octomap::point3d sampleViewpoint(double r, double h);
  // number of cells belonging to this frontier
  unsigned int cellCnt();

  octomap_planner_utils::AABB bbx_;
  octomap::point3d center_;
  frontier_t cells_;
  std::vector<viewpoint_t> viewpoints_;
};



} // namespace frontier_detection


#endif
