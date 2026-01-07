#ifndef FIS_H
#define FIS_H

#include <vector>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <algorithm>
#include <math.h>
#include "utils.hpp"

namespace mrs_octomap_planner
{

struct viewpoint_t{
  octomap::point3d position;
  int coverage;
};

class FIS
{
private:
    /* data */
    double sample_r_;
    double sample_h_;
    int sample_cnt_;
    
    
    

public:
  FIS();
  FIS(frontier_t cells, unsigned long id);
  ~FIS();

  unsigned long id_;

  octomap::point3d sampleViewpoint(double r, double h);
  unsigned int cellCnt();

  AABB bbx_;
  octomap::point3d center_;
  frontier_t cells_;
  frontier_t visible_cells_;
  std::vector<viewpoint_t> viewpoints_;
};



} // namespace mrs_


#endif