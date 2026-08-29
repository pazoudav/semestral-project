#include "frontier_detection/fis.hpp"


namespace frontier_detection
{


// frontier information structure from FUEL paper
FIS::FIS(){
  valid_ = true;
}

FIS::FIS(frontier_t cells, unsigned long id){
  id_ = id;
  cells_ = cells;
  center_ = mean(cells);
  octomap::point3d min = getMinBound(cells);
  octomap::point3d max = getMaxBound(cells);
  bbx_ = {.min=min, .max=max};
  sample_cnt_ = 20;
  viewpoints_ = std::vector<viewpoint_t>(0);
  valid_ = true;
}

octomap::point3d FIS::sampleViewpoint(double r, double h)
{
  h = h*(octomap_planner_utils::getRand()-0.5);
  r = r*octomap_planner_utils::getRand();
  float theta = M_PI*octomap_planner_utils::getRand();

  return octomap::point3d(center_.x()+r*cos(theta),center_.y()+r*sin(theta), center_.z()+h);
}

unsigned int FIS::cellCnt()
{
  return cells_.size();
}

FIS::~FIS(){}

} // namespace frontier_detection
