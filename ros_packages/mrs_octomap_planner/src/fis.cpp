
#include "fis.hpp" 



namespace mrs_octomap_planner
{





FIS::FIS(){}

FIS::FIS(frontier_t cells, unsigned long id){
  id_ = id;
  cells_ = cells;
  center_ = mean(cells);
  octomap::point3d min = getMinBound(cells);
  octomap::point3d max = getMaxBound(cells);
  bbx_ = {.min=min, .max=max};
  sample_cnt_ = 20;
  viewpoints_ = std::vector<viewpoint_t>(0);
  visible_cells_ = frontier_t(0);
}

octomap::point3d FIS::sampleViewpoint(double r, double h)
{
  h = h*(getRand()-0.5);
  r = r*getRand();
  float theta = M_PI*getRand();

  return octomap::point3d(center_.x()+r*cos(theta),center_.y()+r*sin(theta), center_.z()+h);
}

unsigned int FIS::cellCnt()
{
  return cells_.size();
}

FIS::~FIS(){}

} // namespace mrs_
