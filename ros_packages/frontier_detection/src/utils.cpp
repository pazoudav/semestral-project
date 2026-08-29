#include "frontier_detection/utils.hpp"


namespace frontier_detection
{

octomap::point3d mean(frontier_t cells){
  octomap::point3d s(0.0,0.0,0.0);
  for (auto &c : cells)
  {
    s += c;
  }
  s /= (float)cells.size();
  return s;
}

// min x,y,z values in octomath::vecor
octomap::point3d getMinBound(const frontier_t& fr){
  double mx=fr[0].x();
  double my=fr[0].y();
  double mz=fr[0].z();
  for (auto & cell : fr)
  {
    if (cell.x() < mx)
      mx = cell.x();
     if (cell.y() < my)
      my = cell.y();
     if (cell.z() < mz)
      mz = cell.z();
  }
  return octomap::point3d(mx,my,mz);
}

// max x,y,z values in octomath::vecor
octomap::point3d getMaxBound(const frontier_t& fr){
  double mx=fr[0].x();
  double my=fr[0].y();
  double mz=fr[0].z();
  for (auto & cell : fr)
  {
    if (cell.x() > mx)
      mx = cell.x();
     if (cell.y() > my)
      my = cell.y();
     if (cell.z() > mz)
      mz = cell.z();
  }
  return octomap::point3d(mx,my,mz);
}

}
