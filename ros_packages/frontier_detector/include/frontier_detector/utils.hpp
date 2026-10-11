#ifndef FRONTIER_DETECTOR_UTILS_H
#define FRONTIER_DETECTOR_UTILS_H

#include <vector>
#include <octomap/octomap.h>

namespace frontier_detector
{

typedef std::vector<octomap::point3d> frontier_t;

octomap::point3d mean(const frontier_t& cells);
octomap::point3d getMinBound(const frontier_t& fr);
octomap::point3d getMaxBound(const frontier_t& fr);

}

#endif
