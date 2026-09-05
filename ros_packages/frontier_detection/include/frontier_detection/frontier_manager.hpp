#ifndef FRONTIER_DETECTION_FRONTIER_MANAGER_H
#define FRONTIER_DETECTION_FRONTIER_MANAGER_H


#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <vector>
#include <queue>
#include <cstdint>

#include "frontier_detection/fis.hpp"


namespace frontier_detection
{


// tracks the set of frontier clusters (FIS) inside a local zone: extracts new frontiers via BFS over the octomap,
// invalidates/removes stale ones, and samples/scores viewpoints for the remaining ones
class FrontierManager
{
private:
  octomap_planner_utils::AABB zone_;
  octomap::OcTreeKey min_key_;
  octomap::OcTreeKey max_key_;
  long dx_;
  long dy_;
  long dz_;

  // non-owning: valid only for the duration of the processNewMap() call that sets it, which owns the tree via a shared_ptr
  octomap::OcTree* tree_ = nullptr;
  std::vector<bool> global_closed_;

  // per-cell "last visited during frontier epoch" stamps, sized to the zone like global_closed_; avoids
  // reallocating/zeroing a zone-sized closed-set for every frontier cluster extracted in frontierSearch()
  std::vector<uint32_t> frontier_stamp_;
  uint32_t current_frontier_epoch_ = 0;

  // reused across viewpointCoverage() raycasts to avoid octomap::KeyRay's 100k-element buffer
  // allocation on every default construction (computeRayKeys() resets it internally before use)
  octomap::KeyRay ray_scratch_;

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
  int     min_coverage_;

  // drops frontiers that are no longer valid (isStillFrontier), growing the search zone to cover them, and marks cells of surviving frontiers as already-explored
  void removeFrontiers();
  // adds a frontier cluster as a new FIS, recursively splitting it via SVD first if it is too large/elongated
  void addFrontier(const frontier_t& frontier);
  // re-checks a previously found frontier against the current map: still valid if enough of its cells are still unknown-adjacent-to-free cells
  bool isStillFrontier(const frontier_t& frontier);
  // true if the cell is unknown and has at least one free cell among its 6-connected neighbors
  bool isFrontierCell(octomap::point3d cell_pos);
  // maps an octree key to a flat index into the zone's closed-cell bookkeeping arrays
  long keyToClosedIdx(octomap::OcTreeKey start_key);
  // true if key lies within [min_key_, max_key_] (equivalent to intersect(zone_, tree_->keyToCoord(key)) but without the coordinate conversion)
  bool keyInZone(const octomap::OcTreeKey& key);
  // sets the active search zone and (re)sizes the closed-cell bookkeeping arrays to cover it
  void setZone(octomap_planner_utils::AABB zone);
  // true if the flat index is within bounds and not yet marked closed
  bool canBeProcessed(long key_idx, const std::vector<bool>& closed);
  // samples candidate viewpoints around a frontier, keeping only free-space ones with sufficient coverage, sorted best-first
  void makeViewpoints(std::shared_ptr<FIS> fis);
  // (re)builds viewpoints for every valid frontier in zone_ that has none yet, re-checks existing viewpoints' coverage, drops frontiers left with no viewpoints, and updates viewable_frontier_cnt_
  void setViewpoints();
  // counts how many of the frontier's cells are within range/FOV and unoccluded (raycast) from the given viewpoint
  int  viewpointCoverage(octomap::point3d viewpoint_pos, const frontier_t& frontier_cells);
  // BFS over free/unknown space from start_key within zone_, extracting all newly-encountered frontier clusters
  std::vector<frontier_t> frontierSearch(octomap::OcTreeKey start_key);



public:
  std::vector<std::shared_ptr<FIS>> fis_c_;
  int viewable_frontier_cnt_;
  std::vector<octomap::point3d> invalidated_frontiers_;
  std::vector<octomap::point3d> added_frontiers_;

  FrontierManager(double free_space_diameter,
                  int    min_frontier_size,
                  int    min_svd_eigen_value,
                  double size_decrease_ratio,
                  int    viewpoint_sample_attempts,
                  double viewpoint_sample_radius,
                  double viewpoint_sample_height,
                  double viewpoint_max_distance,
                  double viewpoint_max_angle,
                  int    max_viewpoints_per_fr,
                  int    min_coverage);
  ~FrontierManager();

  // full update pass for a new octomap: removes stale frontiers, BFS-extracts new frontier clusters from the given zone/start cell, and (re)samples their viewpoints
  // tree is borrowed for the duration of this call only; the caller retains ownership
  void processNewMap(octomap::OcTree* tree, octomap_planner_utils::AABB region, octomap::OcTreeKey start_key);

  // minimum viewpoint coverage a viewpoint must retain to be considered valid (used by callers building their own visualization of fis_c_)
  int minCoverage() const { return min_coverage_; }
  // the current (possibly grown-to-cover-removed-frontiers) search zone, as last set by processNewMap
  octomap_planner_utils::AABB currentZone() const { return zone_; }
};

} // namespace frontier_detection


#endif
