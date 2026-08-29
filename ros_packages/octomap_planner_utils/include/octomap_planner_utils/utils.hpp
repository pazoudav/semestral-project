#ifndef OCTOMAP_PLANNER_UTILS_UTILS_H
#define OCTOMAP_PLANNER_UTILS_UTILS_H


#include <vector>
#include <octomap/octomap.h>
#include <algorithm>
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <math.h>
#include <functional>
#include <optional>
#include <string>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <geometry_msgs/Point.h>


namespace octomap_planner_utils
{

// RGB color in [0,1] range, used for visualization markers
struct color_t
{
  float r;
  float g;
  float b;
};

// axis-aligned bounding box defined by its min and max corner points
struct AABB
{
  octomap::point3d min;
  octomap::point3d max;
};

// cost breakdown for a path segment between two points (distance in meters, plus velocity/heading change and height terms), produced by a distance_funcion_t
struct path_info_t
{
  double distance=0.0;
  double velocity_delta=0.0;
  double angle_delta=0.0;
  double height=0.0;
};


// signature of a pairwise cost/distance function between two points, returning a path_info_t breakdown (e.g. used as an edge-cost metric for TSP ordering)
typedef std::function<path_info_t (octomap::point3d, octomap::point3d)> distance_funcion_t;

// sentinel distance values: no valid path found / effectively-infinite cost
const double INVALID_DISTANCE = -1.0;
const double BIG_DISTANCE = 1000000.0;

// the 26 integer (dx,dy,dz) offsets to a voxel's neighbours in the surrounding 3x3x3 block, excluding itself
const std::vector<std::vector<int>> NEIGHBOUR_OFFSETS = {
    {0, 0, -1}, {0, 0, 1}, {0, 1, 0},
    {0, -1, 0}, {1, 0, 0}, {-1, 0, 0},
    {-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1},
    {-1, 0, -1},  {-1, 0, 1},
    {-1, 1, -1}, {-1, 1, 0}, {-1, 1, 1},
    {0, -1, -1}, {0, -1, 1},
    {0, 1, -1},  {0, 1, 1},
    {1, -1, -1}, {1, -1, 0}, {1, -1, 1},
    {1, 0, -1}, {1, 0, 1},
    {1, 1, -1}, {1, 1, 0}, {1, 1, 1}
};


// returns a color from a small fixed palette, cycling every 15 indices (used e.g. for color-coding frontiers)
color_t getColor(int i);

// returns the key of the voxel offset from key by neighbour_offset (a (dx,dy,dz) integer offset)
octomap::OcTreeKey getNeighbourKey(octomap::OcTreeKey key, std::vector<int> neighbour_offset);
// returns the keys of all 26 voxels adjacent to current_node_key (see NEIGHBOUR_OFFSETS)
std::vector<octomap::OcTreeKey> getNeighboursKeys(octomap::OcTreeKey current_node_key);


// generates n quasi-uniformly distributed points on the unit sphere (Fibonacci sphere method); currently unused
std::vector<octomap::point3d> sampleSpherePoints(int n);

// returns the 8 corner points of the AABB
std::vector<octomap::point3d> getConrners(AABB a);

// builds an AABB centered at c with side lengths x, y, z
AABB aabbFromCenter(octomap::point3d c, double x, double y,double z);
// like aabbFromCenter, but clamps the min z up to floor if it would otherwise go below it; currently unused
AABB aabbSmartFromCenter(octomap::point3d c, double x, double y,double z, double floor=0.0);

// AABB centered on position, sized (width, width, height), clamped to flight_zone
AABB localZoneFromPosition(geometry_msgs::Point position, AABB flight_zone, double width, double height);

// true if point p lies within (inclusive of) AABB bbx0
bool intersect(AABB bbx0, octomap::point3d p);
// true if the two AABBs overlap (tested via corner containment in both directions)
bool intersect(AABB bbx0, AABB bbx1);

// component-wise a <= b
bool isSmallerEq (const octomath::Vector3& a, const octomath::Vector3& b);
// component-wise a >= b
bool isBiggerEq(const octomath::Vector3& a, const octomath::Vector3& b);
// smallest AABB that contains both a and b
AABB makeUnion(AABB a, AABB b);
// largest AABB contained within both a and b (degenerate/inverted if a and b don't overlap)
AABB makeIntersection(AABB a, AABB b);
// true if AABB a is fully contained within AABB b
bool isSubset(AABB a, AABB b);
// box volume (dx*dy*dz); meaningless/negative if min > max on any axis
float volume(AABB a);

// true if every voxel inside zone is known and free in tree (unknown or occupied voxels make it false)
bool isFreeSpace(AABB zone, const std::shared_ptr<octomap::OcTree>& tree);
// true if every voxel within diameter/2 of center (approx. a sphere) is known and free in tree
bool isFreeSpace(octomap::point3d center, double diameter, const std::shared_ptr<octomap::OcTree>& tree);

// uniform random float in [0,1)
float getRand();
// uniform random float in [a,b)
float getRand(float a, float b);
// uniform random int in [a,b)
int getRand(int a, int b);
// uniform random point sampled inside AABB a
octomap::point3d getSampleFromAABB(AABB a);

// get the UAV's current position (tracker command), transformed into octree_frame; assumes the octree mutex has already been locked by the caller to read octree_frame
std::optional<mrs_msgs::ReferenceStamped> getPosition(
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& sh_control_manager_diag,
    mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>&            sh_tracker_cmd,
    const std::string&                                              octree_frame,
    mrs_lib::Transformer&                                           transformer,
    const std::string&                                              log_tag);

}

#endif
