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

struct color_t
{
  float r;
  float g;
  float b;
};

struct AABB
{
  octomap::point3d min;
  octomap::point3d max;
};

struct path_info_t
{
  double distance=0.0;
  double velocity_delta=0.0;
  double angle_delta=0.0;
  double height=0.0;
};


typedef std::function<path_info_t (octomap::point3d, octomap::point3d)> distance_funcion_t;

const double INVALID_DISTANCE = -1.0;
const double BIG_DISTANCE = 1000000.0;

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


color_t getColor(int i);

octomap::OcTreeKey getNeighbourKey(octomap::OcTreeKey key, std::vector<int> neighbour_offset);
std::vector<octomap::OcTreeKey> getNeighboursKeys(octomap::OcTreeKey current_node_key);


std::vector<octomap::point3d> sampleSpherePoints(int n);

std::vector<octomap::point3d> getConrners(AABB a);

AABB aabbFromCenter(octomap::point3d c, double x, double y,double z);
AABB aabbSmartFromCenter(octomap::point3d c, double x, double y,double z, double floor=0.0);

// AABB centered on position, sized (width, width, height), clamped to flight_zone
AABB localZoneFromPosition(geometry_msgs::Point position, AABB flight_zone, double width, double height);

bool intersect(AABB bbx0, octomap::point3d p);
bool intersect(AABB bbx0, AABB bbx1);

bool isSmallerEq (const octomath::Vector3& a, const octomath::Vector3& b);
bool isBiggerEq(const octomath::Vector3& a, const octomath::Vector3& b);
AABB makeUnion(AABB a, AABB b);
AABB makeIntersection(AABB a, AABB b);
bool isSubset(AABB a, AABB b);
float volume(AABB a);

bool isFreeSpace(AABB zone, const std::shared_ptr<octomap::OcTree>& tree);
bool isFreeSpace(octomap::point3d center, double diameter, const std::shared_ptr<octomap::OcTree>& tree);

float getRand();
float getRand(float a, float b);
int getRand(int a, int b);
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
