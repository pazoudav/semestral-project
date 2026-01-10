#ifndef UTILS_H
#define UTILS_H


#include <vector>
#include <octomap/octomap.h>
#include <algorithm>
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <math.h>
#include <functional>

// /home/david/CVUT/diplomka/mrs_apptainer/user_ros_workspace/include_ros/**
// /home/david/CVUT/diplomka/mrs_apptainer/user_ros_workspace/include_eigen/**


namespace mrs_octomap_planner
{

typedef std::vector<octomap::point3d> frontier_t;
typedef std::function<double (octomap::point3d, octomap::point3d)> distance_funcion_t;

const double INVALID_DISTANCE = -1.0;
const double BIG_DISTANCE = 1000000.0;

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

// const float FREE_SPACE_SIZE = 1.6;
// const float ZONE_HEIGHT = 11.0;
// const float ZONE_WIDTH = 11.0;
// const float BIG_DISTANCE = 1000000.0;

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

// auto SPHERE_POINTS = sampleSpherePoints(128);



color_t getColor(int i);

octomap::OcTreeKey getNeighbourKey(octomap::OcTreeKey key, std::vector<int> neighbour_offset);
std::vector<octomap::OcTreeKey> getNeighboursKeys(octomap::OcTreeKey current_node_key);


std::vector<octomap::point3d> sampleSpherePoints(int n);

std::vector<octomap::point3d> getConrners(AABB a);

AABB aabbFromCenter(octomap::point3d c, double x, double y,double z);
AABB aabbSmartFromCenter(octomap::point3d c, double x, double y,double z, double floor=0.0);

bool intersect(AABB bbx0, octomap::point3d p);
bool intersect(AABB bbx0, AABB bbx1);

octomap::point3d mean(frontier_t cells);
bool isSmallerEq (const octomath::Vector3& a, const octomath::Vector3& b);
bool isBiggerEq(const octomath::Vector3& a, const octomath::Vector3& b);
octomap::point3d getMinBound(const frontier_t& fr);
octomap::point3d getMaxBound(const frontier_t& fr);
AABB makeUnion(AABB a, AABB b);
AABB makeIntersection(AABB a, AABB b);
bool isSubset(AABB a, AABB b);
float volume(AABB a);

bool isFreeSpace(AABB zone, const std::shared_ptr<octomap::OcTree>& tree);
bool isFreeSpace(octomap::point3d center, double diameter, const std::shared_ptr<octomap::OcTree>& tree);

float getRand();
float getRand(float a, float b);
octomap::point3d getSampleFromAABB(AABB a);

}

#endif
