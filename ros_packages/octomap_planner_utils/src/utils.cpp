#include "octomap_planner_utils/utils.hpp"


namespace octomap_planner_utils
{

auto SPHERE_POINTS = sampleSpherePoints(128);


// used for plotting frontires
color_t COLORS[] = {{.r=1.0, .g=0.0, .b=0.0},
                    {.r=0.0, .g=1.0, .b=0.0},
                    {.r=1.0, .g=0.0, .b=1.0},

                    {.r=1.0, .g=1.0, .b=0.0},
                    {.r=1.0, .g=0.0, .b=1.0},
                    {.r=0.0, .g=1.0, .b=1.0},

                    {.r=1.0, .g=0.5, .b=0.0},
                    {.r=1.0, .g=0.0, .b=0.5},

                    {.r=0.0, .g=1.0, .b=0.5},
                    {.r=0.5, .g=1.0, .b=0.0},

                    {.r=0.5, .g=0.0, .b=1.0},
                    {.r=0.0, .g=0.5, .b=1.0},

                    {.r=1.0, .g=1.0, .b=0.5},
                    {.r=1.0, .g=0.5, .b=1.0},
                    {.r=0.5, .g=1.0, .b=1.0},
                  };

color_t getColor(int i){
  return COLORS[i%15];
}

// get a key to adjecent node in octomap
octomap::OcTreeKey getNeighbourKey(octomap::OcTreeKey key, std::vector<int> neighbour_offset){
  octomap::OcTreeKey new_key;
  new_key.k[0] = key.k[0] + neighbour_offset[0];
  new_key.k[1] = key.k[1] + neighbour_offset[1];
  new_key.k[2] = key.k[2] + neighbour_offset[2];
  return new_key;
}

// get keys to adjecent (26 neighbors) nodes in octomap
std::vector<octomap::OcTreeKey> getNeighboursKeys(octomap::OcTreeKey current_node_key) {

  //Add current node to the closed set
  std::vector<octomap::OcTreeKey> res(0);

  for(auto &neighbour_offset : NEIGHBOUR_OFFSETS){
      octomap::OcTreeKey neighbour_key = getNeighbourKey(current_node_key, neighbour_offset);
      res.push_back(neighbour_key);
  }
  return res;
}

// not used now
std::vector<octomap::point3d> sampleSpherePoints(int n)
{
  std::vector<octomap::point3d> points(0);
  float x; float y; float z; float theta; float psi;
  float eps = 0.5;
  float phi = (1+std::sqrt(5))/2;
  for (int i=0; i<n; i++)
  {
    x = (i/phi);
    x -= std::floor(x);
    y = (i+eps)/(n-1+eps);
    theta = 2*M_PI*x;
    psi = std::acos(1-2*y);
    x = std::cos(theta)*std::sin(psi);
    y = sin(theta)*sin(psi);
    z = cos(psi);
    points.push_back(octomap::point3d(x,y,z));
  }
  return points;
}


bool isSmallerEq(const octomath::Vector3& a, const octomath::Vector3& b)
{
  return a.x() <= b.x() && a.y() <= b.y() && a.z() <= b.z();
}

bool isBiggerEq(const octomath::Vector3& a, const octomath::Vector3& b)
{
  return a.x() >= b.x() && a.y() >= b.y() && a.z() >= b.z();
}

std::vector<octomap::point3d> getConrners(AABB a){
  std::vector<octomap::point3d> l;

  for (auto dx : {a.max.x(), a.min.x()}){
    for (auto dy : {a.max.y(), a.min.y()}){
      for (auto dz : {a.max.z(), a.min.z()}){
        octomap::point3d corner(dx, dy, dz);
        l.push_back(corner);
      }
    }
  }
  return l;
}

float volume(AABB a)
{
  octomap::point3d d = a.max - a.min;
  return  d.x()*d.y()*d.z();
}

// make AABB form centerpoint and side lenghts
AABB aabbFromCenter(octomap::point3d c, double x, double y,double z){
  AABB b;
  b.min = octomap::point3d(c.x()-x/2, c.y()-y/2,c.z()-z/2);
  b.max = octomap::point3d(c.x()+x/2, c.y()+y/2,c.z()+z/2);
  return b;
}

AABB localZoneFromPosition(geometry_msgs::Point position, AABB flight_zone, double width, double height)
{
  octomap::point3d coord(position.x, position.y, position.z);
  AABB local_zone = aabbFromCenter(coord, width, width, height);
  return makeIntersection(local_zone, flight_zone);
}

// make AABB form centerpoint and side lenghts that is above a floor level (not used)
AABB aabbSmartFromCenter(octomap::point3d c, double x, double y,double z, double floor){
  AABB res = aabbFromCenter(c,x,y,z);

  if (c.z() - (z/2.0) < floor){
    res.min = octomap::point3d(res.min.x(), res.min.y(), floor);
  }
  return res;
}

// make smllest AABB that contains 2 AABB
AABB makeUnion(AABB a, AABB b)
{
  octomap::point3d min(std::min(a.min.x(), b.min.x()), std::min(a.min.y(), b.min.y()), std::min(a.min.z(), b.min.z()));
  octomap::point3d max(std::max(a.max.x(), b.max.x()), std::max(a.max.y(), b.max.y()), std::max(a.max.z(), b.max.z()));

  AABB u = {.min=min, .max=max};
  return u;
}

// make biggest AABB that is contained by 2 AABB
AABB makeIntersection(AABB a, AABB b)
{
  octomap::point3d min(std::max(a.min.x(), b.min.x()), std::max(a.min.y(), b.min.y()), std::max(a.min.z(), b.min.z()));
  octomap::point3d max(std::min(a.max.x(), b.max.x()), std::min(a.max.y(), b.max.y()), std::min(a.max.z(), b.max.z()));

  AABB u = {.min=min, .max=max};
  return u;
}

// is a a subset of b?
bool isSubset(AABB a, AABB b)
{
  return isBiggerEq(a.min, b.min) && isSmallerEq(a.max, b.max);
}

// do a and b intersect
bool intersect(AABB bbx0, AABB bbx1){

  for (auto corner : getConrners(bbx1)){
    if (intersect(bbx0, corner))
    {
      return true;
    }
  }
  for (auto corner : getConrners(bbx0)){
    if (intersect(bbx1, corner))
    {
      return true;
    }
  }
  return false;
}

bool intersect(AABB bbx0, octomap::point3d p){

    if (isSmallerEq(bbx0.min, p) && isBiggerEq(bbx0.max,p))
    {
      return true;
    }

  return false;
}

// get random float between 0-1
float getRand()
{
  return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

float getRand(float a, float b)
{
  float r = getRand();
  return a + (b-a)*r;
}
int getRand(int a, int b)
{
  float r = getRand();
  return (int) std::floor(a + (b-a)*r);
}

// sample random in point from AABB
octomap::point3d getSampleFromAABB(AABB a)
{
  return octomap::point3d(getRand(a.min.x(), a.max.x()),
                          getRand(a.min.y(), a.max.y()),
                          getRand(a.min.z(), a.max.z()));
}

// is a space in AABB free in teh octotree
bool isFreeSpace(AABB zone, const std::shared_ptr<octomap::OcTree>& tree)
{
  octomap::point3d idx_bound = (zone.max - zone.min)*(1.0/tree->getResolution());
  for (int zi=0; zi<=(int)idx_bound.z(); zi++)
  {
    float z = zone.min.z() + zi*tree->getResolution();
    for (int yi=0; yi<=(int)idx_bound.y(); yi++)
    {
      float y = zone.min.y() + yi*tree->getResolution();
      for (int xi=0; xi<=(int)idx_bound.x(); xi++)
      {
        float x = zone.min.x() + xi*tree->getResolution();
        octomap::OcTreeKey key;
        bool inTree = tree->coordToKeyChecked(x,y,z, key);
        if (!inTree)
        {
          return false;
        }
        auto node = tree->search(key, tree->getTreeDepth());
        if (!node){
          return false;
        }
        if (tree->isNodeOccupied(node)){
          return false;
        }
      }
    }
  }
  return true;
}

// is a sphere with diametr free in the octotree
bool isFreeSpace(octomap::point3d center, double diameter, const std::shared_ptr<octomap::OcTree>& tree)
{
  double radius = diameter/2.0;
  int idx_bound = (int)std::ceil(radius*(1.0/tree->getResolution()));
  // octomap::point3d start = center + octomap::point3d(-radius, -radius, -radius);
  for (int zi=-idx_bound; zi<=idx_bound; zi++)
  {
    float dz =  zi*tree->getResolution();
    for (int yi=-idx_bound; yi<=idx_bound; yi++)
    {
      float dy = yi*tree->getResolution();
      for (int xi=-idx_bound; xi<=idx_bound; xi++)
      {
        float dx = xi*tree->getResolution();
        octomap::point3d dv(dx,dy,dz);
        if (dv.norm() > 1.1*radius)
        {
          continue;
        }
        octomap::OcTreeKey key;
        bool inTree = tree->coordToKeyChecked(center+dv, key);
        if (!inTree)
        {
          return false;
        }
        auto node = tree->search(key, tree->getTreeDepth());
        if (!node){
          return false;
        }
        if (tree->isNodeOccupied(node)){
          return false;
        }
      }
    }
  }
  return true;
}

// get the UAV's current position (tracker command), transformed into octree_frame
std::optional<mrs_msgs::ReferenceStamped> getPosition(
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& sh_control_manager_diag,
    mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>&            sh_tracker_cmd,
    const std::string&                                              octree_frame,
    mrs_lib::Transformer&                                           transformer,
    const std::string&                                              log_tag)
{
  const bool got_control_manager_diag = sh_control_manager_diag.hasMsg() && (ros::Time::now() - sh_control_manager_diag.lastMsgTime()).toSec() < 2.0;
  const bool got_tracker_cmd          = sh_tracker_cmd.hasMsg() && (ros::Time::now() - sh_tracker_cmd.lastMsgTime()).toSec() < 2.0;
  if (!got_control_manager_diag || !got_tracker_cmd) {
    ROS_WARN_THROTTLE(1.0, "%s: tracker not redy", log_tag.c_str());
    return {};
  }
  // ROS_WARN("%s", octree_frame.c_str());

  mrs_msgs::TrackerCommandConstPtr tracker_cmd = sh_tracker_cmd.getMsg();

  mrs_msgs::ReferenceStamped position_cmd_ref;
  position_cmd_ref.header               = tracker_cmd->header;
  position_cmd_ref.reference.position.x = tracker_cmd->position.x;
  position_cmd_ref.reference.position.y = tracker_cmd->position.y;
  position_cmd_ref.reference.position.z = tracker_cmd->position.z;
  position_cmd_ref.reference.heading    = tracker_cmd->heading;
  std::optional<mrs_msgs::ReferenceStamped> res = transformer.transformSingle(position_cmd_ref, octree_frame);
  if (!res) {
    ROS_WARN("%s: could not transform position cmd to the map frame", log_tag.c_str());
    return {};
  }
  return res;
}

}
