#include "octomap_planner_utils/utils.hpp"


namespace octomap_planner_utils
{

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

octomap::OcTreeKey getNeighbourKey(octomap::OcTreeKey key, const NeighbourOffset& neighbour_offset){
  octomap::OcTreeKey new_key;
  new_key.k[0] = key.k[0] + neighbour_offset.dx;
  new_key.k[1] = key.k[1] + neighbour_offset.dy;
  new_key.k[2] = key.k[2] + neighbour_offset.dz;
  return new_key;
}

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

std::array<octomap::point3d, 8> getConrners(AABB a){
  std::array<octomap::point3d, 8> l;

  int i = 0;
  for (auto dx : {a.max.x(), a.min.x()}){
    for (auto dy : {a.max.y(), a.min.y()}){
      for (auto dz : {a.max.z(), a.min.z()}){
        l[i++] = octomap::point3d(dx, dy, dz);
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

AABB aabbFromCenter(octomap::point3d c, double x, double y,double z){
  AABB b;
  b.min = octomap::point3d(c.x()-x/2, c.y()-y/2,c.z()-z/2);
  b.max = octomap::point3d(c.x()+x/2, c.y()+y/2,c.z()+z/2);
  return b;
}

AABB localZoneFromPosition(octomap::point3d position, AABB flight_zone, double width, double height)
{
  AABB local_zone = aabbFromCenter(position, width, width, height);
  return makeIntersection(local_zone, flight_zone);
}

AABB localZoneFromPosition(geometry_msgs::Point position, AABB flight_zone, double width, double height)
{
  return localZoneFromPosition(pointToOctomap(position), flight_zone, width, height);
}

octomap::point3d pointToOctomap(const geometry_msgs::Point& p)
{
  return octomap::point3d(p.x, p.y, p.z);
}

geometry_msgs::Point octomapToPoint(const octomap::point3d& p)
{
  geometry_msgs::Point res;
  res.x = p.x();
  res.y = p.y();
  res.z = p.z();
  return res;
}

AABB aabbSmartFromCenter(octomap::point3d c, double x, double y,double z, double floor){
  AABB res = aabbFromCenter(c,x,y,z);

  if (c.z() - (z/2.0) < floor){
    res.min = octomap::point3d(res.min.x(), res.min.y(), floor);
  }
  return res;
}

AABB makeUnion(AABB a, AABB b)
{
  octomap::point3d min(std::min(a.min.x(), b.min.x()), std::min(a.min.y(), b.min.y()), std::min(a.min.z(), b.min.z()));
  octomap::point3d max(std::max(a.max.x(), b.max.x()), std::max(a.max.y(), b.max.y()), std::max(a.max.z(), b.max.z()));

  AABB u = {.min=min, .max=max};
  return u;
}

AABB makeIntersection(AABB a, AABB b)
{
  octomap::point3d min(std::max(a.min.x(), b.min.x()), std::max(a.min.y(), b.min.y()), std::max(a.min.z(), b.min.z()));
  octomap::point3d max(std::min(a.max.x(), b.max.x()), std::min(a.max.y(), b.max.y()), std::min(a.max.z(), b.max.z()));

  AABB u = {.min=min, .max=max};
  return u;
}

bool isSubset(AABB a, AABB b)
{
  return isBiggerEq(a.min, b.min) && isSmallerEq(a.max, b.max);
}

bool intersect(AABB bbx0, AABB bbx1){
  // standard separating-axis AABB overlap test: the boxes overlap iff they overlap on every axis
  return isSmallerEq(bbx0.min, bbx1.max) && isBiggerEq(bbx0.max, bbx1.min);
}

bool intersect(AABB bbx0, octomap::point3d p){

    if (isSmallerEq(bbx0.min, p) && isBiggerEq(bbx0.max,p))
    {
      return true;
    }

  return false;
}

float getRand()
{
  // thread_local, seeded-per-thread engine: avoids libc rand()'s shared global state (unseeded => deterministic
  // across runs, and unsafe to call concurrently from path_planning/frontier_detection/tsp_solver's own threads)
  thread_local std::mt19937 generator(std::random_device{}());
  thread_local std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
  return distribution(generator);
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

octomap::point3d getSampleFromAABB(AABB a)
{
  return octomap::point3d(getRand(a.min.x(), a.max.x()),
                          getRand(a.min.y(), a.max.y()),
                          getRand(a.min.z(), a.max.z()));
}

namespace
{
  // descends from node towards key, creating missing children along the way, and returns the node reached at target_depth
  // (or at the tree's max depth if target_depth is 0); ported from mrs_octomap_server's touchNodeRecurs
  octomap::OcTreeNode* touchNodeRecurs(octomap::OcTree& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key,
                                        unsigned int depth, unsigned int target_depth)
  {
    if (depth < octree.getTreeDepth() && (target_depth == 0 || depth < target_depth))
    {
      unsigned int pos = octomap::computeChildIdx(key, int(octree.getTreeDepth() - depth - 1));
      if (!octree.nodeChildExists(node, pos))
      {
        octree.createNodeChild(node, pos);
      }
      return touchNodeRecurs(octree, octree.getNodeChild(node, pos), key, depth + 1, target_depth);
    }
    else
    {
      return node;
    }
    
  }

  octomap::OcTreeNode* touchNode(octomap::OcTree& octree, const octomap::OcTreeKey& key, unsigned int target_depth = 0)
  {
    return touchNodeRecurs(octree, octree.getRoot(), key, 0, target_depth);
  }
}

void mergeInto(const octomap::OcTree& from, octomap::OcTree& to)
{

  if (!to.getRoot())
  {
    octomap::OcTreeKey root_key = to.coordToKey(0, 0, 0, to.getTreeDepth());
    to.setNodeValue(root_key, octomap::logodds(0.0));
  }

  for (auto it = from.begin_leafs(), end = from.end_leafs(); it != end; ++it)
  {
    octomap::OcTreeKey    key  = it.getKey();
    octomap::OcTreeNode*  node = touchNode(to, key, it.getDepth());
    node->setValue(it->getValue());
  }
}

bool isFreeSpace(AABB zone, const octomap::OcTree& tree)
{
  // walk only the leaves overlapping zone (native leaf_bbx_iterator) instead of descending the tree per fine voxel;
  // a coarse free/occupied leaf covering many voxels is then visited once instead of once per voxel it contains.
  // unknown space leaves no leaf behind, so "every voxel known and free" is checked by requiring the known leaves'
  // volume (clipped to zone) to add up to the full zone volume.
  const double zone_volume = static_cast<double>(volume(zone));
  double       known_volume = 0.0;

  for (auto it = tree.begin_leafs_bbx(zone.min, zone.max), end = tree.end_leafs_bbx(); it != end; ++it)
  {
    if (tree.isNodeOccupied(*it))
    {
      return false;
    }

    const float half_size = static_cast<float>(it.getSize() / 2.0);
    const octomap::point3d leaf_min = it.getCoordinate() - octomap::point3d(half_size, half_size, half_size);
    const octomap::point3d leaf_max = it.getCoordinate() + octomap::point3d(half_size, half_size, half_size);
    known_volume += static_cast<double>(volume(makeIntersection(AABB{.min=leaf_min, .max=leaf_max}, zone)));
  }

  return known_volume >= zone_volume - 1e-6;
}

bool isFreeSpace(octomap::point3d center, double diameter, const octomap::OcTree& tree)
{
  double radius = diameter/2.0;
  double resolution = tree.getResolution();
  int idx_bound = (int)std::ceil(radius*(1.0/resolution));
  for (int zi=-idx_bound; zi<=idx_bound; zi++)
  {
    float dz =  zi*resolution;
    for (int yi=-idx_bound; yi<=idx_bound; yi++)
    {
      float dy = yi*resolution;
      for (int xi=-idx_bound; xi<=idx_bound; xi++)
      {
        float dx = xi*resolution;
        octomap::point3d dv(dx,dy,dz);
        if (dv.norm() > 1.1*radius)
        {
          continue;
        }
        octomap::OcTreeKey key;
        bool inTree = tree.coordToKeyChecked(center+dv, key);
        if (!inTree)
        {
          return false;
        }
        auto node = tree.search(key, tree.getTreeDepth());
        if (!node){
          return false;
        }
        if (tree.isNodeOccupied(node)){
          return false;
        }
      }
    }
  }
  return true;
}

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

std::optional<mrs_msgs::MpcPredictionFullState> getFullStatePrediction(
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& sh_control_manager_diag,
    mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>&            sh_tracker_cmd,
    const std::string&                                              octree_frame,
    mrs_lib::Transformer&                                           transformer,
    const std::string&                                              log_tag)
{
  const bool got_control_manager_diag = sh_control_manager_diag.hasMsg() && (ros::Time::now() - sh_control_manager_diag.lastMsgTime()).toSec() < 2.0;
  const bool got_tracker_cmd          = sh_tracker_cmd.hasMsg() && (ros::Time::now() - sh_tracker_cmd.lastMsgTime()).toSec() < 2.0;
  if (!got_control_manager_diag || !got_tracker_cmd)
  {
    ROS_WARN_THROTTLE(1.0, "%s: could not get controller prediction", log_tag.c_str());
    return {};
  }

  mrs_msgs::MpcPredictionFullState prediction = sh_tracker_cmd.getMsg()->full_state_prediction;
  auto ret = transformer.getTransform(prediction.header.frame_id, octree_frame, prediction.header.stamp);

  if (!ret) {
    ROS_WARN_THROTTLE(1.0, "%s: could not transform position cmd to the map frame! can not check for potential collisions!", log_tag.c_str());
    return {};
  }

  const geometry_msgs::TransformStamped& tf = ret.value();
  for (auto& p : prediction.position)
  {
    const auto transformed = transformer.transform(p, tf);
    if (!transformed) {
      ROS_WARN_THROTTLE(1.0, "%s: could not transform prediction position to the map frame! can not check for potential collisions!", log_tag.c_str());
      return {};
    }
    p = transformed.value();
  }
  for (auto& v : prediction.velocity)
  {
    const auto transformed = transformer.transform(v, tf);
    if (!transformed) {
      ROS_WARN_THROTTLE(1.0, "%s: could not transform prediction velocity to the map frame! can not check for potential collisions!", log_tag.c_str());
      return {};
    }
    v = transformed.value();
  }
  prediction.header.frame_id = octree_frame;
  return prediction;
}

}
