
#include <vector>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include "prm.hpp"


namespace mrs_octomap_planner
{

PRM::PRM(){}

PRM::PRM( std::shared_ptr<mrs_lib::BatchVisualizer> bv_planner, 
          double                                    free_space_diameter, 
          double                                    overlap_coefficient, 
          double                                    resample_factor,
          int                                       node_max_age)
{
  bv_prm_ = bv_planner;
  cost_matrix_ = Eigen::MatrixXd(32,32);
  nodes_ = std::vector<std::shared_ptr<node_t>>(0);
  node_cnt_ = 0;
  frozen_path_ = std::vector<bool>(32, false); 
  resample_factor_ = resample_factor;
  max_cost_ = 1000000.0;
  node_max_age_ = node_max_age;
  neighbor_overlap_ = overlap_coefficient;
  free_space_diameter_ = free_space_diameter;
}

PRM::~PRM()
{
}

void PRM::updateZone(const std::shared_ptr<octomap::OcTree>& tree, AABB zone, bool map_update)
{
  tree_ = tree;
  float sample_cnt = resample_factor_*volume(zone);
  int invalids = 0;

  for (auto &node : nodes_){

    // node->age++;
    if (intersect(zone, node->position)){
      if (node->age++ > node_max_age_/(node->neighbors.size()+1) \
            || (map_update && !isFreeSpace(node->position, free_space_diameter_, tree)))
      {
        invalids++;
        node->valid = false;
      }
      else 
      {
        sample_cnt-=1.0;
      }
    }
  }
  removeInvalidNodes();

  int i = 0;
  while ( i < sample_cnt)
  {
    octomap::point3d sample = getSampleFromAABB(zone);
    if (isFreeSpace(sample, free_space_diameter_, tree))
    {
      addNode(sample);
    }
    i++;
  }
  for (auto &node : nodes_){
    bv_prm_->addPoint(Eigen::Vector3d(node->position.x(),node->position.y(),node->position.z()));
    for (auto &neighbor_p : node->neighbors){
      
      if (auto neighbor = neighbor_p.lock())
        bv_prm_->addRay(mrs_lib::geometry::Ray( Eigen::Vector3d(node->position.x(), node->position.y(), node->position.z()),
                                                    Eigen::Vector3d(neighbor->position.x(), neighbor->position.y(), neighbor->position.z())),
                                                    1.0, 0.0, 0.0, 0.1);
    }
  }
  ROS_INFO_THROTTLE(1.0,"[MrsExplorer]: number of PRM nodes: %i", nodes_.size());
}



void PRM::removeInvalidNodes()
{
  node_cnt_ = nodes_.size();
  int back_idx = node_cnt_-1;
  // std::vector<bool> visited_neighbors(node_cnt_, false);
  for (unsigned long i=0;i<node_cnt_; i++){
    auto node = nodes_[i];
    auto ne = std::remove_if(node->neighbors.begin(), node->neighbors.end(), 
                  [node](std::weak_ptr<node_t> neighbor_p){
                              return neighbor_p.expired() || !neighbor_p.lock()->valid;});
          node->neighbors.erase(ne, node->neighbors.end());

    if (!node->valid)
    {
      while (back_idx>i && !nodes_[back_idx]->valid) {
        back_idx--;
      }
      std::swap(nodes_[i], nodes_[back_idx]);
      nodes_[i]->idx = i;
      // cost_matrix_.col(i).swap(cost_matrix_.col(back_idx));
      // cost_matrix_.row(i).swap(cost_matrix_.row(back_idx));
      back_idx--;
      node_cnt_--;
    }
  }
  nodes_.erase(nodes_.begin()+node_cnt_, nodes_.end());

}


void PRM::addNode(octomap::point3d position)
{

  
  std::shared_ptr<node_t> node(new node_t);
  node->position = position;
  node->neighbors=std::vector<std::weak_ptr<node_t>>(0),
  node->idx = nodes_.size(),

  nodes_.push_back(node);
  // node = nodes_.back();

  for (int i=0; i<nodes_.size()-1; i++)
  {
    auto neighbor = nodes_[i];
    double dist = node->position.distance(neighbor->position);
    if (dist <= free_space_diameter_*neighbor_overlap_)
    {
      node->neighbors.push_back(neighbor);
      neighbor->neighbors.push_back(node);
    }
    else if (dist <= 2*free_space_diameter_*neighbor_overlap_)
    {
      if (isFreeSpace(node->position + (neighbor->position - node->position)*(0.5/dist), free_space_diameter_, tree_))
      {
        node->neighbors.push_back(neighbor);
        neighbor->neighbors.push_back(node);
      }
      
    }
  }

} 


std::vector<node_t> PRM::findCloseNode(octomap::point3d point, double r)
{
  std::vector<node_t> close_points(0);
  for (auto &node : nodes_)
  {
    if (point.distance(node->position) <= r)
    {
      close_points.push_back(*node);
    }
  }
  
  std::sort(close_points.begin(), close_points.end(), [point](node_t a, node_t b){return point.distance(a.position) < point.distance(b.position);});

  return close_points;
}


path_t PRM::findNodePath(node_t start, node_t goal)
{
  
  auto h = [goal](node_t* n)
  {
    return (float)goal.position.distance(n->position);
  };

  unsigned long NO_PARENT = std::numeric_limits<unsigned long>::max();
  std::vector<unsigned long> parents(nodes_.size(), NO_PARENT);
  std::vector<float> g_score(nodes_.size(), std::numeric_limits<float>::max());
  std::vector<float> f_score(nodes_.size(), std::numeric_limits<float>::max());
  g_score[start.idx] = 0.0;
  f_score[start.idx] = h(&start);

  std::vector<node_t> path(0);
  std::priority_queue<a_start_node_t, std::vector<a_start_node_t>, customLess> open_q;
  open_q.push(a_start_node_t{.idx=start.idx, .f=f_score[start.idx]});
  

  while(open_q.size() > 0)
  {
    auto curr_node =  open_q.top();
    open_q.pop();

    unsigned long curr_idx = curr_node.idx;
    float curr_f = curr_node.f;
  
    if (f_score[curr_idx] < curr_f)
    {
      continue;
    }

    if (curr_idx == goal.idx )
    {
      path.push_back(*nodes_[curr_idx]);
      while (parents[curr_idx] != NO_PARENT)
      {
        curr_idx = parents[curr_idx];
        path.push_back(*nodes_[curr_idx]);
      }
      std::reverse(path.begin(), path.end());
      return path_t{.nodes=path, .length=g_score[goal.idx]};
    }

    for (auto &child_p : nodes_[curr_idx]->neighbors)
    {
      if (auto child = child_p.lock())
      {

        float cost = g_score[curr_idx] + child->position.distance(nodes_[curr_idx]->position);
        if (cost < g_score[child->idx])
        {
          parents[child->idx] = curr_idx;
          g_score[child->idx] = cost;
          f_score[child->idx] = cost + h(child.get());

          open_q.push(a_start_node_t{.idx=child->idx, .f=f_score[child->idx]});
        }   
      }
    }
  }
  return path_t{.nodes=path, .length=0};
}


std::vector<octomap::point3d> PRM::findPath(octomap::point3d start, octomap::point3d goal, octomath::Vector3 velocity)
{
  std::vector<octomap::point3d> path(0);
  // if (!isFreeSpace(aabbFromCenter(start, free_space_diameter_/2, free_space_diameter_/2, free_space_diameter_/2), tree_))
  // {
  //   ROS_WARN("search start not in free space");
  //   return path;
  // }
  // if (!isFreeSpace(goal, free_space_diameter_, tree_))
  // {
  //   ROS_WARN("search goal not in free space");
  //   return path;
  // }

  auto start_candidates = findCloseNode(start, free_space_diameter_);
  auto goal_candidates  = findCloseNode(goal,  free_space_diameter_);

  if (start_candidates.size() == 0)
  {
    ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: search start is far from nav nodes");
    return path;
  }
  if (goal_candidates.size() == 0)
  {
    ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: search goal is far from nav nodes");
    return path;
  }
  if (velocity.norm() > 0.001)
  {
    std::sort(start_candidates.begin(), start_candidates.end(), [start, velocity](node_t a, node_t b){return velocity.dot(a.position-start) > velocity.dot(b.position-start);});
  }
  // std::sort(goal_candidates.begin(),  goal_candidates.end(),  [goal,  velocity](node_t a, node_t b){return velocity.dot(goal-a.position ) > velocity.dot(goal-b.position );});

  auto node_path = findNodePath(start_candidates[0], goal_candidates[0]);
  if (node_path.nodes.size() == 0)
  {
    ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: A* found no path");
    return path;
  }
  frozen_path_ = std::vector<bool>(nodes_.size(), false);

  path.push_back(start);
  for (auto &node : node_path.nodes)
  {
    frozen_path_[node.idx] = true;
    path.push_back(node.position);
    // nodes_[node.idx]->age -= nodes_[node.idx]->age > 0 ? 1 : 0;
  }
  path.push_back(goal);
  // ROS_ERROR("%f", node_path.length);

  return path;
}



std::vector<octomap::point3d> PRM::simplifyRaycastPath(std::vector<octomap::point3d> path)
{
  std::vector<octomap::point3d> simplified_path(0);
  int current = 0;
  int last_good  = 1;
  int next       = 1;
  
  simplified_path.push_back(path[current]);

  while (next != path.size()-1)
  {
    bool obstacle = false;
    octomap::KeyRay key_ray;
    tree_->computeRayKeys(path[current], path[next], key_ray);

    for (octomap::KeyRay::iterator it1 = key_ray.begin(), end = key_ray.end(); it1 != end; ++it1) 
    {
      auto node = tree_->search(*it1, tree_->getTreeDepth());
      if (!node || tree_->isNodeOccupied(node)){
        obstacle = true;
        break;
      }
    }

    if (obstacle)
    {
      simplified_path.push_back(path[last_good]);
      current = last_good;
      last_good = current + 1;
      next = current + 1;
    }
    else
    {
      last_good = next;
      next +=1;
    }
  }
  simplified_path.push_back(path[next]);
  return simplified_path;
}

std::vector<octomap::point3d> PRM::simplifyFreeSpacePath(std::vector<octomap::point3d> path)
{
  std::vector<octomap::point3d> simplified_path(0);
  std::vector<octomap::point3d> last_segment_division;
  int current = 0;
  int last_good  = 1;
  int next       = 1;
  
  simplified_path.push_back(path[current]);

  while (next != path.size()-1)
  {
    bool obstacle = false;

    octomath::Vector3 segment = path[next] - path[current];
    int div_cnt = (int)std::ceil(segment.norm()/free_space_diameter_/neighbor_overlap_);

    octomath::Vector3 delta = segment*(1.0/div_cnt);
    octomath::Vector3 temp_point = path[current];
    for (int i=1; i < div_cnt; i++)
    {
      temp_point += delta;
      if (!isFreeSpace(temp_point, free_space_diameter_, tree_))
      {
        obstacle = true;
        break;
      }
    }
    if (!obstacle)
    {
      last_segment_division = std::vector<octomap::point3d>(0);
      temp_point = path[current];
      for (int i=1; i <= div_cnt; i++)
      {
        temp_point += delta;
        last_segment_division.push_back(temp_point);
      }
    }      

    if (obstacle)
    {
      for (auto segment : last_segment_division)
      { 
        simplified_path.push_back(segment);
      }
      last_segment_division = std::vector<octomap::point3d>(0);
      
      current = last_good;
      last_good = current + 1;
      next = current + 1;
    }
    else
    {
      last_good = next;
      next +=1;
    }
  }
  simplified_path.push_back(path[next]);
  return simplified_path;
}


}