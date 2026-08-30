
#include "frontier_detection/frontier_manager.hpp"
#include <Eigen/Dense>
#include <algorithm>


namespace frontier_detection
{

FrontierManager::FrontierManager( double free_space_diameter,
                                  int    min_frontier_size,
                                  int    min_svd_eigen_value,
                                  double size_decrease_ratio,
                                  int    viewpoint_sample_attempts,
                                  double viewpoint_sample_radius,
                                  double viewpoint_sample_height,
                                  double viewpoint_max_distance,
                                  double viewpoint_max_angle,
                                  int    max_viewpoints_per_fr,
                                  int    min_coverage)
{
  free_space_diameter_ = free_space_diameter;
  min_frontier_size_ = min_frontier_size;
  min_svd_eigen_value_ = min_svd_eigen_value;
  size_decrease_ratio_ = size_decrease_ratio;
  viewpoint_sample_attempts_ = viewpoint_sample_attempts;
  viewpoint_sample_radius_ = viewpoint_sample_radius;
  viewpoint_sample_height_ = viewpoint_sample_height;
  viewpoint_max_distance_ = viewpoint_max_distance;
  viewpoint_max_angle_ = viewpoint_max_angle;
  max_viewpoints_per_fr_ = max_viewpoints_per_fr;
  min_coverage_ = min_coverage;

  fis_c_ = std::vector<std::shared_ptr<FIS>>(0);
  frontier_id_ = 1;
}

FrontierManager::~FrontierManager() {}

// adds forntier, if frontier is too large it splits it in two with SVD and recursivly calls addFrontiers again
void FrontierManager::addFrontier(const frontier_t& frontier)
{
  int p = 3;
  int n = frontier.size();

  // if a split could create a frontier smaller then min, dont split and create the frontier
  if (n/2 <= min_frontier_size_){
    auto fis = std::make_shared<FIS>(frontier, frontier_id_++);
    fis_c_.push_back(fis);
    return;
  }

  // matrinx for SVD split
  Eigen::MatrixXd X(n,p);
  octomap::point3d center = mean(frontier);
  for (int idx=0; idx<n; idx++)
  {
    X(idx,0) = frontier[idx].x() - center.x();
    X(idx,1) = frontier[idx].y() - center.y();
    X(idx,2) = frontier[idx].z() - center.z();
  }

  Eigen::BDCSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeFullV);
  double eig_val = svd.singularValues()(0);

  // split large(long) frontier
  if (eig_val > min_svd_eigen_value_)
  {
    auto V = svd.matrixV();
    frontier_t f0(0);
    frontier_t f1(0);
    for (int idx=0; idx<n; idx++)
    {
      if (V.row(0).dot(X.row(idx)) > 0)
      {
        f0.push_back(frontier[idx]);
      } else
      {
        f1.push_back(frontier[idx]);
      }
    }
    // ROS_WARN("[Detector] frontier of size %i with eigenval %.2f split into %zu and %zu", n, eig_val, f0.size(), f1.size());
    addFrontier(f0);
    addFrontier(f1);
  }
  else
  {
    // ROS_WARN("[Detector] added of size %i with eigenval %.2f ", n, eig_val);
    auto fis = std::make_shared<FIS>(frontier, frontier_id_++);
    fis_c_.push_back(fis);
    added_frontiers_.push_back(octomap::point3d(0,0,0));
    return;
  }
}

// removes frontiers based on isStillFrontier, increases a zone to search for frontiers in so that it contians the removed frontiers
// (new frontiers in a places of removed frontiers can be found)
// marks cells of not removed frontiers so that that space in not seareched again
void FrontierManager::removeFrontiers()
{
  invalidated_frontiers_ = std::vector<octomap::point3d>(0);
  octomap_planner_utils::AABB new_zone = zone_;

  for (int idx=fis_c_.size()-1; idx>=0; idx--)
  {
    std::shared_ptr<FIS> frontier = fis_c_[idx];
    if (octomap_planner_utils::intersect(frontier->bbx_, zone_) && frontier->valid_)
    {
      bool is_frontier = isStillFrontier(frontier->cells_);
      if (!is_frontier)
      {
        invalidated_frontiers_.push_back(octomap::point3d(0,0,0));
        frontier->valid_ = false;
        // increaese the search zone
        new_zone = octomap_planner_utils::makeUnion(new_zone, frontier->bbx_);
        // fis_c_.erase(fis_c_.begin()+ idx);
      }
    }

    if (!frontier->valid_)
    {
      fis_c_.erase(fis_c_.begin()+ idx);
    }
  }
  // set the increaesed search zone
  setZone(new_zone);

  // marks the cells of remaining frontiers in the zone so that they are not searched agian
  for (auto& frontier : fis_c_)
  {
    if (octomap_planner_utils::intersect(frontier->bbx_, zone_) && frontier->valid_)
    {
      for (auto &cell : frontier->cells_)
      {
        if (octomap_planner_utils::intersect(zone_, cell))
        {
          long closed_idx = keyToClosedIdx(tree_->coordToKey(cell));
          global_closed_[closed_idx] = true;
        }
      }
    }
  }
  return;
}

// check if frontier is stil frontier
// removes small frontiers
// removes frontier that loose more then size_decrease_ratio_*frontier.size() cells
bool FrontierManager::isStillFrontier(const frontier_t& frontier)
{
  int fr_cell_cnt = 0;
  float size = frontier.size();
  if (size < min_frontier_size_)
  {
    return false;
  }
  for (auto &cell : frontier)
  {
    if (isFrontierCell(cell)){
      fr_cell_cnt++;
    }
  }
  if (fr_cell_cnt < min_frontier_size_)
  {
    return false;
  }
  float ratio = fr_cell_cnt/size;
  return ratio >= size_decrease_ratio_;
}

// check if an unknown cell has at least one free neighbor in the closest 6 neighbothood (upd, down, loft, right, top, bottom)
bool FrontierManager::isFrontierCell(octomap::point3d cell_pos)
{
  octomap::OcTreeKey cell_key = tree_->coordToKey(cell_pos);
  octomap::OcTreeNode* node = tree_->search(cell_key, tree_->getTreeDepth());
  if (node)
  {
    return false;
  }

  for(int idx=0; idx<6; idx++)
  {
    auto neighbour_offset = octomap_planner_utils::NEIGHBOUR_OFFSETS[idx];
    octomap::OcTreeKey n_key = octomap_planner_utils::getNeighbourKey(cell_key, neighbour_offset);
    octomap::OcTreeNode* n_node = tree_->search(n_key, tree_->getTreeDepth());
    if (n_node && !tree_->isNodeOccupied(n_node))
    {
      return true;
    }
  }
  return false;
}

// helper func to check if cell was processed
long FrontierManager::keyToClosedIdx(octomap::OcTreeKey start_key)
{
  return (start_key[2]-min_key_[2])*dx_*dy_+(start_key[1]-min_key_[1])*dx_ + (start_key[0]-min_key_[0]);
}


// true if the flat index is within bounds and not yet marked closed
bool FrontierManager::canBeProcessed(long closed_idx, const std::vector<bool>& closed)
{
  return (closed_idx >= 0 && closed_idx < closed.size() && !closed[closed_idx]);
}

// sets zone for frontier search
void FrontierManager::setZone(octomap_planner_utils::AABB zone)
{
  zone_ = zone;
  min_key_ = tree_->coordToKey(zone_.min);
  max_key_ = tree_->coordToKey(zone_.max);

  dx_ = max_key_[0] - min_key_[0] + 1;
  dy_ = max_key_[1] - min_key_[1] + 1;
  dz_ = max_key_[2] - min_key_[2] + 1;
  global_closed_ = std::vector<bool>(dx_*dy_*dz_, false);
  return;
}

// counts how many frontier cells are within max distance/angle of the viewpoint and reachable by an unoccluded ray (raycast through the octree)
int FrontierManager::viewpointCoverage(octomap::point3d viewpoint_pos, const frontier_t& frontier_cells)
{
  int coverage = 0;

  // check how many FR cells are visible from viewpoint
  for (auto &cell : frontier_cells)
  {
    if (cell.distance(viewpoint_pos) > viewpoint_max_distance_
          || std::abs((cell-viewpoint_pos).normalized().z()) > std::sin(viewpoint_max_angle_)) // 0.7 for 90 degree horizontal fow
    {
      continue;
    }
    bool isVisible = true;
    octomap::KeyRay key_ray;
    tree_->computeRayKeys(viewpoint_pos, cell, key_ray);

    for (octomap::KeyRay::iterator it1 = key_ray.begin(), end = key_ray.end(); it1 != end; ++it1)
    {
      auto node = tree_->search(*it1, tree_->getTreeDepth());
      if (node && tree_->isNodeOccupied(node)){
        isVisible = false;
        break;
      }
    }
    if (isVisible)
    {
      coverage += 1;
    }
  }

  return coverage;
}


// make viewpoint for a frontier
void FrontierManager::makeViewpoints(std::shared_ptr<FIS> fis)
{
  for (int i=0; i<viewpoint_sample_attempts_; i++)
  {
    octomap::point3d viewpoint_pos = fis->sampleViewpoint(viewpoint_sample_radius_, viewpoint_sample_height_);
    if (!octomap_planner_utils::intersect(zone_, viewpoint_pos)){
      continue;
    }
    // viewpoint must be in free space
    auto node = tree_->search(tree_->coordToKey(viewpoint_pos), tree_->getTreeDepth());
    if (!node || tree_->isNodeOccupied(node)){
      continue;
    }

    // viewpoint must have free space around
    if (octomap_planner_utils::isFreeSpace(viewpoint_pos, free_space_diameter_, tree_))
    {
      int coverage = viewpointCoverage(viewpoint_pos, fis->cells_);
      if (coverage >= min_coverage_)
      {
        viewpoint_t viewpoint = {.position=viewpoint_pos, .coverage=coverage};
        fis->viewpoints_.push_back(viewpoint);
      }
    }

    if (fis->viewpoints_.size() >= max_viewpoints_per_fr_) break;
  }
  // sort viewpoints by their coverage (how many FR cells they can see)
  std::sort(fis->viewpoints_.begin(), fis->viewpoints_.end(), [](viewpoint_t a, viewpoint_t b){return a.coverage > b.coverage;});

}

// removes old frontiers, serches for new ones and ads them, also creates viewpoints
// classical BFS search, checks if cell was alredy explored or is a part of another forntier
// updates only frontiers in zone that is visible by lidar and that contiants removed fromtiers
void FrontierManager::processNewMap(const std::shared_ptr<octomap::OcTree>& tree, octomap_planner_utils::AABB zone, octomap::OcTreeKey start_key)
{

    ROS_INFO("Processing frontiers");
    tree_ = tree;
    setZone(zone);

    removeFrontiers();

    octomap::point3d start_pos = tree_->keyToCoord(start_key);
    std::queue<octomap::OcTreeKey> qu;
    qu.push(start_key);

    std::vector<frontier_t> new_frontiers(0);

    while(qu.size() > 0)
    {

      octomap::OcTreeKey current_key = qu.front();
      qu.pop();

      for(auto &neighbour_offset : octomap_planner_utils::NEIGHBOUR_OFFSETS)
      {
        octomap::OcTreeKey n_key = octomap_planner_utils::getNeighbourKey(current_key, neighbour_offset);
        octomap::point3d n_pos = tree_->keyToCoord(n_key);

        if (!octomap_planner_utils::intersect(zone_, n_pos))
        {
          continue;
        }
        long closed_idx = keyToClosedIdx(n_key);

        if (!canBeProcessed(closed_idx, global_closed_))
        {
          continue;
        }

        global_closed_[closed_idx] = true;

        octomap::OcTreeNode* n_node = tree_->search(n_key, tree_->getTreeDepth());

        // if cell is unknown and neighbors free cell, do second BFS to extract a frontier
        if (!n_node)
        {
            // found frontier cell;
            std::vector<bool> frontier_closed(dx_*dy_*dz_, false);
            frontier_t frontier_cells(0);
            std::queue<octomap::OcTreeKey> frontier_qu;
            frontier_qu.push(n_key);
            // ---------------------- traversing frontier ---------------------//
            while(frontier_qu.size()>0)
            {
              octomap::OcTreeKey current_fr_key = frontier_qu.front();
              frontier_qu.pop();

              closed_idx = keyToClosedIdx(current_fr_key);

              if (!canBeProcessed(closed_idx, frontier_closed))
              {
                continue;
              }
              frontier_closed[closed_idx] = true;
              global_closed_[closed_idx] = true;
              bool is_frontier_cell = false;
              std::vector<octomap::OcTreeKey> unk_neighbors(0);
              int d_neighbor_idx = -1;
              for(auto &neighbour_offset : octomap_planner_utils::NEIGHBOUR_OFFSETS)
              {
                d_neighbor_idx++;
                octomap::OcTreeKey fn_key = octomap_planner_utils::getNeighbourKey(current_fr_key, neighbour_offset);
                octomap::point3d fn_pos = tree_->keyToCoord(fn_key);
                if (!octomap_planner_utils::intersect(zone_, fn_pos))
                {
                  continue;
                }
                closed_idx = keyToClosedIdx(fn_key);
                octomap::OcTreeNode* fn_node = tree_->search(fn_key, tree_->getTreeDepth());
                if (!fn_node)
                {
                  if (canBeProcessed(closed_idx, frontier_closed) && canBeProcessed(closed_idx, global_closed_))
                  {
                    unk_neighbors.push_back(fn_key);
                  }
                }
                else if (!tree_->isNodeOccupied(fn_node) && d_neighbor_idx < 6 && !is_frontier_cell) // diorect neighbor cell is free, and current cell wasnt added yet
                {
                  is_frontier_cell = true;
                  frontier_cells.push_back(tree_->keyToCoord(current_fr_key));
                }
              }
              if (is_frontier_cell)
              {
                for (auto &cell : unk_neighbors)
                {
                  frontier_qu.push(cell);
                }
              }
            }
            if (frontier_cells.size() >= min_frontier_size_){
              new_frontiers.push_back(frontier_cells);
            }
        }
        else if (! tree_->isNodeOccupied(n_node))
        {
          qu.push(n_key);
        }

      }
    }


    int tmp = fis_c_.size();
    added_frontiers_ = std::vector<octomap::point3d>(0);
    for (auto& frontier : new_frontiers)
    {
      addFrontier(frontier);
    }
    viewable_frontier_cnt_ = 0;

    for (auto& fis : fis_c_)
    {
      // ROS_ERROR("id %u is %s with %zu viewpoints and size %zu", fis->id_, fis->valid_  ? "valid" : "invalid", fis->viewpoints_.size(), fis->cellCnt());
      if (!fis->valid_) {
        continue;
      };
      int viewpoint_cnt = fis->viewpoints_.size();

      if (octomap_planner_utils::intersect(fis->bbx_, zone_)){
        if (viewpoint_cnt == 0)
        {
          makeViewpoints(fis);
        }
        for (int vidx = fis->viewpoints_.size()-1; vidx >= 0; vidx--)
        {
          int new_coverage = viewpointCoverage(fis->viewpoints_[vidx].position, fis->cells_);
          if (new_coverage < min_coverage_)
          {
            fis->viewpoints_.erase(fis->viewpoints_.begin() + vidx);
          }
          else
          {
            fis->viewpoints_[vidx].coverage = new_coverage;
          }
        }
        std::sort(fis->viewpoints_.begin(), fis->viewpoints_.end(), [](viewpoint_t a, viewpoint_t b){return a.coverage > b.coverage;});
      }

      if (fis->viewpoints_.size() > 0){
        // added_frontiers_.push_back(fis->viewpoints_[0].position);
      }
      else{
        fis->valid_ = false;
        continue;
      }

      viewable_frontier_cnt_ += fis->viewpoints_.size() == 0 ? 0 : 1;
    }

    ROS_INFO("[FrontierDetection]: final frontier count: %zu/%zu/%zu (total/inv/add)", fis_c_.size(), invalidated_frontiers_.size(), added_frontiers_.size());
}

}
