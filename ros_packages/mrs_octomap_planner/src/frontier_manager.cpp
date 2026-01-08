
#include "frontier_manager.hpp"



namespace mrs_octomap_planner
{

FrontierManager::FrontierManager( const std::shared_ptr<mrs_lib::BatchVisualizer>&  bv_frontiers,
                                  double                                            free_space_diameter,
                                  int                                               min_frontier_size,
                                  int                                               min_svd_eigen_value,
                                  double                                            size_decrease_ratio,
                                  int                                               viewpoint_sample_attempts,
                                  double                                            viewpoint_sample_radius,
                                  double                                            viewpoint_sample_height,
                                  double                                            viewpoint_max_distance,
                                  double                                            viewpoint_max_angle,
                                  int                                               max_viewpoints_per_fr,
                                  int                                               min_viewpoints_per_fr,
                                  double                                            resample_probability, 
                                  int                                               min_coverage)
{
  bv_frontiers_ = bv_frontiers;
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
  min_viewpoints_per_fr_ = min_viewpoints_per_fr;
  resample_probability_ = resample_probability; 
  min_coverage_ = min_coverage;
  
  fis_c_ = std::vector<std::unique_ptr<FIS>>(0);
  frontier_id_ = 0;
}

// adds forntier, if frontier is too large it splits it in two with SVD and recursivly calls addFrontiers again
void FrontierManager::addFrontier(const frontier_t& frontier)
{
  // ROS_INFO("adding frontier");
  
  int p = 3;
  int n = frontier.size();

  // if a split cound create a frontier smaller then min, dont split and create the frontier
  if (n/2.0 <= min_frontier_size_){
    auto fis = std::make_unique<FIS>(frontier, frontier_id_++);
    added_frontiers_idx_.push_back(fis->id_);
    fis_c_.push_back(std::move(fis));
    return;
  }

  // matrinx for SVD split
  Eigen::MatrixXd X(n,p);
  octomap::point3d center = mean(frontier);
  for (int idx=0; idx<frontier.size(); idx++)
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
    addFrontier(f0);
    addFrontier(f1);
  }
  else
  {
    auto fis = std::make_unique<FIS>(frontier, frontier_id_++);
    added_frontiers_idx_.push_back(fis->id_);
    fis_c_.push_back(std::move(fis));
    return;
  }
}

// remodes frontiers based on isStillFrontier, increases a zone to search for frontiers in so that it contians the removed frontiers 
// (new frontiers in a paces of removed frontiers can be found)
// marks cells of not removed frontiers so that that space in not seareched again
void FrontierManager::removeFrontiers()
{
  // ROS_INFO("removing frontier");
  removed_frontiers_ = std::vector<int>(0);
  AABB new_zone = zone_;

  for (int idx=fis_c_.size()-1; idx>=0; idx--)
  {
    FIS* frontier  = fis_c_[idx].get();
    if (intersect(frontier->bbx_, zone_))
    {
      bool is_frontier = isStillFrontier(frontier->cells_);
      if (!is_frontier)
      {
        removed_frontiers_.push_back(frontier->id_);
        // increaese the search zone
        new_zone = makeUnion(new_zone, frontier->bbx_);
        fis_c_.erase(std::next(fis_c_.begin(),idx));
      }
    }
  }
  // set the increaesed search zone
  setZone(new_zone);

  // marks the cells of remaining frontiers in the zone so that they are not searched agian
  for (auto& frontier : fis_c_)
  {
    if (intersect(frontier->bbx_, zone_))
    {
      for (auto &cell : frontier->cells_)
      {
        if (intersect(zone_, cell))
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
    auto neighbour_offset = NEIGHBOUR_OFFSETS[idx];
    octomap::OcTreeKey n_key = getNeighbourKey(cell_key, neighbour_offset);
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


bool FrontierManager::canBeProcessed(long closed_idx, const std::vector<bool>& closed)
{
  return (closed_idx >= 0 && closed_idx < closed.size() && !closed[closed_idx]);
}

// sets zone for frontier search
void FrontierManager::setZone(AABB zone)
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

// make viewpoint for a frontier
void FrontierManager::makeViewpoints(FIS* fis)
{
  for (int i=0; i<viewpoint_sample_attempts_; i++)
  {
    octomap::point3d viewpoint_pos = fis->sampleViewpoint(viewpoint_sample_radius_, viewpoint_sample_height_);
    if (!intersect(zone_, viewpoint_pos)){
      continue;
    }
    // viewpoint must be in free space
    auto node = tree_->search(tree_->coordToKey(viewpoint_pos), tree_->getTreeDepth());
    if (!node || tree_->isNodeOccupied(node)){
      continue;
    }

    // viewpoint must have free space around
    if (isFreeSpace(viewpoint_pos, free_space_diameter_, tree_))
    {
      int coverage = 0;

      // check how many FR cells are visible from viewpoint
      for (auto &cell : fis->cells_)
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
          fis->visible_cells_.push_back(cell);
          coverage += 1;
        }
      }
      
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
void FrontierManager::processNewMap(const std::shared_ptr<octomap::OcTree>& tree, AABB zone, octomap::OcTreeKey start_key)
{
    // ROS_INFO("processing new map");
    tree_ = tree;
    setZone(zone);
  
    removeFrontiers();

    octomap::point3d start_pos = tree_->keyToCoord(start_key);
    std::queue<octomap::OcTreeKey> qu;
    qu.push(start_key);

    std::vector<frontier_t> all_frontiers(0);

    while(qu.size() > 0)
    {

      octomap::OcTreeKey current_key = qu.front();
      qu.pop();

      for(auto &neighbour_offset : NEIGHBOUR_OFFSETS)
      {
        octomap::OcTreeKey n_key = getNeighbourKey(current_key, neighbour_offset);
        octomap::point3d n_pos = tree_->keyToCoord(n_key);

        if (!intersect(zone_, n_pos))
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
            
              closed_idx = keyToClosedIdx(current_fr_key); //(current_fr_key[2]-min_key[2])*dx_*dy_+(current_fr_key[1]-min_key[1])*dx_ + (current_fr_key[0]-min_key[0]);

              if (!canBeProcessed(closed_idx, frontier_closed))
              {
                continue;
              }
              frontier_closed[closed_idx] = true;
              global_closed_[closed_idx] = true;
              bool is_frontier_cell = false;
              std::vector<octomap::OcTreeKey> unk_neighbors(0);
              int d_neighbor_idx = -1;
              for(auto &neighbour_offset : NEIGHBOUR_OFFSETS)
              {
                d_neighbor_idx++;
                octomap::OcTreeKey fn_key = getNeighbourKey(current_fr_key, neighbour_offset);
                octomap::point3d fn_pos = tree_->keyToCoord(fn_key);
                if (!intersect(zone_, fn_pos))
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
              all_frontiers.push_back(frontier_cells);
            }
        }
        else if (! tree_->isNodeOccupied(n_node))
        {
          qu.push(n_key); 
        }
        
      }
    }

    
    int tmp = fis_c_.size();
    added_frontiers_idx_ = std::vector<int>(0);
    for (auto& frontier : all_frontiers)
    { 
      addFrontier(frontier);
    }
    viewable_frontier_cnt_ = fis_c_.size();

    int cidx = 0;
    for (auto& fis : fis_c_)
    {
      if (fis->viewpoints_.size() < max_viewpoints_per_fr_)
      {
        // if forntier doesnt have enough viewpoint try to search for them
        if (fis->viewpoints_.size() < min_viewpoints_per_fr_)
        {
          makeViewpoints(fis.get());
        }
        // if forntier doesnt have all viewpoint try to search for them with some probability
        else if (getRand()<resample_probability_)
        {
          makeViewpoints(fis.get());
        }
          
      }
      viewable_frontier_cnt_ -= fis->viewpoints_.size() == 0 ? 1 : 0;
      
      // draw frontiers in different colors
      color_t color = getColor(cidx++);
      bv_frontiers_->addPoint(Eigen::Vector3d(fis->center_.x(),fis->center_.y(),fis->center_.z()), color.r,color.g,color.b, 1.0);
      if (fis->viewpoints_.size() > 0)
      {
        viewpoint_t v = fis->viewpoints_[0];
        if (v.coverage >= min_coverage_)
        {
          Eigen::Vector3d           center(v.position.x(), v.position.y(), v.position.z());
          double                    cube_scale  = tree_->getResolution() * 1.0;
          Eigen::Vector3d           size        = Eigen::Vector3d(1, 1, 1) * cube_scale;
          Eigen::Quaterniond        orientation = Eigen::Quaterniond::Identity();
          mrs_lib::geometry::Cuboid c(center, size, orientation);
          bv_frontiers_->addCuboid( c, 1.0, 0.0, 1.0, 1.0, true);   
        }
      }
      
      for (auto &cell : fis->cells_)
      {
        Eigen::Vector3d           center(cell.x(), cell.y(), cell.z());
        double                    cube_scale  = tree_->getResolution() * 0.5;
        Eigen::Vector3d           size        = Eigen::Vector3d(1, 1, 1) * cube_scale;
        Eigen::Quaterniond        orientation = Eigen::Quaterniond::Identity();
        mrs_lib::geometry::Cuboid c(center, size, orientation);
        bv_frontiers_->addCuboid( c, color.r,color.g,color.b, 0.1, true);   
      }
    }
    
    ROS_INFO("[MRsExplorer]: final frontier count: %i/%i/%i/%i (total/rem/add/0v)", fis_c_.size(), removed_frontiers_.size(), fis_c_.size()-tmp, viewable_frontier_cnt_);
}

}