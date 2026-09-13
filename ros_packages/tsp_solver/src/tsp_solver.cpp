
#include "tsp_solver/tsp_solver.hpp"
#include <set>

namespace tsp_solver
{
  unsigned long START_ID = 0;

  // Initializes the fallback-solver time budget (nanoseconds) and default state: a single dummy start node, plus the client used to query real (roadmap) distances.
  TSPsolver::TSPsolver(int max_duration, ros::NodeHandle &nh, double lkh_time_limit, int lkh_runs)
{
  duration_ = ros::Duration(0, max_duration);
  GlobalTimeLimit_ = lkh_time_limit;
  GlobalRuns_ = lkh_runs;
  cost_matrix_ = Eigen::MatrixXd(32,32);
  viewpoint_positions_ = {octomap::point3d(0.0,0.0,0.0)};
  isAccesible_ = {true};
  sc_find_simplified_path_ = mrs_lib::ServiceClientHandler<path_planning::FindSimplifiedPath>(nh, "find_simplified_path_out");
  tree_ = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
}

TSPsolver::~TSPsolver()
{
}


// Builds the cost matrix for the current graph/start heading, solves it with LKH, and maps the resulting index tour back to viewpoint positions.
std::vector<octomap::point3d> TSPsolver::solve(octomath::Vector3 velocity)
{
  ROS_ERROR("start TSP solve");
  start_velocity_ = velocity;

  constructDistanceMatrix();
  int n = viewpoint_positions_.size();
  auto permutation = LKHSolve(); //solve(cost_matrix_, true);

  std::vector<octomap::point3d> solution(0);
  for (auto i : permutation){
    solution.push_back(viewpoint_positions_[i]);
  }
  ROS_INFO("dist map size %d, solution size %d, permutation size %d", dist_map_.size(), n, permutation.size());
  ROS_ERROR("end TSP solve");
  return solution;
}



// Single find_simplified_path_in call from `start` to every point in `goals`, so the (potentially expensive) roadmap Dijkstra search is shared across all of them
// instead of run once per pair. Path length is the sum of the returned raycast-simplified path's segment lengths; a goal path_planning couldn't reach, or a
// failed/empty service call, is reported as BIG_DISTANCE (matching the previous Euclidean code's unreachable-path sentinel).
std::vector<double> TSPsolver::findPathDistances(const octomap::point3d &start, const std::vector<octomap::point3d> &goals)
{
  std::vector<double> distances(goals.size(), octomap_planner_utils::BIG_DISTANCE);
  if (goals.empty())
    return distances;

  // ros::WallTime t_checkpoint = ros::WallTime::now();
  // ros::WallTime t_total      = ros::WallTime::now();

  path_planning::FindSimplifiedPath srv;
  srv.request.start.x = start.x();
  srv.request.start.y = start.y();
  srv.request.start.z = start.z();
  srv.request.use_raycast = true;
  srv.request.goals.reserve(goals.size());
  for (auto &g : goals)
  {
    geometry_msgs::Point p;
    p.x = g.x();
    p.y = g.y();
    p.z = g.z();
    srv.request.goals.push_back(p);
  }
  // ROS_INFO("[TSPsolver]: findPathDistances build_req    %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  // t_checkpoint = ros::WallTime::now();

  if (!sc_find_simplified_path_.call(srv))
  {
    ROS_ERROR("[TSPsolver]: find_simplified_path_out call failed for %lu goals", goals.size());
    // ROS_INFO("[TSPsolver]: findPathDistances svc_call     %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
    // ROS_INFO("[TSPsolver]: findPathDistances total        %4.3fms,", 1000 * (ros::WallTime::now() - t_total).toSec());
    // ROS_INFO("[TSPsolver]: ---------------------------");
    return distances;
  }
  // ROS_INFO("[TSPsolver]: findPathDistances svc_call     %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  // t_checkpoint = ros::WallTime::now();

  for (size_t i = 0; i < goals.size() && i < srv.response.success.size(); i++)
  {
    if (!srv.response.success[i])
      continue;

    const auto &points = srv.response.paths[i].points;
    double length = 0.0;
    for (size_t k = 0; k + 1 < points.size(); k++)
    {
      double dx = points[k + 1].x - points[k].x;
      double dy = points[k + 1].y - points[k].y;
      double dz = points[k + 1].z - points[k].z;
      length += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    distances[i] = length;
  }
  // ROS_INFO("[TSPsolver]: findPathDistances parse_resp   %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());

  // ROS_INFO("[TSPsolver]: findPathDistances total        %4.3fms,", 1000 * (ros::WallTime::now() - t_total).toSec());
  // ROS_INFO("[TSPsolver]: ---------------------------");
  return distances;
}


// Keeps dist_map_ in sync with the latest frontier set: removes nodes/distances for frontiers no longer present, then adds a node for each new frontier's
// first viewpoint, with its pairwise distances to all existing nodes computed in a single batched find_simplified_path_in call per new frontier.
void TSPsolver::syncFrontiers(const frontier_detection::FrontierArray::ConstPtr& msg)
{
  // ROS_ERROR("SYNC FRONTIERS TSP");

  ros::WallTime t_checkpoint = ros::WallTime::now();
  ros::WallTime t_total      = ros::WallTime::now();

  std::set<unsigned long> incoming_ids;
  for (auto &f : msg->frontiers)
  {
    incoming_ids.insert(f.id);
  }
  ROS_INFO("[TSPsolver]: syncFrontiers incoming_ids  %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  t_checkpoint = ros::WallTime::now();

  std::vector<uint32_t> removed_ids;
  for (auto it = dist_map_.begin(); it != dist_map_.end(); )
  {
    if (it->first != START_ID && incoming_ids.count(it->first) == 0)
    {
      removed_ids.push_back(it->first);
      it = dist_map_.erase(it);
    }
    else
    {
      ++it;
    }
  }

  for (auto &x : dist_map_)
  {
    for (auto &id : removed_ids)
    {
      auto it = x.second.distances.find(id);
      if (it != x.second.distances.end())
        x.second.distances.erase(it);
    }
  }
  
  ROS_INFO("[TSPsolver]: syncFrontiers prune_removed %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  t_checkpoint = ros::WallTime::now();
  ROS_INFO("[TSPsolver]: syncFrontiers removed %lu frontiers", removed_ids.size());

  int add_cnt = 0;

  for (auto &f : msg->frontiers)
  {
    // not empty and not already in dist_map_
    if (!f.viewpoints.empty() && dist_map_.count(f.id) == 0)
    {
      octomap::point3d p1(f.viewpoints[0].position.x, f.viewpoints[0].position.y, f.viewpoints[0].position.z);
      planner_t new_node = {.id=f.id, .position=p1, .isAccesible=true, .distances=std::map<unsigned long, float>()};

      std::vector<unsigned long> ids;
      std::vector<octomap::point3d> positions;
      ids.reserve(dist_map_.size());
      positions.reserve(dist_map_.size());
      for (auto &x : dist_map_)
      {
        ids.push_back(x.first);
        positions.push_back(x.second.position);
      }

      std::vector<double> distances = findPathDistances(p1, positions);

      for (size_t i = 0; i < ids.size(); i++)
      {
        double dist = distances[i];
        new_node.distances.insert({ids[i], dist});
        dist_map_[ids[i]].distances.insert({f.id, dist});
      }
      dist_map_.insert({f.id, new_node});
      add_cnt += 1;
    }
  }
  
  ROS_INFO("[TSPsolver]: syncFrontiers add_new       %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  ROS_INFO("[TSPsolver]: syncFrontiers added %d frontiers", add_cnt);

  ROS_INFO("[TSPsolver]: syncFrontiers total         %4.3fms,", 1000 * (ros::WallTime::now() - t_total).toSec());
  ROS_INFO("[TSPsolver]: ---------------------------");
}

// Replaces the START_ID node with the given position and recomputes its distance to every other node (one batched find_simplified_path_in call), marking neighbors unreachable (BIG_DISTANCE) as inaccessible.
void TSPsolver::setStart(octomap::point3d position)
{
  // ROS_ERROR("SET START TSP");
  ros::WallTime t_checkpoint = ros::WallTime::now();
  ros::WallTime t_total      = ros::WallTime::now();

  auto it = dist_map_.find(START_ID);
  if( it != dist_map_.end() )
      dist_map_.erase( it );

  planner_t new_node = {.id=START_ID, .position=position, .isAccesible=true, .distances=std::map<unsigned long, float>()};

  std::vector<unsigned long> ids;
  std::vector<octomap::point3d> positions;
  ids.reserve(dist_map_.size());
  positions.reserve(dist_map_.size());
  for (auto &x : dist_map_)
  {
    ids.push_back(x.first);
    positions.push_back(x.second.position);
  }
  ROS_INFO("[TSPsolver]: setStart gather_ids   %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  t_checkpoint = ros::WallTime::now();

  std::vector<double> distances = findPathDistances(position, positions);
  ROS_INFO("[TSPsolver]: setStart find_dists   %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  t_checkpoint = ros::WallTime::now();

  for (size_t i = 0; i < ids.size(); i++)
  {
    double dist = distances[i];
    new_node.distances.insert({ids[i], dist});
    auto &x = dist_map_[ids[i]];
    x.distances.insert({START_ID, dist});
    if (dist == octomap_planner_utils::BIG_DISTANCE)
      x.isAccesible = false;
  }
  dist_map_.insert({START_ID, new_node});
  ROS_INFO("[TSPsolver]: setStart update_map    %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());

  ROS_INFO("[TSPsolver]: setStart total         %4.3fms,", 1000 * (ros::WallTime::now() - t_total).toSec());
  ROS_INFO("[TSPsolver]: ---------------------------");
  // ROS_ERROR("SET START TSP");

}

// Flattens dist_map_ (excluding START_ID's own row bookkeeping) into cost_matrix_/viewpoint_positions_/isAccesible_; adds a heading-change penalty and, if a KD-tree of guiding viewpoints is set, a nearest-guiding-viewpoint distance term to the start node's row.
void TSPsolver::constructDistanceMatrix()
{
  int size = dist_map_.size();
  cost_matrix_.resize(size, size);
  viewpoint_positions_ = std::vector<octomap::point3d>(0);
  isAccesible_ = std::vector<bool>(0);
  std::vector<int> idx(1);
  std::vector<float> dist(1);
  float vp_weight = 0.7;
  float dir_weight = 4.0;
  octomap::point3d start_position = dist_map_[START_ID].position;

  // Fixed node->index mapping (dist_map_ order, START_ID sorts first) shared by both rows and columns,
  // so cost_matrix_(i,j) always refers to the same pair of nodes regardless of which row is being filled.
  std::vector<unsigned long> ids;
  ids.reserve(size);
  for (auto &entry : dist_map_)
    ids.push_back(entry.first);

  for (int i=0; i<size; i++)
  {
    auto &value = dist_map_[ids[i]];
    octomap::point3d position = value.position;
    viewpoint_positions_.push_back(position);
    isAccesible_.push_back(value.isAccesible);

    for (int j=0; j<size; j++)
    {
      cost_matrix_(i,j) = (i == j) ? 0.0 : value.distances.at(ids[j]);
    }

    // if (pcset_)
    // {
    //   pcl::PointXYZ point = pcl::PointXYZ(position.x(), position.y(), position.z());
    //   if (tree_->nearestKSearch(point, 1, idx, dist) > 0)
    //     cost_matrix_(0, i) += vp_weight*dist[0];
    // }
    // cost_matrix_(0, i) += dir_weight*std::acos(start_velocity_.normalized().dot((position - start_position).normalized()));
    cost_matrix_(i, 0) = 0.0; // BIG_DISTANCE/100;
  }
  // costmatrixViewpointAdjustment();
}

// Writes the LKH .par file: problem/output file paths, GAIN23 heuristic, and the configured time limit/run count.
void TSPsolver::GlobalParWrite()
{
  std::ofstream par_file(GlobalPar_);
  par_file << "PROBLEM_FILE = " << GlobalProF_ << "\n"; // TIME_LIMIT
  par_file << "GAIN23 = YES\n";
  par_file << "TIME_LIMIT = " << std::to_string(GlobalTimeLimit_) << "\n";
  par_file << "OUTPUT_TOUR_FILE =" << GlobalResult_ << "\n";
  par_file << "RUNS = " << std::to_string(GlobalRuns_) << "\n";
  par_file.close();
}

// Writes costMat as an explicit full-matrix ATSP problem file for LKH; costs are scaled by precision_ and truncated to integers, as LKH requires integer edge weights.
void TSPsolver::GlobalProblemWrite(Eigen::MatrixXd& costMat)
{
  const int dimension = costMat.rows();
  std::ofstream prob_file(GlobalProF_);
  std::string prob_spec = "NAME : global\nTYPE : ATSP\nDIMENSION : " + std::to_string(dimension) +
    "\nEDGE_WEIGHT_TYPE : "
    "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;

  for (int i=0; i<dimension; ++i)
  {
    for (int j=0; j<dimension; ++j)
    {
      int int_cost = costMat(i,j)*precision_;
      prob_file << std::to_string(int_cost) << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();
}

// Parses LKH's output tour file (TOUR_SECTION, 1-based node ids terminated by -1) into a 0-based index sequence.
std::vector<int> TSPsolver::GlobalResultsRead()
{
  std::vector<int> results;

  std::ifstream res_file(GlobalResult_);
  std::string res;
  while (getline(res_file, res))
    if (res.compare("TOUR_SECTION") == 0) break;

  while (getline(res_file, res))
  {
    int id = std::stoi(res);
    // if (id == 1)
    //   continue;
    if (id == -1) break;
    results.push_back(id - 1);
  }
  res_file.close();

  return results;
}

// Full LKH invocation: writes the .par and cost-matrix problem files, shells out to the bundled LKH binary via a blocking system() call, then reads back the solved tour.
std::vector<int> TSPsolver::LKHSolve()
{
  /* write par file */
  GlobalParWrite();
  /* write problem file */
  GlobalProblemWrite(cost_matrix_);
  /* ATSP solving */
  std::string command_ = "cd " + GlobalDir_ + " && ./LKH " + GlobalPar_;
  const char* charPtr = command_.c_str();
  int system_back_ = std::system(charPtr);
  /* read solution results */
  std::vector<int> result = GlobalResultsRead();
  return result;
}

// Adds each node's distance to its nearest KD-tree guiding viewpoint into the start row of the cost matrix; no-op if setKDtreeInput() was never called. Dead code: the only call site is commented out in constructDistanceMatrix().
void TSPsolver::costmatrixViewpointAdjustment()
{
  ROS_WARN("VP adjustment");
  if (!pcset_){
    return;
  }
  std::vector<int> idx(1);
  std::vector<float> dist(1);
  int i = 0;
  float weight = 1.0;

  for (auto &el : dist_map_)
  {
    auto p =  el.second.position;
    pcl::PointXYZ point = pcl::PointXYZ(p.x(), p.y(), p.z());
    if (tree_->nearestKSearch(point, 1, idx, dist) > 0)
    {
      cost_matrix_(0, i) += weight*dist[0];
    }
    i++;
  }
}

// Loads the given point cloud (skeleton-derived guiding viewpoints) into the KD-tree used to bias the cost matrix; ignored if empty.
void TSPsolver::setKDtreeInput(pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints)
{
  // ROS_ERROR("pc call");
  if (guiding_viewpoints->size() == 0){
    return;
  }
  ROS_ERROR("pc size %d", guiding_viewpoints->size());
  tree_->setInputCloud(guiding_viewpoints);
  pcset_ = true;
  // ROS_ERROR("pc set");
}

}
