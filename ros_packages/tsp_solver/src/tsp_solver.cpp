
#include "tsp_solver/tsp_solver.hpp"
#include <set>
#include <cstdio>

namespace tsp_solver
{
  unsigned long START_ID = 0;

  // Initializes the fallback-solver time budget (nanoseconds) and default state: a single dummy start node, plus the client used to query real (roadmap) distances.
  TSPsolver::TSPsolver(int max_duration, ros::NodeHandle &nh, double lkh_time_limit, int lkh_runs)
{
  duration_ = ros::Duration(0, max_duration);
  GlobalTimeLimit_ = lkh_time_limit;
  GlobalRuns_ = lkh_runs;
  ensureCapacity(32);
  sc_find_simplified_path_ = mrs_lib::ServiceClientHandler<path_planning::FindSimplifiedPath>(nh, "find_simplified_path_out");
  tree_ = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  // LKH .par contents are fixed for the process lifetime; write once here instead of on every solve().
  GlobalParWrite();
}

TSPsolver::~TSPsolver()
{
}


// Grows distances_ (conservativeResize preserves the existing top-left block) to at least `needed` x `needed`.
void TSPsolver::ensureCapacity(int needed)
{
  int cap = static_cast<int>(distances_.rows());
  if (needed <= cap)
    return;
  int new_cap = std::max(needed, cap > 0 ? cap * 2 : 32);
  distances_.conservativeResize(new_cap, new_cap);
}

int TSPsolver::addNode(unsigned long id, const octomap::point3d &position, bool isAccesible)
{
  int index = static_cast<int>(nodes_.size());
  ensureCapacity(index + 1);
  nodes_.push_back(planner_t{id, position, isAccesible});
  id_to_index_[id] = index;
  return index;
}

void TSPsolver::removeNode(unsigned long id)
{
  auto it = id_to_index_.find(id);
  if (it == id_to_index_.end())
    return;

  int index = it->second;
  int last  = static_cast<int>(nodes_.size()) - 1;

  if (index != last)
  {
    nodes_[index] = nodes_[last];
    id_to_index_[nodes_[index].id] = index;
    distances_.row(index).head(last) = distances_.row(last).head(last);
    distances_.col(index).head(last) = distances_.row(last).head(last).transpose();
  }

  nodes_.pop_back();
  id_to_index_.erase(id);
}

void TSPsolver::setDistance(int i, int j, float value)
{
  distances_(i, j) = value;
  distances_(j, i) = value;
}

// One-time setup: makes room for START_ID at index 0 by shifting every already-present node (and the
// matrix rows/cols they occupy) up by one. Only ever runs on the first setStart() call, since START_ID
// is never removed afterwards (syncFrontiers explicitly excludes it from pruning).
void TSPsolver::reserveStartSlot()
{
  int n = static_cast<int>(nodes_.size());
  ensureCapacity(n + 1);
  nodes_.insert(nodes_.begin(), planner_t{START_ID, octomap::point3d(0.0, 0.0, 0.0), true});
  for (auto &kv : id_to_index_)
    kv.second += 1;
  id_to_index_[START_ID] = 0;
  if (n > 0)
    distances_.block(1, 1, n, n) = distances_.block(0, 0, n, n).eval();
}


// Builds the cost matrix for the current graph/start heading, solves it with LKH, and maps the resulting index tour back to viewpoint positions.
std::vector<octomap::point3d> TSPsolver::solve(octomath::Vector3 velocity)
{
  std::scoped_lock solve_lock(mutex_solve_);

  ROS_ERROR("start TSP solve");
  start_velocity_ = velocity;

  {
    // Only the snapshot into cost_matrix_/viewpoint_positions_/isAccesible_ needs the dist-graph lock;
    // releasing it before LKHSolve() keeps syncFrontiers()/setStart() from stalling behind LKH's runtime.
    std::scoped_lock dist_lock(mutex_dist_graph_);
    constructDistanceMatrix();
  }

  int n = viewpoint_positions_.size();
  auto permutation = LKHSolve(); //solve(cost_matrix_, true);

  std::vector<octomap::point3d> solution;
  solution.reserve(permutation.size());
  for (auto i : permutation){
    // Guards against a stale/mismatched solution.txt (e.g. left over from a failed LKH run)
    // being parsed into indices that no longer fit the current viewpoint set.
    if (i < 0 || i >= n)
    {
      ROS_ERROR("[TSPsolver]: LKH tour index %d out of range for %d viewpoints, discarding solve result", i, n);
      solution.clear();
      break;
    }
    solution.push_back(viewpoint_positions_[i]);
  }
  ROS_INFO("dist map size %lu, solution size %d, permutation size %d", nodes_.size(), n, permutation.size());
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


// Keeps the distance graph in sync with the latest frontier set: removes nodes for frontiers no longer present, then adds a node for each new frontier's
// first viewpoint, with its pairwise distances to all existing nodes computed in a single batched find_simplified_path_in call per new frontier.
void TSPsolver::syncFrontiers(const frontier_detection::FrontierArray::ConstPtr& msg)
{
  // ROS_ERROR("SYNC FRONTIERS TSP");

  ros::WallTime t_checkpoint = ros::WallTime::now();
  ros::WallTime t_total      = ros::WallTime::now();

  std::scoped_lock lock(mutex_dist_graph_);

  std::set<unsigned long> incoming_ids;
  for (auto &f : msg->frontiers)
  {
    incoming_ids.insert(f.id);
  }
  ROS_INFO("[TSPsolver]: syncFrontiers incoming_ids  %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  t_checkpoint = ros::WallTime::now();

  std::vector<unsigned long> removed_ids;
  for (auto &node : nodes_)
  {
    if (node.id != START_ID && incoming_ids.count(node.id) == 0)
      removed_ids.push_back(node.id);
  }
  for (auto id : removed_ids)
    removeNode(id);

  ROS_INFO("[TSPsolver]: syncFrontiers prune_removed %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  t_checkpoint = ros::WallTime::now();
  ROS_INFO("[TSPsolver]: syncFrontiers removed %lu frontiers", removed_ids.size());

  int add_cnt = 0;

  for (auto &f : msg->frontiers)
  {
    // not empty and not already in the graph
    if (!f.viewpoints.empty() && id_to_index_.count(f.id) == 0)
    {
      octomap::point3d p1(f.viewpoints[0].position.x, f.viewpoints[0].position.y, f.viewpoints[0].position.z);

      std::vector<octomap::point3d> positions;
      positions.reserve(nodes_.size());
      for (auto &node : nodes_)
        positions.push_back(node.position);

      std::vector<double> distances = findPathDistances(p1, positions);

      int new_index = addNode(f.id, p1, true);
      for (size_t i = 0; i < distances.size(); i++)
        setDistance(new_index, static_cast<int>(i), static_cast<float>(distances[i]));

      add_cnt += 1;
    }
  }

  ROS_INFO("[TSPsolver]: syncFrontiers add_new       %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  ROS_INFO("[TSPsolver]: syncFrontiers added %d frontiers", add_cnt);

  ROS_INFO("[TSPsolver]: syncFrontiers total         %4.3fms,", 1000 * (ros::WallTime::now() - t_total).toSec());
  ROS_INFO("[TSPsolver]: ---------------------------");
}

// Replaces the START_ID node's position (always index 0 — see reserveStartSlot()) and recomputes its distance to every other node
// (one batched find_simplified_path_in call), marking neighbors unreachable (BIG_DISTANCE) as inaccessible.
void TSPsolver::setStart(octomap::point3d position)
{
  // ROS_ERROR("SET START TSP");
  ros::WallTime t_checkpoint = ros::WallTime::now();
  ros::WallTime t_total      = ros::WallTime::now();

  std::scoped_lock lock(mutex_dist_graph_);

  if (id_to_index_.count(START_ID) == 0)
    reserveStartSlot();

  std::vector<octomap::point3d> positions;
  positions.reserve(nodes_.size() - 1);
  for (size_t i = 1; i < nodes_.size(); i++)
    positions.push_back(nodes_[i].position);
  ROS_INFO("[TSPsolver]: setStart gather_ids   %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  t_checkpoint = ros::WallTime::now();

  std::vector<double> distances = findPathDistances(position, positions);
  ROS_INFO("[TSPsolver]: setStart find_dists   %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
  t_checkpoint = ros::WallTime::now();

  nodes_[0].position    = position;
  nodes_[0].isAccesible = true;
  for (size_t i = 0; i < distances.size(); i++)
  {
    int node_index = static_cast<int>(i) + 1;
    double dist = distances[i];
    setDistance(0, node_index, static_cast<float>(dist));
    if (dist == octomap_planner_utils::BIG_DISTANCE)
      nodes_[node_index].isAccesible = false;
  }
  ROS_INFO("[TSPsolver]: setStart update_map    %4.3fms,", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());

  ROS_INFO("[TSPsolver]: setStart total         %4.3fms,", 1000 * (ros::WallTime::now() - t_total).toSec());
  ROS_INFO("[TSPsolver]: ---------------------------");
  // ROS_ERROR("SET START TSP");

}

// Flattens the distance graph into cost_matrix_/viewpoint_positions_/isAccesible_ via a single block copy (no per-cell lookups).
void TSPsolver::constructDistanceMatrix()
{
  int size = static_cast<int>(nodes_.size());
  // Fails loudly (throws) if setStart() was never called; also documents the invariant relied on below.
  octomap::point3d start_position = nodes_[id_to_index_.at(START_ID)].position;
  (void)start_position; // only consumed by the commented-out heading-penalty/KD-tree code below

  cost_matrix_ = distances_.topLeftCorner(size, size).cast<double>();
  cost_matrix_.diagonal().setZero();

  viewpoint_positions_.clear();
  viewpoint_positions_.reserve(size);
  isAccesible_.clear();
  isAccesible_.reserve(size);

  float vp_weight = 0.7;
  float dir_weight = 4.0;
  (void)vp_weight;
  (void)dir_weight;

  for (int i=0; i<size; i++)
  {
    viewpoint_positions_.push_back(nodes_[i].position);
    isAccesible_.push_back(nodes_[i].isAccesible);

    // if (pcset_)
    // {
    //   pcl::PointXYZ point = pcl::PointXYZ(nodes_[i].position.x(), nodes_[i].position.y(), nodes_[i].position.z());
    //   if (tree_->nearestKSearch(point, 1, idx, dist) > 0)
    //     cost_matrix_(0, i) += vp_weight*dist[0];
    // }
    // cost_matrix_(0, i) += dir_weight*std::acos(start_velocity_.normalized().dot((nodes_[i].position - start_position).normalized()));

    // START_ID is always index 0 (see reserveStartSlot()), so this is the zero-cost "return to start"
    // column the ATSP formulation relies on to let LKH produce an open (rather than closed) tour.
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
  par_file << "TIME_LIMIT = " << GlobalTimeLimit_ << "\n";
  par_file << "OUTPUT_TOUR_FILE =" << GlobalResult_ << "\n";
  par_file << "RUNS = " << GlobalRuns_ << "\n";
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
      prob_file << int_cost << ' ';
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

// Full LKH invocation: writes the cost-matrix problem file (the .par file is written once at construction), shells out to the bundled LKH binary via a blocking system() call, then reads back the solved tour.
std::vector<int> TSPsolver::LKHSolve()
{
  /* write problem file */
  GlobalProblemWrite(cost_matrix_);
  /* drop any previous result so a failed run below can't be mistaken for a stale success */
  std::remove(GlobalResult_.c_str());
  /* ATSP solving */
  std::string command_ = "cd " + GlobalDir_ + " && ./LKH " + GlobalPar_;
  int system_back_ = std::system(command_.c_str());
  if (system_back_ != 0)
  {
    ROS_ERROR("[TSPsolver]: LKH invocation failed (exit code %d)", system_back_);
    return {};
  }
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
  float weight = 1.0;

  for (size_t i = 0; i < nodes_.size(); i++)
  {
    auto p = nodes_[i].position;
    pcl::PointXYZ point = pcl::PointXYZ(p.x(), p.y(), p.z());
    if (tree_->nearestKSearch(point, 1, idx, dist) > 0)
    {
      cost_matrix_(0, i) += weight*dist[0];
    }
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
