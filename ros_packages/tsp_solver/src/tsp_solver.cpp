
#include "tsp_solver/tsp_solver.hpp"
#include <set>

namespace tsp_solver
{
  unsigned long START_ID = 0;

  TSPsolver::TSPsolver(int max_duration)
{
  duration_ = ros::Duration(0, max_duration);
  cost_matrix_ = Eigen::MatrixXd(32,32);
  viewpoint_positions_ = {octomap::point3d(0.0,0.0,0.0)};
  isAccesible_ = {true};
  distanceFunciton_ = [](octomap::point3d a, octomap::point3d b) {
    octomap_planner_utils::path_info_t info;
    info.distance = a.distance(b);
    return info;
  };
  tree_ = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
}

TSPsolver::~TSPsolver()
{
}


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



double TSPsolver::computeDistance(octomap::point3d a, octomap::point3d b)
{
  octomap_planner_utils::path_info_t path_info = distanceFunciton_(a,b);
  double weighed_distance;
  if (path_info.distance == octomap_planner_utils::INVALID_DISTANCE)
  {
    weighed_distance = octomap_planner_utils::BIG_DISTANCE;
  }
  else
  {
    weighed_distance = path_info.distance; // + path_info.velocity_delta + 2*path_info.height;
  }
  return weighed_distance;
}


void TSPsolver::syncFrontiers(const frontier_detection::FrontierArray::ConstPtr& msg)
{
  ROS_ERROR("SYNC FRONTIERS TSP");

  std::set<unsigned long> incoming_ids;
  for (auto &f : msg->frontiers)
  {
    incoming_ids.insert(f.id);
  }

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

  ROS_ERROR("removed %d frontiers", removed_ids.size());

  ROS_WARN("all frontiers %d", msg->frontiers.size());
  ROS_WARN("dist map size %d", dist_map_.size());
  for (auto &f : msg->frontiers)
  {
    // not empty and not already in dist_map_
    if (!f.viewpoints.empty() && dist_map_.count(f.id) == 0)
    {
      octomap::point3d p1(f.viewpoints[0].position.x, f.viewpoints[0].position.y, f.viewpoints[0].position.z);
      planner_t new_node = {.id=f.id, .position=p1, .isAccesible=true, .distances=std::map<unsigned long, float>()};

      for (auto &x: dist_map_)
      {
        double dist = computeDistance(p1, x.second.position);
        new_node.distances.insert({x.first, dist});
        x.second.distances.insert({f.id, dist});
      }
      dist_map_.insert({f.id, new_node});
    }
  }
  ROS_WARN("dist map size %d", dist_map_.size());
  ROS_ERROR("SYNC FRONTIERS TSP");

}

void TSPsolver::setStart(octomap::point3d position)
{
  ROS_ERROR("SET START TSP");
  auto it = dist_map_.find(START_ID);
  if( it != dist_map_.end() )
      dist_map_.erase( it );

  planner_t new_node = {.id=START_ID, .position=position, .isAccesible=true, .distances=std::map<unsigned long, float>()};

  for (auto &x: dist_map_)
  {
    double dist = computeDistance(position, x.second.position);
    new_node.distances.insert({x.first, dist});
    x.second.distances.insert({START_ID, dist});
    if (dist == octomap_planner_utils::BIG_DISTANCE)
      x.second.isAccesible = false;
  }
  dist_map_.insert({START_ID, new_node});
  ROS_ERROR("SET START TSP");

}

void TSPsolver::constructDistanceMatrix()
{
  int size = dist_map_.size() - 1;
  cost_matrix_.resize(size, size);
  auto start_dist = dist_map_.begin();
  viewpoint_positions_ = std::vector<octomap::point3d>(0);
  isAccesible_ = std::vector<bool>(0);
  std::vector<int> idx(1);
  std::vector<float> dist(1);
  float vp_weight = 0.7;
  float dir_weight = 4.0;
  octomap::point3d start_position = dist_map_[START_ID].position;

  for (int i=0; i<size; i++)
  {
    auto value = std::next(start_dist, i)->second;
    octomap::point3d position = value.position;
    viewpoint_positions_.push_back(position);
    isAccesible_.push_back(value.isAccesible);
    auto start_neighbors = value.distances.begin();


    for (int j=0; j<size; j++)
    {
      auto it_j = std::next(start_neighbors, j);
      cost_matrix_(i,j) = it_j->second;
    }

    if (pcset_)
    {
      pcl::PointXYZ point = pcl::PointXYZ(position.x(), position.y(), position.z());
      if (tree_->nearestKSearch(point, 1, idx, dist) > 0)
        cost_matrix_(0, i) += vp_weight*dist[0];
    }
    cost_matrix_(0, i) += dir_weight*std::acos(start_velocity_.normalized().dot((position - start_position).normalized()));
    cost_matrix_(i, i) = 0.0;
    cost_matrix_(i, 0) = 0.0; // BIG_DISTANCE/100;
  }
  // costmatrixViewpointAdjustment();
}

void TSPsolver::GlobalParWrite()
{
  std::ofstream par_file(GlobalPar_);
  par_file << "PROBLEM_FILE = " << GlobalProF_ << "\n"; // TIME_LIMIT
  par_file << "GAIN23 = YES\n";
  par_file << "TIME_LIMIT = 0.5\n";
  par_file << "OUTPUT_TOUR_FILE =" << GlobalResult_ << "\n";
  par_file << "RUNS = " << std::to_string(GlobalRuns_) << "\n";
  par_file.close();
}

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

std::vector<int> TSPsolver::LKHSolve()
{
  /* write par file */
  GlobalParWrite();
  /* construct ATSP cost matrix */
  // Eigen::MatrixXd GloablCostMat;
  // GloablCostMat = GlobalCostMat(solver_start_, centroids);
  // constructDistanceMatrix();
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


std::vector<int> TSPsolver::solve(const Eigen::MatrixXd &cost_matrix, bool reuse_solution)
{
  // int size = cost_matrix.cols();
  std::vector<int> best_solution;
  double best_cost = std::numeric_limits<double>::max();

  start_ = ros::Time::now();

  reuse_solution = true;
  std::vector<int> solution = generateGreedySolution(cost_matrix.cols(), cost_matrix, true);

  double cost = computeCost(cost_matrix, solution);
  int solution_size = solution.size();
  if (solution_size < 3){
    return solution;
  }
  int iter_cnt = 0; int good = 0;
  int idx0 = 0;
  int idx1 = 0;
  int idx2 = 0;
  double temp_cost = 0.0;
  ROS_WARN("3-opt start %i", solution_size);
  // try to inmprove cost by randomly swaping two nodes
  while ((ros::Time::now()-start_).nsec < duration_.nsec)
  {
    iter_cnt++;
    idx0 = octomap_planner_utils::getRand(0,      solution_size-1);
    idx1 = octomap_planner_utils::getRand(idx0+1, solution_size);
    idx2 = octomap_planner_utils::getRand(idx1+1, solution_size+1);

    std::reverse(solution.begin()+idx0, solution.begin()+idx1);
    temp_cost = computeCost(cost_matrix, solution);
    if (temp_cost < cost) {
      good++;
      cost = temp_cost;
      continue;
    } else {
      std::reverse(solution.begin()+idx0, solution.begin()+idx1);
    }

    std::reverse(solution.begin()+idx1, solution.begin()+idx2);
    temp_cost = computeCost(cost_matrix, solution);
    if (temp_cost < cost) {
      good++;
      cost = temp_cost;
      continue;
    } else {
      std::reverse(solution.begin()+idx1, solution.begin()+idx2);
    }

    std::reverse(solution.begin()+idx0, solution.begin()+idx1);
    std::reverse(solution.begin()+idx1, solution.begin()+idx2);
    temp_cost = computeCost(cost_matrix, solution);
    if (temp_cost < cost) {
      good++;
      cost = temp_cost;
      continue;
    } else {
      std::reverse(solution.begin()+idx0, solution.begin()+idx1);
      std::reverse(solution.begin()+idx1, solution.begin()+idx2);
    }

    // int idx3 = idx0 + idx2-idx1;
    std::vector<int> temp_solution(0);
    for (int i=0; i < idx0; i++){
      temp_solution.push_back(solution[i]);
    }
    for (int i=idx1; i < idx2; i++){
      temp_solution.push_back(solution[i]);
    }
    for (int i=idx0; i < idx1; i++){
      temp_solution.push_back(solution[i]);
    }
    for (int i=idx2; i < solution.size(); i++){
      temp_solution.push_back(solution[i]);
    }

    temp_cost = computeCost(cost_matrix, temp_solution);
    if (temp_cost < cost) {
      good++;
      cost = temp_cost;
      continue;
    }

    std::reverse(temp_solution.begin()+idx0, temp_solution.begin()+idx1);
    temp_cost = computeCost(cost_matrix, temp_solution);
    if (temp_cost < cost) {
      good++;
      cost = temp_cost;
      continue;
    } else {
      std::reverse(temp_solution.begin()+idx0, temp_solution.begin()+idx1);
    }

    std::reverse(temp_solution.begin()+idx1, temp_solution.begin()+idx2);
    temp_cost = computeCost(cost_matrix, temp_solution);
    if (temp_cost < cost) {
      good++;
      cost = temp_cost;
      continue;
    } else {
      std::reverse(temp_solution.begin()+idx1, temp_solution.begin()+idx2);
    }

    std::reverse(temp_solution.begin()+idx0, temp_solution.begin()+idx1);
    std::reverse(temp_solution.begin()+idx1, temp_solution.begin()+idx2);
    temp_cost = computeCost(cost_matrix, temp_solution);
    if (temp_cost < cost) {
      good++;
      cost = temp_cost;
      continue;
    } else {
      std::reverse(temp_solution.begin()+idx0, temp_solution.begin()+idx1);
      std::reverse(temp_solution.begin()+idx1, temp_solution.begin()+idx2);
    }
  }
  ROS_WARN("iter_cnt %i/%i", good, iter_cnt);
  if (cost < best_cost)
  {
    best_cost = cost;
    best_solution = solution;
  }

  // process found global path to begin at node 0 (current position)
  auto zero_itr = std::find(best_solution.begin(), best_solution.end(), 0);
  std::rotate(best_solution.begin(), best_solution.begin() + std::distance(best_solution.begin(), zero_itr), best_solution.end());
  prev_solution_ = best_solution;

  return best_solution;
}

double TSPsolver::computeCost(const Eigen::MatrixXd &cost_matrix, const std::vector<int> &solution)
{
  double cost = 0.0;
  int size = solution.size();
  double dir_weight = 4.0;
  double height_weight = 3.0;
  int zero_position = std::distance(solution.begin(),std::find(solution.begin(), solution.end(), 0));
  for (int i=0; i<size; i++)
  {
    int idx0 = i;
    int idx1 = (i+1)%size;
    cost += cost_matrix(solution[idx0], solution[idx1]);
    if (solution[idx0] == 0)
    {
      cost += dir_weight*std::acos(start_velocity_.normalized().dot((viewpoint_positions_[idx1] - viewpoint_positions_[idx0]).normalized()));
      cost += height_weight*std::abs(viewpoint_positions_[idx1].z() - viewpoint_positions_[idx0].z());
    }
    else if (solution[idx1] != 0)
    {
      int prev_idx = (i-1)%size;
      int denominator = 10; // (i-zero_position) > 0 ? (i-zero_position) : (i-zero_position+size);
      cost += (dir_weight/denominator)*std::acos((viewpoint_positions_[idx0] - viewpoint_positions_[prev_idx]).normalized().dot((viewpoint_positions_[idx1] - viewpoint_positions_[idx0]).normalized()));
      cost += (height_weight/denominator)*std::abs(viewpoint_positions_[idx1].z() - viewpoint_positions_[idx0].z());
    }

  }
  return cost;
}

std::vector<int> TSPsolver::generateRadnSolution(int n)
{
  std::vector<double> temp_solution(0);
  std::vector<int> solution(0);
  for (int i = 0; i < n; i++)
  {
    temp_solution.push_back(rand());
    solution.push_back(i);
  }

  std::sort(solution.begin(), solution.end(), [&temp_solution](int a, int b)
        {
          return temp_solution[a] < temp_solution[b];
        });

  return solution;
}


std::vector<int> TSPsolver::generateGreedySolution(int n, const Eigen::MatrixXd &cost_matrix, bool zero_start)
{
  std::vector<int> solution(0);

  if (zero_start)
  {
    solution.push_back(0);
  }
  else
  {
    solution.push_back((int)std::floor(octomap_planner_utils::getRand()*n*0.999999));
  }

  for (int i = 1; i < n; i++)
  {
    if (!isAccesible_[i]){
      continue;
    }
    auto row       = cost_matrix.row(solution.back());
    double min_val = std::numeric_limits<double>::max();
    int best_neighbor;
    for (int j=0; j<n; j++)
    {
      if (std::find(solution.begin(), solution.end(), j) == solution.end())
      {
        if (row(j) < min_val)
        {
          min_val = row(j);
          best_neighbor = j;
        }
      }
    }
    solution.push_back(best_neighbor); // std::distance(row.begin, min_itr));
  }
  return solution;
}

}
