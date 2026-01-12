
#include "tsp_solver.hpp"

namespace mrs_octomap_planner
{

  TSPsolver::TSPsolver()
{
  cost_matrix_ = Eigen::MatrixXd(32,32);
  viewpoint_position_ = {octomap::point3d(0.0,0.0,0.0)};
  isAccesible_ = {true};
}



TSPsolver::TSPsolver(int max_duration, distance_funcion_t distanceFunciton)
{
  duration_ = ros::Duration(0, max_duration);
  cost_matrix_ = Eigen::MatrixXd(32,32);
  viewpoint_position_ = {octomap::point3d(0.0,0.0,0.0)};
  isAccesible_ = {true};
  distanceFunciton_ = distanceFunciton;
}

TSPsolver::~TSPsolver()
{
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
    solution.push_back((int)std::floor(getRand()*n*0.999999));
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


double TSPsolver::computeCost(const Eigen::MatrixXd &cost_matrix, const std::vector<int> &solution)
{
  double cost = 0.0;
  int size = solution.size();
  double dir_weight = 3.0;
  double height_weight = 2.0;
  int zero_position = std::distance(solution.begin(),std::find(solution.begin(), solution.end(), 0));
  for (int i=0; i<size; i++)
  {
    int idx0 = i;
    int idx1 = (i+1)%size;
    cost += cost_matrix(solution[idx0], solution[idx1]);
    if (solution[idx0] == 0)
    {
      cost += dir_weight*std::acos(start_velocity_.normalized().dot((viewpoint_position_[idx1] - viewpoint_position_[idx0]).normalized()));
      cost += height_weight*std::abs(viewpoint_position_[idx1].z() - viewpoint_position_[idx0].z());
    }
    else if (solution[idx1] != 0)
    {
      int prev_idx = (i-1)%size;
      int denominator = 5; // (i-zero_position) > 0 ? (i-zero_position) : (i-zero_position+size);
      cost += (dir_weight/denominator)*std::acos((viewpoint_position_[idx0] - viewpoint_position_[prev_idx]).normalized().dot((viewpoint_position_[idx1] - viewpoint_position_[idx0]).normalized()));
      cost += (height_weight/denominator)*std::abs(viewpoint_position_[idx1].z() - viewpoint_position_[idx0].z());
    }
    
  }


  return cost;
}

std::vector<octomap::point3d> TSPsolver::solve(octomath::Vector3 velocity)
{
  // ROS_ERROR("start TSP solve");
  start_velocity_ = velocity;
  int n = viewpoint_position_.size();
  auto permutation = solve(cost_matrix_.block(0,0,n,n), true);
  std::vector<octomap::point3d> solution(0);
  for (auto i : permutation){
    solution.push_back(viewpoint_position_[i]);
  }
  // ROS_ERROR("end TSP solve");
  return solution;
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
    idx0 = getRand(0,      solution_size-1);
    idx1 = getRand(idx0+1, solution_size);
    idx2 = getRand(idx1+1, solution_size+1);
    
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


double TSPsolver::computeDistance(octomap::point3d a, octomap::point3d b)
{
  path_info_t path_info = distanceFunciton_(a,b);
  double weighed_distance;
  if (path_info.distance == INVALID_DISTANCE)
  {
    weighed_distance = BIG_DISTANCE;
  }
  else
  {
    weighed_distance = path_info.distance + path_info.velocity_delta + 2*path_info.height;
  }
  return weighed_distance;
}


void TSPsolver::removeFrontiers(std::vector<octomap::point3d> removed_positions)
{
  for (auto &position : removed_positions)
  {
    int size = viewpoint_position_.size();
    int idx = std::distance(viewpoint_position_.begin(), std::find(viewpoint_position_.begin(), viewpoint_position_.end(), position));
    if (idx==0){
      continue;
    }
    if (idx==std::distance(viewpoint_position_.begin(), viewpoint_position_.end())){
      continue;
    }
    int n = cost_matrix_.cols();
    if (idx < n){
      cost_matrix_.block(idx, 0, n-1-idx, n) = cost_matrix_.block(idx+1, 0, n-1-idx, n);
      cost_matrix_.block(0, idx, n, n-1-idx) = cost_matrix_.block(0, idx+1, n, n-1-idx);
      
    }
    viewpoint_position_.erase(viewpoint_position_.begin() + idx);
  }
}
  
void TSPsolver::addFrontiers(std::vector<octomap::point3d> added_positions)
{
  int dist_comp_cnt = 0;
  int size = viewpoint_position_.size();
  if (cost_matrix_.cols() < size+added_positions.size())
  {
    cost_matrix_.conservativeResize((size+added_positions.size())*2, (size+added_positions.size())*2);
  }
  for (auto &viewpoint : added_positions)
  {
    size = viewpoint_position_.size();
    for (int i=0; i<size; i++)
    {
      dist_comp_cnt++;
      double dist = computeDistance(viewpoint , viewpoint_position_[i]);
      cost_matrix_(i, size) = dist;
      cost_matrix_(size, i) = dist;
    }
    cost_matrix_(size, size) = BIG_DISTANCE;
    viewpoint_position_.push_back(viewpoint );
  }
}

void TSPsolver::setStart(octomap::point3d position)
{
  start_position_ = position;
  isAccesible_ = std::vector<bool>(viewpoint_position_.size(), true);
  for (int i=1; i<viewpoint_position_.size(); i++)
  {
    double dist = computeDistance(start_position_, viewpoint_position_[i]);
    cost_matrix_(i, 0) = BIG_DISTANCE/100;
    cost_matrix_(0, i) = dist;
    if (dist == BIG_DISTANCE){
      isAccesible_[i] = false;
    }
  }
  cost_matrix_(0,0) = BIG_DISTANCE;
  viewpoint_position_[0] = start_position_;
}



}