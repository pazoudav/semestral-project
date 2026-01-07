
#include "tsp_solver.hpp"

namespace mrs_octomap_planner
{

  TSPsolver::TSPsolver()
{
}



TSPsolver::TSPsolver(int max_duration)
{
  duration_ = ros::Duration(0, max_duration);
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
    auto row     = cost_matrix.row(solution.back());
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
  for (int i=0; i<size; i++)
  {
    cost += cost_matrix(solution[i], solution[(i+1)%size]);
  }
  return cost;
}

std::vector<int> TSPsolver::solve(const Eigen::MatrixXd &cost_matrix, bool reuse_solution)
{
  int size = cost_matrix.cols();
  std::vector<int> best_solution;
  double best_cost = std::numeric_limits<double>::max();

  start_ = ros::Time::now();

  reuse_solution =true;
  prev_solution_ = generateGreedySolution(size, cost_matrix, true);

  int cnt = 0;
  while (ros::Time::now() - start_ < duration_)
  { 
    std::vector<int> solution;
    if (reuse_solution && cnt  == 0)
    {
      solution = prev_solution_;
    }
    else
    {
      solution = generateGreedySolution(size,cost_matrix);
    }

    double cost = computeCost(cost_matrix, solution);

    for (int i=0; i<size*5000; i++)
    {
      int idx0 = rand() % size;
      int idx1 = rand() % size;
      std::swap(solution[idx0], solution[idx1]);
      double temp_cost = computeCost(cost_matrix, solution);
      if (temp_cost < cost)
      {
        cost = temp_cost;
      }
      else
      {
        std::swap(solution[idx0], solution[idx1]);
      }
    }
    
    if (cost < best_cost)
    {
      best_cost = cost;
      best_solution = solution;
    }

    cnt++;
  }
  prev_solution_ = best_solution;
  return best_solution;
}



}