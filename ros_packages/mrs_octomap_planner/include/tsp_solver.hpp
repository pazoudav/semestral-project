#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <algorithm>
#include "utils.hpp"

namespace mrs_octomap_planner
{

class TSPsolver
{
private:
    /* data */
    ros::Time start_;
    ros::Duration duration_;
    Eigen::MatrixXd cost_matrix_;
    std::vector<int> prev_solution_;

    std::vector<int> generateRadnSolution(int n);
    std::vector<int> generateGreedySolution(int n, const Eigen::MatrixXd &cost_matrix, bool zero_start=false);
    double computeCost(const Eigen::MatrixXd &cost_matrix, const std::vector<int> &solution);

public:
    TSPsolver();
    TSPsolver(int max_duration);
    ~TSPsolver();
    
    std::vector<int> solve(const Eigen::MatrixXd &cost_matrix, bool reuse_solution);
};




}

#endif