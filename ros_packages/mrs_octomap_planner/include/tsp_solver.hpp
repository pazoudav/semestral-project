#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <vector>
#include <numeric>
#include <algorithm>
#include "utils.hpp"
#include "fis.hpp"

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
    // std::vector<unsigned long> viewpoint_id_;
    std::vector<octomap::point3d> viewpoint_position_;
    octomap::point3d start_position_;
    distance_funcion_t distanceFunciton_;
    octomath::Vector3 start_velocity_;
    std::vector<bool> isAccesible_;
    

    std::vector<int> generateRadnSolution(int n);
    std::vector<int> generateGreedySolution(int n, const Eigen::MatrixXd &cost_matrix, bool zero_start=false);
    double computeCost(const Eigen::MatrixXd &cost_matrix, const std::vector<int> &solution);
    double computeDistance(octomap::point3d a, octomap::point3d b);

public:
    TSPsolver();
    TSPsolver(int max_duration, distance_funcion_t distanceFunciton);
    ~TSPsolver();
    
    std::vector<int> solve(const Eigen::MatrixXd &cost_matrix, bool reuse_solution);
    std::vector<octomap::point3d> solve(octomath::Vector3 velocity);
    void removeFrontiers(std::vector<octomap::point3d> frontiers);
    void addFrontiers(std::vector<octomap::point3d> frontiers);
    void setStart(octomap::point3d position);

};




}

#endif