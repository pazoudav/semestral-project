#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <vector>
#include <numeric>
#include <algorithm>
#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>


#include <octomap_planner_utils/utils.hpp>
#include <frontier_detection/FrontierArray.h>

namespace tsp_solver
{



struct planner_t
{
    unsigned long id;
    octomap::point3d position;
    bool isAccesible;
    std::map<unsigned long, float> distances;
};


class TSPsolver
{
private:
    /* data */
    ros::Time start_;
    ros::Duration duration_;
    Eigen::MatrixXd cost_matrix_;
    std::vector<int> prev_solution_;
    // std::vector<unsigned long> viewpoint_id_;
    std::vector<octomap::point3d> viewpoint_positions_;
    octomap::point3d start_position_;
    octomap_planner_utils::distance_funcion_t distanceFunciton_;
    octomath::Vector3 start_velocity_;
    std::vector<bool> isAccesible_;
    std::map<unsigned long, planner_t> dist_map_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_;
    bool pcset_ = false;

    std::string GlobalDir_      = ros::package::getPath("tsp_solver") + "/LKH/";
    std::string GlobalSolver_   = GlobalDir_ + "LKH";
    std::string GlobalPar_      = GlobalDir_ + "params.txt";
    std::string GlobalProF_     = GlobalDir_ + "cost_matrix.txt";
    std::string GlobalResult_   = GlobalDir_ + "solution.txt";
    int         GlobalRuns_     = 1;
    int         precision_      = 10;


    std::vector<int> generateRadnSolution(int n);
    std::vector<int> generateGreedySolution(int n, const Eigen::MatrixXd &cost_matrix, bool zero_start=false);
    double computeCost(const Eigen::MatrixXd &cost_matrix, const std::vector<int> &solution);
    double computeDistance(octomap::point3d a, octomap::point3d b);
    void constructDistanceMatrix();

    void GlobalParWrite();
    void GlobalProblemWrite(Eigen::MatrixXd& costMat);
    std::vector<int> GlobalResultsRead();
    std::vector<int> LKHSolve();
    void costmatrixViewpointAdjustment();


public:
    TSPsolver(int max_duration);
    ~TSPsolver();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints_;

    std::vector<int> solve(const Eigen::MatrixXd &cost_matrix, bool reuse_solution);
    std::vector<octomap::point3d> solve(octomath::Vector3 velocity);
    void syncFrontiers(const frontier_detection::FrontierArray::ConstPtr& msg);
    void setStart(octomap::point3d position);
    void setKDtreeInput(pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints);


};




}

#endif
