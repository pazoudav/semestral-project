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



// A single viewpoint/start node in the TSP distance graph: its 3D position, whether it is
// currently reachable, and its precomputed distance to every other node (keyed by node id).
struct planner_t
{
    unsigned long id;
    octomap::point3d position;
    bool isAccesible;
    std::map<unsigned long, float> distances;
};


// Orders exploration viewpoints (frontiers) into a single tour: maintains a persistent
// distance graph over incoming frontiers, builds a cost matrix from it, and solves the
// resulting (A)TSP via the bundled LKH binary (with a greedy/3-opt fallback if LKH is unused).
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


    // Returns a random permutation of node indices [0, n), used as a fallback initial tour.
    std::vector<int> generateRadnSolution(int n);
    // Builds a tour by repeatedly stepping to the nearest not-yet-visited, accessible node (nearest-neighbor heuristic).
    std::vector<int> generateGreedySolution(int n, const Eigen::MatrixXd &cost_matrix, bool zero_start=false);
    // Sums the cyclic tour cost from the cost matrix, adding heading-change/height penalties near the start node.
    double computeCost(const Eigen::MatrixXd &cost_matrix, const std::vector<int> &solution);
    // Evaluates distanceFunciton_ between two points, mapping an invalid path to BIG_DISTANCE.
    double computeDistance(octomap::point3d a, octomap::point3d b);
    // Rebuilds cost_matrix_/viewpoint_positions_/isAccesible_ from dist_map_, applying start-heading and KD-tree viewpoint weighting to row 0.
    void constructDistanceMatrix();

    // Writes the LKH .par control file (problem/output file paths, time limit, run count).
    void GlobalParWrite();
    // Writes the cost matrix out as an explicit-weight ATSP problem file for LKH.
    void GlobalProblemWrite(Eigen::MatrixXd& costMat);
    // Parses LKH's solution.txt TOUR_SECTION into a 0-based node index sequence.
    std::vector<int> GlobalResultsRead();
    // Writes the LKH input files, shells out to the bundled LKH binary, and reads back the resulting tour.
    std::vector<int> LKHSolve();
    // Adds nearest-guiding-viewpoint distance from the KD-tree to the start row of the cost matrix (currently unused, call site commented out).
    void costmatrixViewpointAdjustment();


public:
    TSPsolver(int max_duration);
    ~TSPsolver();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints_;

    // Fallback local-search solver: greedy nearest-neighbor construction followed by randomized 3-opt moves until duration_ elapses.
    std::vector<int> solve(const Eigen::MatrixXd &cost_matrix, bool reuse_solution);
    // Entry point used by the nodelet: builds the distance matrix for the current start heading, runs LKH, and returns the ordered viewpoint positions.
    std::vector<octomap::point3d> solve(octomath::Vector3 velocity);
    // Reconciles dist_map_ with an incoming FrontierArray: drops nodes for frontiers no longer present and adds/distances new ones.
    void syncFrontiers(const frontier_detection::FrontierArray::ConstPtr& msg);
    // (Re)inserts the start/current-position node (id START_ID) into dist_map_ and recomputes its distances to every other node.
    void setStart(octomap::point3d position);
    // Sets the KD-tree of skeleton-derived guiding viewpoints used to bias the cost matrix toward them.
    void setKDtreeInput(pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints);


};




}

#endif
