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

#include <mrs_lib/service_client_handler.h>

#include <octomap_planner_utils/utils.hpp>
#include <frontier_detection/FrontierArray.h>
#include <path_planning/FindSimplifiedPath.h>

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
    octomath::Vector3 start_velocity_;
    std::vector<bool> isAccesible_;
    std::map<unsigned long, planner_t> dist_map_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_;
    bool pcset_ = false;

    // client for path_planning's ~find_simplified_path_in, used to derive real (roadmap, raycast-simplified) travel distances between nodes
    mrs_lib::ServiceClientHandler<path_planning::FindSimplifiedPath> sc_find_simplified_path_;

    std::string GlobalDir_      = ros::package::getPath("tsp_solver") + "/LKH/";
    std::string GlobalSolver_   = GlobalDir_ + "LKH";
    std::string GlobalPar_      = GlobalDir_ + "params.txt";
    std::string GlobalProF_     = GlobalDir_ + "cost_matrix.txt";
    std::string GlobalResult_   = GlobalDir_ + "solution.txt";
    int         GlobalRuns_;
    double      GlobalTimeLimit_;
    int         precision_      = 10;


    // Single find_simplified_path_in call from `start` to every point in `goals` (one shared Dijkstra search, per path_planning's own batching); returns each goal's raycast-simplified path length, or BIG_DISTANCE where unreachable/the call failed.
    std::vector<double> findPathDistances(const octomap::point3d &start, const std::vector<octomap::point3d> &goals);
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
    TSPsolver(int max_duration, ros::NodeHandle &nh, double lkh_time_limit, int lkh_runs);
    ~TSPsolver();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints_;

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
