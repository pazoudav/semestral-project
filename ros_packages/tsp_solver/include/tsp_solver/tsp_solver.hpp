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
#include <unordered_map>
#include <string>
#include <fstream>
#include <iostream>
#include <mutex>
#include <pcl/kdtree/kdtree_flann.h>

#include <mrs_lib/service_client_handler.h>

#include <octomap_planner_utils/utils.hpp>
#include <frontier_detection/FrontierArray.h>
#include <path_planning/FindSimplifiedPath.h>

namespace tsp_solver
{



// A single viewpoint/start node in the TSP distance graph: its 3D position and whether it is
// currently reachable. Pairwise distances live in TSPsolver's flat distances_ matrix, at the
// row/column each node is assigned in nodes_/id_to_index_.
struct planner_t
{
    unsigned long id;
    octomap::point3d position;
    bool isAccesible;
};


// Orders exploration viewpoints (frontiers) into a single tour: maintains a persistent
// distance graph over incoming frontiers, builds a cost matrix from it, and solves the
// resulting (A)TSP via the bundled LKH binary.
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

    // Persistent distance graph, stored flat instead of as nested std::maps: nodes_[i] and row/column i
    // of distances_ describe the same node, kept in sync via id_to_index_. START_ID is always index 0
    // (see reserveStartSlot()) once set, since constructDistanceMatrix()/the ATSP formulation rely on
    // column 0 being the start node. Any other node is removed by swapping the last node into its slot
    // (removeNode), so indices stay dense without an O(n) shift.
    std::vector<planner_t> nodes_;
    Eigen::MatrixXf distances_;
    std::unordered_map<unsigned long, int> id_to_index_;

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_;
    bool pcset_ = false;

    // Guards nodes_/distances_/id_to_index_: held by syncFrontiers, setStart, and (briefly) the
    // dist-graph snapshot phase of solve() — kept separate from mutex_solve_ so a frontier/start
    // update is never stalled behind LKH's own (potentially ~second-scale) runtime.
    std::mutex mutex_dist_graph_;
    // Guards cost_matrix_/viewpoint_positions_/isAccesible_ and the LKH input/output files; held for
    // the full duration of solve() so concurrent solve() calls can't race on the same LKH files.
    std::mutex mutex_solve_;

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

    // Grows distances_ (conservativeResize, which preserves existing entries) so it can hold at least `needed` nodes.
    void ensureCapacity(int needed);
    // Appends a new node at the end of nodes_, growing distances_ if needed; returns its assigned index.
    int addNode(unsigned long id, const octomap::point3d &position, bool isAccesible);
    // Removes the node with the given id by swapping the last node into its slot (O(1) index churn
    // instead of an O(n) shift); no-op if id is unknown. Never called with START_ID (see class comment).
    void removeNode(unsigned long id);
    // Writes the same distance into both (i,j) and (j,i), matching the graph's assumed-symmetric distances.
    void setDistance(int i, int j, float value);
    // One-time setup (first ever setStart() call): inserts the START_ID node at index 0, shifting every
    // already-present node (and the matrix rows/cols they occupy) up by one to make room.
    void reserveStartSlot();

    // Single find_simplified_path_in call from `start` to every point in `goals` (one shared Dijkstra search, per path_planning's own batching); returns each goal's raycast-simplified path length, or BIG_DISTANCE where unreachable/the call failed.
    std::vector<double> findPathDistances(const octomap::point3d &start, const std::vector<octomap::point3d> &goals);
    // Rebuilds cost_matrix_/viewpoint_positions_/isAccesible_ from the flat distance graph. Must be called with mutex_dist_graph_ held.
    void constructDistanceMatrix();

    // Writes the LKH .par control file (problem/output file paths, time limit, run count).
    void GlobalParWrite();
    // Writes the cost matrix out as an explicit-weight ATSP problem file for LKH.
    void GlobalProblemWrite(Eigen::MatrixXd& costMat);
    // Parses LKH's solution.txt TOUR_SECTION into a 0-based node index sequence.
    std::vector<int> GlobalResultsRead();
    // Writes the cost-matrix problem file, shells out to the bundled LKH binary, and reads back the resulting tour.
    std::vector<int> LKHSolve();
    // Adds nearest-guiding-viewpoint distance from the KD-tree to the start row of the cost matrix (currently unused, call site commented out).
    void costmatrixViewpointAdjustment();


public:
    TSPsolver(int max_duration, ros::NodeHandle &nh, double lkh_time_limit, int lkh_runs);
    ~TSPsolver();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints_;

    // Entry point used by the nodelet: snapshots the distance graph (briefly under mutex_dist_graph_),
    // then runs LKH against that snapshot (under mutex_solve_ only, so it doesn't block frontier/start
    // updates) and returns the ordered viewpoint positions.
    std::vector<octomap::point3d> solve(octomath::Vector3 velocity);
    // Reconciles the distance graph with an incoming FrontierArray: drops nodes for frontiers no longer present and adds/distances new ones.
    void syncFrontiers(const frontier_detection::FrontierArray::ConstPtr& msg);
    // (Re)sets the start/current-position node (id START_ID, always index 0) and recomputes its distances to every other node.
    void setStart(octomap::point3d position);
    // Sets the KD-tree of skeleton-derived guiding viewpoints used to bias the cost matrix toward them.
    void setKDtreeInput(pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints);


};




}

#endif
