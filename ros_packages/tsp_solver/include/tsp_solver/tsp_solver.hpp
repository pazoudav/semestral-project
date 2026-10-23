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

#include <explorer_utils/utils.hpp>
#include <frontier_detector/FrontierArray.h>
#include <path_planner/FindSimplifiedPath.h>

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
    // Previous solve's tour as stable node ids (START_ID first); seeds the next solve's LKH initial tour.
    std::vector<unsigned long> prev_solution_;
    // Node id at each row/column of cost_matrix_, snapshotted with it so tour indices can be mapped to/from ids.
    std::vector<unsigned long> node_ids_;
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

    // client for path_planner's ~find_simplified_path_in, used to derive real (roadmap, raycast-simplified) travel distances between nodes
    mrs_lib::ServiceClientHandler<path_planner::FindSimplifiedPath> sc_find_simplified_path_;

    std::string GlobalDir_      = ros::package::getPath("tsp_solver") + "/LKH/";
    std::string GlobalSolver_   = GlobalDir_ + "LKH";
    std::string GlobalPar_      = GlobalDir_ + "params.txt";
    std::string GlobalProF_     = GlobalDir_ + "cost_matrix.txt";
    std::string GlobalResult_   = GlobalDir_ + "solution.txt";
    // Comparison-only LKH run: same problem/settings as GlobalPar_ but without INITIAL_TOUR_FILE, writing to its own solution file.
    std::string GlobalParNoInit_    = GlobalDir_ + "params_no_init.txt";
    std::string GlobalResultNoInit_ = GlobalDir_ + "solution_no_init.txt";
    std::string GlobalInitTour_ = GlobalDir_ + "initial_tour.txt";
    int         GlobalRuns_;
    float       vp_weight_;
    float       dir_weight_;
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

    // Single find_simplified_path_in call from `start` to every point in `goals` (one shared Dijkstra search, per path_planner's own batching); returns each goal's raycast-simplified path length, or BIG_DISTANCE where unreachable/the call failed.
    std::vector<double> findPathDistances(const octomap::point3d &start, const std::vector<octomap::point3d> &goals);
    // Rebuilds cost_matrix_/viewpoint_positions_/isAccesible_ from the flat distance graph. Must be called with mutex_dist_graph_ held.
    void constructDistanceMatrix();

    // Writes the LKH .par control files (problem/output file paths, time limit, run count): params.txt (with the initial tour) and params_no_init.txt (comparison run).
    void GlobalParWrite();
    // Writes the cost matrix out as an explicit-weight ATSP problem file for LKH.
    void GlobalProblemWrite(Eigen::MatrixXd& costMat);
    // Builds the 0-based initial tour handed to LKH (start node first). Back half: the previous tour's tail, kept while its nodes still exist
    // (walking from the back, stopping at the first removed one). Front half: greedy nearest-neighbor from the start over the remaining nodes.
    // On the first solve there is no previous tour, so this is a pure greedy tour.
    std::vector<int> buildInitialTour() const;
    // Writes a 0-based index tour as an LKH TOUR file (1-based ids) for the INITIAL_TOUR_FILE parameter.
    void InitialTourWrite(const std::vector<int>& tour);
    // Parses an LKH output tour file's TOUR_SECTION into a 0-based node index sequence.
    std::vector<int> GlobalResultsRead(const std::string& result_file);
    // Runs the bundled LKH binary with the given .par file and returns the tour it writes to result_file (empty on failure).
    std::vector<int> runLKH(const std::string& par_file, const std::string& result_file);
    // Total cost of a tour under cost_matrix_, including the closing edge back to the first node (zero-cost for this ATSP formulation).
    double tourLength(const std::vector<int>& tour) const;
    // Writes the cost-matrix problem file and initial tour, solves with LKH with and without the initial tour (logging both tour lengths), and returns the initialized run's tour.
    std::vector<int> LKHSolve();
    // Adds nearest-guiding-viewpoint distance from the KD-tree to the start row of the cost matrix (currently unused, call site commented out).
    void costmatrixViewpointAdjustment();


public:
    TSPsolver(int max_duration, ros::NodeHandle &nh, double lkh_time_limit, int lkh_runs, float vp_weight, float dir_weight);
    ~TSPsolver();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints_;

    // Entry point used by the nodelet: snapshots the distance graph (briefly under mutex_dist_graph_),
    // then runs LKH against that snapshot (under mutex_solve_ only, so it doesn't block frontier/start
    // updates) and returns the ordered viewpoint positions.
    std::vector<octomap::point3d> solve(octomath::Vector3 velocity);
    // Reconciles the distance graph with an incoming FrontierArray: drops nodes for frontiers no longer present and adds/distances new ones.
    void syncFrontiers(const frontier_detector::FrontierArray::ConstPtr& msg);
    // (Re)sets the start/current-position node (id START_ID, always index 0) and recomputes its distances to every other node.
    void setStart(octomap::point3d position);
    // Sets the KD-tree of skeleton-derived guiding viewpoints used to bias the cost matrix toward them.
    void setKDtreeInput(pcl::PointCloud<pcl::PointXYZ>::Ptr guiding_viewpoints);


};




}

#endif
