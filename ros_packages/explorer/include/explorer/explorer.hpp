#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/service_client_handler.h>

#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/GetPathSrv.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <tf2_eigen/tf2_eigen.h>

#include <unordered_set>
#include <vector>
#include <queue>
#include <iostream>
#include <memory>
#include <bits/stdc++.h>
#include <set>
#include <map>

#include <explorer_utils/utils.hpp>
#include <explorer_utils/timing.hpp>
#include <frontier_detector/FrontierArray.h>
#include <tsp_solver/SetStart.h>
#include <tsp_solver/Solve.h>
#include <path_planner/FindSimplifiedPath.h>
#include <skeleton_estimator/PrecomputedPath.h>

typedef enum
{
  STATE_IDLE,
  STATE_PATH_PLANNING,
  STATE_TRAJ_EXEC,
  STATE_COLLISION,
} State_t;


inline const std::string _state_names_[] = {"IDLE", "STATE_PATH_PLANNING", "STATE_TRAJ_EXEC", "STATE_COLLISION", "TSP_UPDATED", "WAITING", "FLYING"};


namespace explorer
{

  using OcTree_t          = octomap::OcTree;
  using OcTreeSharedPtr_t = std::shared_ptr<octomap::OcTree>;

  class Explorer : public nodelet::Nodelet
  {
  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh_;

    bool        is_initialized_ = false;
    std::string _uav_name_;

    // params
    // double _replanning_distance_;
    double _flight_free_distance_;
    double _skip_path_point_distance_;
    double _max_length_after_first_viewpoint_;
    double _scale_points_;
    double _scale_lines_;
    double _rate_main_timer_;
    double _rate_path_timer_;
    double _rate_fast_timer_;
    double _replan_request_cooldown_;
    double _stuck_distance_;
    double _stuck_duration_;
    double _latency_compensation_;
    double _latency_ema_alpha_;
    double latency_estimate_; // discounted average of measured replan latency, seeded with _latency_compensation_; see pathAndTrajectory()
    bool   _refine_enabled_;
    double _refine_radius_;
    int    _refine_max_clusters_;
    int    _refine_max_viewpoints_;
    double _refine_w_consistency_;
    double _refine_w_coverage_;
    bool   _pre_enabled_;
    double _pre_sample_step_;
    double _pre_clearance_;
    double _pre_reach_tolerance_;
    double _pre_min_path_length_;
    double _pre_failure_backoff_;
    double _pre_replan_period_;
    double _pre_join_distance_;
    double _pre_follow_clearance_;
    double _pre_follow_horizon_;
    double _pre_max_known_gap_;
    double _pre_off_path_distance_;
    int    _pre_max_entry_attempts_;
    int    _pre_max_no_progress_;
    // flight zone from world_info.yaml (zone_x/zone_y/zone_z) + zone/global/floor, the same AABB path_planner samples roadmap nodes in;
    // precomputed path samples outside it are unreachable and skipped (LABEL_OUTSIDE)
    double               _flight_zone_width_x_;
    double               _flight_zone_width_y_;
    double               _flight_zone_height_;
    double               _flight_zone_floor_;
    explorer_utils::AABB flight_zone_;

    struct Path_t
    {
      int                            id = 0;
      std::vector<octomap::point3d> points;
      std::vector<double>            headings; // [rad] per point; filled only by precomputed path following (empty = heading left to the trajectory generator)
    };

    Path_t            path_;
    int               next_path_id_ = 0;
    octomap::point3d current_viewpoint_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_viewpoint_candidates_;

    // latest frontier_detector viewpoints per frontier id (sorted by coverage, best first), cached by callbackFrontiers() for refineLocalTour()
    struct Viewpoint_t
    {
      octomap::point3d position;
      int              coverage;
    };
    std::mutex                                                  mutex_frontier_viewpoints_;
    std::unordered_map<unsigned long, std::vector<Viewpoint_t>> frontier_viewpoints_;


    // precomputed coverage path (skeleton_estimator/precomputed_path), followed by makePathPrecomputed() where it enters unknown space
    struct PrecomputedWaypoint_t
    {
      octomap::point3d position;
      double           yaw;
      bool             is_viewpoint;
    };
    // the precomputed path resampled every precomputed/sample_step (resamplePath()); labels and entries live on these samples
    struct PathSample_t
    {
      octomap::point3d position;
      double           yaw; // yaw of the waypoint this sample's segment leads to (along +index)
      int              wp;  // index of the waypoint this sample was resampled from (its segment's start)
      double           u;   // path parameter: wp + fraction along the segment; unlike the position, unaffected by transform drift
    };
    typedef enum
    {
      LABEL_KNOWN,    // whole precomputed/clearance ball known free (path_planner's own reachability test)
      LABEL_UNKNOWN,  // unknown itself, or an unknown voxel (and no occupied one) within precomputed/clearance
      LABEL_OCCUPIED, // occupied itself, or an occupied voxel within precomputed/clearance (too close to the structure), skipped
      LABEL_OUTSIDE,  // outside flight_zone_ (no roadmap nodes there, path_planner can't reach it) or on segment 0, skipped
    } WaypointLabel_t;
    static bool isSkippedLabel(WaypointLabel_t label) { return label == LABEL_OCCUPIED || label == LABEL_OUTSIDE; }
    // makePathPrecomputed() outcome: new path_ / nothing to follow (frontier fallback, or keep the current trajectory for an advance replan)
    typedef enum
    {
      PRE_PATH,
      PRE_NONE,
    } PrecomputedResult_t;
    // precomputed following mode: NONE = not on it, GOTO = flying to an entry anchor via path_planner, FOLLOW = flying along the path itself
    typedef enum
    {
      PRE_MODE_NONE,
      PRE_MODE_GOTO,
      PRE_MODE_FOLLOW,
    } PrecomputedMode_t;
    // planGoto() outcome: new path_ to an anchor / the UAV is already at an anchor (FOLLOW was entered) / no entry could be planned
    typedef enum
    {
      GOTO_PATH,
      GOTO_FOLLOW,
      GOTO_NONE,
    } GotoResult_t;
    // planFollow() outcome: new path_ along the path / segment done (no more unknown on it, blocked, or off the path)
    typedef enum
    {
      FOLLOW_PATH,
      FOLLOW_DONE,
    } FollowResult_t;
    // where the path enters unknown space: anchor = index of the last KNOWN sample, dir = +1/-1 direction along the tour indices
    typedef std::pair<int, int> PathEntry_t;
    std::mutex                         mutex_precomputed_;
    std::vector<PrecomputedWaypoint_t> precomputed_path_;           // latest message's waypoints (world frame), guarded by mutex_precomputed_
    std::string                        precomputed_frame_;
    int                                precomputed_path_id_ = -1;
    bool                               precomputed_stable_  = false;
    bool                               precomputed_reset_   = false; // path_id changed -> different path, drop the following state below
    // following state, only touched from makePathPrecomputed() (replans are serialized through the replan_request self-call)
    // entries are kept by path parameter u, not position (drifts with the transform) or sample index (shifts with segment lengths)
    struct EntryRecord_t
    {
      double           u;
      int              dir;
      int              no_progress; // times the UAV was at its anchor without it advancing; dropped at max_no_progress
      ros::Time        retry_after; // skipped until then after a planning failure (precomputed/failure_backoff)
    };
    std::optional<std::pair<double, int>>           committed_entry_; // GOTO: anchor path parameter u, dir
    std::vector<EntryRecord_t>                      entry_records_;
    std::atomic<PrecomputedMode_t>                  pre_mode_{PRE_MODE_NONE}; // read by timerReplanner()/monitoring, written by replans only
    double                                          pre_entry_u_    = 0.0;    // FOLLOW: u of the entry anchor the segment was joined at
    double                                          pre_progress_u_ = 0.0;    // FOLLOW: u of the sample the UAV was last at
    int                                             pre_dir_        = 1;      // FOLLOW: direction along the sample indices
    std::atomic<bool>                  precomputed_has_targets_{false}; // last makePathPrecomputed() saw a usable entry; blocks end of exploration
    std::atomic<bool>                  following_precomputed_{false};   // the trajectory being flown came from makePathPrecomputed() (-> periodic advance replans)
    std::atomic<bool>                  advance_replan_{false};          // the replan in flight is a periodic advance replan (set by requestReplan(), read by makePath())

    std::mutex                                mutex_octree_;
    std::shared_ptr<OcTree_t>                 octree_ = nullptr;
    std::string                               octree_frame_;
    std::shared_ptr<mrs_lib::BatchVisualizer> bv_path_;

    std::atomic<State_t> state_;
    void                 changeState(const State_t new_state);

    std::atomic<bool> map_update_;
    std::atomic<bool> map_ready_;
    std::atomic<bool> tsp_ready_;
    // end of exploration: frontier count of the latest frontier_detector message, whether any message was non-empty yet, whether the
    // latest makePath() ended in finalizePath()'s "no suitable frontiers" (every remaining tour viewpoint already reached), and the
    // (sticky) finished flag set by timerReplanner() and published on exploration_finished_out
    std::atomic<size_t> num_frontiers_{0};
    std::atomic<bool>   had_frontiers_{false};
    std::atomic<bool>   no_suitable_frontiers_{false};
    std::atomic<bool>   exploration_finished_{false};
    // bool              first_path_planend_;
    bool              bv_map_frame_set_;
    bool              braking_;
    // fail-safe: consecutive "temp path not found" failures in buildLocalPath(); after kMaxTempPathFailures a 1 m random-direction step is flown instead
    static constexpr int kMaxTempPathFailures = 10;
    int               temp_path_fail_count_ = 0;
    std::mt19937      rng_{std::random_device{}()};
    octomap::point3d last_free_point_;
    // std::atomic<bool> valid_path_;
    octomap::point3d position_before_map_update_;
    octomap::point3d goal_;
    octomap::point3d next_goal_;
    std::shared_ptr<OcTree_t> big_tree_;


    // subscribers
    mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand> sh_tracker_cmd_;
    mrs_lib::SubscribeHandler<octomap_msgs::Octomap>    sh_octomap_;
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
    mrs_lib::SubscribeHandler<frontier_detector::FrontierArray> sh_frontiers_;
    mrs_lib::SubscribeHandler<skeleton_estimator::PrecomputedPath> sh_precomputed_path_;

    // publishers
    ros::Publisher pub_reference_;
    ros::Publisher pub_exploration_finished_;
    mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>             sc_get_trajectory_;
    mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference_;
    mrs_lib::ServiceClientHandler<tsp_solver::SetStart>             sc_tsp_set_start_;
    mrs_lib::ServiceClientHandler<tsp_solver::Solve>                sc_tsp_solve_;
    mrs_lib::ServiceClientHandler<path_planner::FindSimplifiedPath> sc_path_planner_find_simplified_path_;
    mrs_lib::ServiceClientHandler<std_srvs::Trigger>                sc_replan_request_;
    ros::Time                                                       last_replan_request_time_ = ros::Time(0);

    // stuck detection (checkStuck())
    octomap::point3d last_moved_position_;
    ros::Time        last_moved_time_ = ros::Time(0);
    bool             stuck_check_reset_ = true;

    // service servers
    ros::ServiceServer ss_replan_request_;
    bool                callbackReplanRequest(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    // timeout callbacks
    void timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg);
    void timeoutOctomap(const std::string& topic,
                        const ros::Time&   last_msg);

    // subscriber callbacks
    void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
    void callbackFrontiers(const frontier_detector::FrontierArray::ConstPtr msg);
    void callbackPrecomputedPath(const skeleton_estimator::PrecomputedPath::ConstPtr msg);
    void controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);


    // timers
    ros::Timer timer_main_;
    void       timerMain([[maybe_unused]] const ros::TimerEvent& evt);
    // ros::Timer timer_frontiers_;
    // void       timer_frontiers_([[maybe_unused]] const ros::TimerEvent& evt);
    // ros::Timer timer_path_;
    // void       timerPath([[maybe_unused]] const ros::TimerEvent& evt);
    // void       timerPath([[maybe_unused]] const ros::TimerEvent& evt);
    ros::Timer timer_replanner_;
    void       timerReplanner([[maybe_unused]] const ros::TimerEvent& evt);

    

    std::unique_ptr<mrs_lib::Transformer> transformer_;

    // per-phase wall-clock timings published on ~timing/<function>/<phase> (ms), for live plotting
    std::unique_ptr<explorer_utils::TimingPublisher> timing_;

    std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> getPosition();
    std::optional<mrs_msgs::MpcPredictionFullState>                   getFullStatePrediction();
    std::optional<OcTreeSharedPtr_t>                                  msgToMap(const octomap_msgs::OctomapConstPtr octomap);
    bool                                                              makePath();
    bool  getCurrentStateAndPrediction(octomap::point3d& start_coord, mrs_msgs::MpcPredictionFullState& prediction, octomath::Vector3& velocity);
    void  compensateLatency(const mrs_msgs::MpcPredictionFullState& prediction, octomap::point3d& start_coord, octomath::Vector3& velocity);
    bool  checkImminentCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction);
    // bool  nearGoal(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction);
    bool  goalWithinLatency(const mrs_msgs::MpcPredictionFullState& prediction);
    bool  checkStuck(const octomap::point3d& start_coord);
    bool  solveTsp(const octomath::Vector3& velocity, std::vector<octomap::point3d>& glob_path, std::vector<unsigned long>& tour_ids);
    void  refineLocalTour(const octomap::point3d& start_coord, const octomath::Vector3& velocity, std::vector<octomap::point3d>& glob_path,
                          const std::vector<unsigned long>& tour_ids);
    void  visualizeGlobalPath(const std::vector<octomap::point3d>& glob_path);
    bool  buildLocalPath(const octomap::point3d& start_coord, const std::vector<octomap::point3d>& glob_path, const octomath::Vector3& velocity,
                          std::vector<octomap::point3d>& path, std::vector<octomap::point3d>& sub_global_path);
    bool  findSimplifiedPath(const octomap::point3d& start, const octomap::point3d& goal, const octomath::Vector3& velocity,
                             std::vector<octomap::point3d>& segment);
    void  randomStepPath(const octomap::point3d& start_coord, std::vector<octomap::point3d>& path, std::vector<octomap::point3d>& sub_global_path);
    bool  finalizePath(std::vector<octomap::point3d>& path, const std::vector<octomap::point3d>& sub_global_path);
    PrecomputedResult_t makePathPrecomputed(const octomap::point3d& start_coord, const octomath::Vector3& velocity, bool advance_only);
    GotoResult_t   planGoto(const std::vector<PathSample_t>& samples, const std::vector<WaypointLabel_t>& labels, const octomap::point3d& start_coord,
                            const octomath::Vector3& velocity);
    FollowResult_t planFollow(const std::vector<PathSample_t>& samples, const std::vector<WaypointLabel_t>& labels,
                              const std::vector<PrecomputedWaypoint_t>& wps, const octomap::point3d& start_coord, bool advance_only);
    void           enterFollow(double entry_u, double progress_u, int dir);
    void           resetPrecomputedMode();
    bool           occupiedOnlyChecks() const { return following_precomputed_ && pre_mode_ == PRE_MODE_FOLLOW; }
    static bool    occupiedNear(const OcTree_t& tree, const octomap::point3d& c, double r);
    std::vector<PathSample_t>    resamplePath(const std::vector<PrecomputedWaypoint_t>& wps) const;
    std::vector<WaypointLabel_t> labelSamples(const std::vector<PathSample_t>& samples);
    static int                   nextFollowed(const std::vector<WaypointLabel_t>& labels, int i, int dir);
    static std::vector<PathEntry_t> findEntries(const std::vector<WaypointLabel_t>& labels);
    EntryRecord_t& entryRecord(const std::vector<PathSample_t>& samples, int i, int dir);
    static int     nearestSample(const std::vector<PathSample_t>& samples, double u);
    bool  planEntry(const octomap::point3d& start_coord, const octomath::Vector3& velocity, const octomap::point3d& target, double yaw,
                    std::vector<octomap::point3d>& path, std::vector<double>& headings);
    std::optional<mrs_msgs::TrajectoryReference>                     makeTrajectory(const Path_t& path);
    bool                                                              publishTrajectory(const mrs_msgs::TrajectoryReference& trajectory, int id);
    bool                                                              checkTrajectoryCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction);
    void                                                              requestReplan(bool advance_only = false);
    bool pathAndTrajectory();

  };

}  // namespace explorer
