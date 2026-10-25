
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

#include <mrs_octomap_tools/octomap_methods.h>
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

#include <explorer_utils/utils.hpp>
#include <frontier_detector/FrontierArray.h>
#include <tsp_solver/SetStart.h>
#include <tsp_solver/Solve.h>
#include <path_planner/FindSimplifiedPath.h>

typedef enum
{
  STATE_IDLE,
  STATE_PATH_PLANNING,
  STATE_TRAJ_EXEC,
  STATE_COLLISION,
} State_t;


const std::string _state_names_[] = {"IDLE", "STATE_PATH_PLANNING", "STATE_TRAJ_EXEC", "STATE_COLLISION", "TSP_UPDATED", "WAITING", "FLYING"};


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
    double _replanning_distance_;
    double _flight_free_distance_;
    double _skip_path_point_distance_;
    double _max_length_after_first_viewpoint_;
    double _free_space_dia_;
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

    struct Path_t
    {
      int                            id = 0;
      std::vector<octomap::point3d> points;
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


    std::mutex                                mutex_octree_;
    std::shared_ptr<OcTree_t>                 octree_ = nullptr;
    std::string                               octree_frame_;
    std::shared_ptr<mrs_lib::BatchVisualizer> bv_path_;

    std::atomic<State_t> state_;
    void                 changeState(const State_t new_state);

    std::atomic<bool> map_update_;
    std::atomic<bool> map_ready_;
    std::atomic<bool> tsp_ready_;
    // end of exploration: frontier count of the latest frontier_detector message, whether any message was non-empty yet, and the
    // (sticky) finished flag set by timerReplanner() and published on exploration_finished_out
    std::atomic<size_t> num_frontiers_{0};
    std::atomic<bool>   had_frontiers_{false};
    std::atomic<bool>   exploration_finished_{false};
    bool              first_path_planend_;
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

    std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> getPosition();
    std::optional<mrs_msgs::MpcPredictionFullState>                   getFullStatePrediction();
    std::optional<OcTreeSharedPtr_t>                                  msgToMap(const octomap_msgs::OctomapConstPtr octomap);
    bool                                                              makePath();
    bool  getCurrentStateAndPrediction(octomap::point3d& start_coord, mrs_msgs::MpcPredictionFullState& prediction, octomath::Vector3& velocity);
    void  compensateLatency(const mrs_msgs::MpcPredictionFullState& prediction, octomap::point3d& start_coord, octomath::Vector3& velocity);
    bool  checkImminentCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction);
    bool  nearGoal(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction);
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
    std::optional<mrs_msgs::TrajectoryReference>                     makeTrajectory(const Path_t& path);
    bool                                                              publishTrajectory(const mrs_msgs::TrajectoryReference& trajectory, int id);
    bool                                                              checkTrajectoryCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction);
    void                                                              requestReplan();
    bool pathAndTrajectory();

  };

  // nodelet entry point: loads params from explorer.yaml, wires up subscribers/publishers/service clients and the replan_request_in server,
  // starts timer_main_/timer_replanner_, and sets state_ to STATE_IDLE
  void Explorer::onInit()
  {
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ros::Time::waitForValid();

    ROS_INFO("[MrsExplorer]: initializing");

    mrs_lib::ParamLoader param_loader(nh_, "MrsExplorer");

    param_loader.loadParam("uav_name", _uav_name_);

    param_loader.loadParam("planning/replanning_distance",_replanning_distance_);
    param_loader.loadParam("planning/flight_free_distance",_flight_free_distance_);
    param_loader.loadParam("planning/skip_path_point_distance",_skip_path_point_distance_);
    param_loader.loadParam("planning/max_length_after_first_viewpoint", _max_length_after_first_viewpoint_);
    param_loader.loadParam("planning/replan_request_cooldown",_replan_request_cooldown_);
    param_loader.loadParam("planning/stuck_distance",_stuck_distance_);
    param_loader.loadParam("planning/stuck_duration",_stuck_duration_);
    param_loader.loadParam("planning/latency_compensation",_latency_compensation_);
    param_loader.loadParam("planning/latency_ema_alpha",_latency_ema_alpha_);

    param_loader.loadParam("local_refinement/enabled",        _refine_enabled_);
    param_loader.loadParam("local_refinement/radius",         _refine_radius_);
    param_loader.loadParam("local_refinement/max_clusters",   _refine_max_clusters_);
    param_loader.loadParam("local_refinement/max_viewpoints", _refine_max_viewpoints_);
    param_loader.loadParam("local_refinement/w_consistency",  _refine_w_consistency_);
    param_loader.loadParam("local_refinement/w_coverage",     _refine_w_coverage_);

    param_loader.loadParam("prm/free_space_diameter",_free_space_dia_);

    param_loader.loadParam("viz/scale/points", _scale_points_);
    param_loader.loadParam("viz/scale/lines", _scale_lines_);

    param_loader.loadParam("timer_rates/main",       _rate_main_timer_);
    param_loader.loadParam("timer_rates/path",       _rate_path_timer_);
    param_loader.loadParam("timer_rates/fast",       _rate_fast_timer_);


    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[MrsExplorer]: Could not load all parameters");
      ros::shutdown();
    }

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = "MrsExplorer";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 1;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_tracker_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in", ros::Duration(3.0), &Explorer::timeoutTrackerCmd, this);
    sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", ros::Duration(5.0),
                                                                   &Explorer::timeoutOctomap,  this,
                                                                   &Explorer::callbackOctomap, this);
    sh_control_manager_diag_  = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "diagnostics_in", &Explorer::controlManagerDiagCallback, this);
    sh_frontiers_             = mrs_lib::SubscribeHandler<frontier_detector::FrontierArray>(shopts, "frontiers_in", &Explorer::callbackFrontiers, this);

    pub_reference_            = nh_.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);
    pub_exploration_finished_ = nh_.advertise<std_msgs::Bool>("exploration_finished_out", 1, true);

    sc_get_trajectory_            = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
    sc_trajectory_reference_      = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");
    sc_tsp_set_start_             = mrs_lib::ServiceClientHandler<tsp_solver::SetStart>(nh_, "set_start_out");
    sc_tsp_solve_                 = mrs_lib::ServiceClientHandler<tsp_solver::Solve>(nh_, "solve_out");
    sc_path_planner_find_simplified_path_ = mrs_lib::ServiceClientHandler<path_planner::FindSimplifiedPath>(nh_, "find_simplified_path_out");
    sc_replan_request_            = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "replan_request_out");

    ss_replan_request_ = nh_.advertiseService("replan_request_in", &Explorer::callbackReplanRequest, this);

    timer_main_       = nh_.createTimer(ros::Rate(_rate_main_timer_), &Explorer::timerMain,      this);
    // timer_path_       = nh_.createTimer(ros::Rate(_rate_path_timer_), &Explorer::timerPath,      this);
    timer_replanner_  = nh_.createTimer(ros::Rate(_rate_fast_timer_), &Explorer::timerReplanner, this);


    transformer_ = std::make_unique<mrs_lib::Transformer>("MrsExplorer");
    transformer_->setDefaultPrefix(_uav_name_);
    transformer_->retryLookupNewest(true);


    bv_path_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_path", "");
    bv_path_->setPointsScale(_scale_points_);
    bv_path_->setLinesScale(_scale_lines_*2.0);

    map_update_ = false;
    map_ready_ = false;
    tsp_ready_ = false;
    first_path_planend_ = false;
    braking_ = false;

    latency_estimate_ = _latency_compensation_;

    std_msgs::Bool finished_msg;
    finished_msg.data = false;
    pub_exploration_finished_.publish(finished_msg);

    is_initialized_ = true;
    bv_map_frame_set_ = false;
    state_ = STATE_IDLE;

    ROS_INFO("[MrsExplorer]: initialized");
  }


  // periodic main-loop timer (timer_rates/main); currently a no-op placeholder (all real work is driven by timerReplanner())
  void Explorer::timerMain([[maybe_unused]] const ros::TimerEvent& evt)
  {
    // ROS_ERROR_THROTTLE(1.0, "[MrsExplorer]: MAIN LOOP in '%s' STATE", _state_names_[state_].c_str());
    if (!is_initialized_ || !map_ready_) {
      return;
    }

  }


  // driver of the whole replanning loop (rate timer_rates/fast in explorer.yaml); waits for is_initialized_/map_ready_/tsp_ready_, then:
  // - once finished (sticky): does nothing; it becomes finished when frontier_detector has no frontiers left (after having had some) and
  //   checkStuck() reports the UAV is not moving, which sets exploration_finished_ and publishes true on exploration_finished_out
  // - STATE_IDLE: requestReplan() (retried every tick, rate-limited by planning/replan_request_cooldown, until a replan succeeds)
  // - STATE_TRAJ_EXEC: requestReplan() once nearGoal() or checkTrajectoryCollision() fires on the current MPC prediction,
  //   or forces STATE_IDLE if checkStuck() reports no progress
  // - any other state (STATE_PATH_PLANNING, i.e. a replan already in flight): does nothing
  void Explorer::timerReplanner([[maybe_unused]] const ros::TimerEvent& evt)
  {
    if (!is_initialized_) {
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: drone not initialized");
      return;
    }
    if (!map_ready_){
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: map not ready, cannot calculate PATH");
      return;
    }  
    if (!tsp_ready_){
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: TSP not ready, cannot calculate PATH");
      return;
    }  
    if (exploration_finished_){
      ROS_ERROR_THROTTLE(5.0, "[MrsExplorer]: exploration finished");
      // return;
    }

    octomap::point3d start_coord;
    mrs_msgs::MpcPredictionFullState prediction;
    octomath::Vector3 velocity;

    if (!getCurrentStateAndPrediction(start_coord, prediction, velocity))
    {
      ROS_WARN_THROTTLE(0.5, "[MrsExplorer]: cannot get state and prediction");
      return;
    }

    const bool stuck = checkStuck(start_coord);

    // end of exploration: frontier_detector has no frontiers left (after having had some) and the UAV is not moving
    if (had_frontiers_ && num_frontiers_ == 0 && stuck)
    {
      ROS_INFO("[MrsExplorer]: END OF EXPLORATION, no frontiers left and UAV is not moving");
      exploration_finished_ = true;
      std_msgs::Bool finished_msg;
      finished_msg.data = true;
      pub_exploration_finished_.publish(finished_msg);
      return;
    }

    if (state_ == STATE_IDLE)
    {
      ROS_INFO_THROTTLE(0.5, "[MrsExplorer]: idle state, start planning");
      requestReplan();
      return;
    }

    if (state_ != STATE_TRAJ_EXEC)
    {
      return;
    }

    if (nearGoal(start_coord, prediction))
    {
      ROS_WARN_THROTTLE(0.5, "[MrsExplorer]: goal reached -> replanning");
      requestReplan();
      return;
    }

    // check if current path from predition is in collision
    if (checkTrajectoryCollision(start_coord, prediction))
    {
      ROS_WARN_THROTTLE(0.5, "[MrsExplorer]: collision detected");
      requestReplan();
      return;
    }

    // force back to IDLE if the UAV hasn't made meaningful progress for a while 
    if (stuck)
    {
      ROS_ERROR("[MrsExplorer]: UAV stuck (moved < %.2f m in %.2f s) -> forcing IDLE", _stuck_distance_, _stuck_duration_);
      changeState(STATE_IDLE);
      return;
    }
  }

  // called from timerReplanner(); switches to STATE_PATH_PLANNING and makes the blocking replan_request_out call, which explorer.launch remaps onto
  // this node's own replan_request_in (-> callbackReplanRequest() -> pathAndTrajectory()); moves to STATE_TRAJ_EXEC (and re-arms checkStuck()) on
  // success or back to STATE_IDLE on failure
  // no-ops if called again within planning/replan_request_cooldown seconds of the last actual call, to avoid hammering the service (e.g. while sitting in STATE_IDLE at the fast timer's rate)
  void Explorer::requestReplan()
  {
    const ros::Time now = ros::Time::now();
    if ((now - last_replan_request_time_).toSec() < _replan_request_cooldown_) {
      return;
    }
    last_replan_request_time_ = now;

    changeState(STATE_PATH_PLANNING);
    std_srvs::Trigger replan_request_srv;
    sc_replan_request_.call(replan_request_srv);
    bool success = replan_request_srv.response.success;

    if (success) {
      stuck_check_reset_ = true;
      changeState(STATE_TRAJ_EXEC);
    }
    else {
      changeState(STATE_IDLE);
    }
  }

  // replan_request_in handler, reached via requestReplan()'s self-call; runs pathAndTrajectory() and reports its result in res.success
  // (res.message is always "acknowledged")
  bool Explorer::callbackReplanRequest([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
  {
    ROS_INFO("[MrsExplorer]: replan request received");

    bool success = pathAndTrajectory();

    res.success = success;
    res.message = "acknowledged";
    ROS_WARN("[MrsExplorer]: replan request processed: %s", success ? "SUCCESS" : "FAIL");
    return true;
  }


  // one full replan-and-publish cycle, run from callbackReplanRequest(): makePath() -> draw path_ (magenta) and flush bv_path_ (incl. the
  // green TSP tour rays queued by visualizeGlobalPath()) -> makeTrajectory(path_) -> publishTrajectory() -> folds the cycle's duration into
  // latency_estimate_; returns true iff a trajectory was published
  bool Explorer::pathAndTrajectory()
  {
    if (!is_initialized_) {
      return false;
    }

    ros::Time t0 = ros::Time::now();

    // wall-clock timing of the whole cycle; makePath()/makeTrajectory() log their own phases indented beneath their line here
    ros::WallTime t_checkpoint = ros::WallTime::now();
    ros::WallTime t_total      = ros::WallTime::now();

    // dont generate trajectory if path generation unsuccesful of no new path needed
    bool new_path = makePath();

    if (!new_path)
    {
      return false;
    }
    ROS_INFO("[MrsExplorer]: makePath         %8.3fms", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
    t_checkpoint = ros::WallTime::now();

    for (int i=0; i+1<path_.points.size(); i++)
    {
      bv_path_->addRay(mrs_lib::geometry::Ray(Eigen::Vector3d(path_.points[i].x(),   path_.points[i].y(),   path_.points[i].z()),
                                              Eigen::Vector3d(path_.points[i+1].x(), path_.points[i+1].y(), path_.points[i+1].z())),
                                                1.0, 0.0, 1.0, 1.0);
    }
    bv_path_->publish();
    bv_path_->clearBuffers();
    ROS_INFO("[MrsExplorer]: visualize        %8.3fms", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
    t_checkpoint = ros::WallTime::now();


    auto trajectory = makeTrajectory(path_);

    if (!trajectory)
    {
      ROS_WARN("[MrsExplorer]: trajectory generation falied");
      return false;
    }
    ROS_INFO("[MrsExplorer]: makeTrajectory   %8.3fms", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
    t_checkpoint = ros::WallTime::now();

    if (!publishTrajectory(*trajectory, path_.id))
    {
      ROS_WARN("[MrsExplorer]: trajectory reference publishing failed");
      return false;
    }
    ROS_INFO("[MrsExplorer]: publish          %8.3fms", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());

    // ROS_WARN("[MrsExplorer]: trajectory replan succesfull");

    ros::Duration dt = ros::Time::now() - t0;

    // state sampled in makePath() -> trajectory published is how far ahead the next replan should start (see compensateLatency())
    if (_latency_compensation_ > 0.0)
    {
      latency_estimate_ = _latency_ema_alpha_*dt.toSec() + (1.0 - _latency_ema_alpha_)*latency_estimate_;
    }

    ROS_INFO("[MrsExplorer]: total            %8.3fms (sim %.1fms), latency estimate %.1fms", 1000 * (ros::WallTime::now() - t_total).toSec(), dt.toSec()*1000.0, latency_estimate_*1000.0);
    ROS_INFO("[MrsExplorer]: ---------------------------");
    return true;
  }

  // (re)plans path_ unconditionally (all gating happens in timerReplanner()): getCurrentStateAndPrediction() -> compensateLatency() moves the start
  // ahead along the MPC prediction -> resets goal_/current_viewpoint_ to that start -> solveTsp() for a fresh global viewpoint tour -> visualizeGlobalPath()
  // -> refineLocalTour() (re-picks the viewpoints of the nearest tour clusters) -> buildLocalPath() (path_planner sub-paths to the next
  // few tour viewpoints) -> finalizePath(); returns true iff path_ was updated
  bool Explorer::makePath()
  {
    // ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: starting path (re)plannig");

    octomap::point3d start_coord;
    mrs_msgs::MpcPredictionFullState prediction;
    octomath::Vector3 velocity;
    if (!getCurrentStateAndPrediction(start_coord, prediction, velocity))
    {
      ROS_WARN("[MrsExplorer-Replanner]: cannot get state and prediction");
      return false;
    }

    // phase timing, logged indented beneath pathAndTrajectory()'s makePath line
    ros::WallTime t_checkpoint = ros::WallTime::now();

    // the UAV keeps flying the old trajectory while this replan runs, so plan from where it will be by the time the new one takes over
    compensateLatency(prediction, start_coord, velocity);

    // ROS_WARN("[MrsExplorer]: replanning");
    goal_ = start_coord;
    current_viewpoint_ = start_coord;

    std::vector<octomap::point3d> glob_path;
    std::vector<unsigned long>    tour_ids;
    if (!solveTsp(velocity, glob_path, tour_ids))
    {
      ROS_ERROR("[MrsExplorer]: TSP solve failed");
      return false;
    }

    visualizeGlobalPath(glob_path);
    ROS_INFO("[MrsExplorer]:   tsp            %8.3fms", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
    t_checkpoint = ros::WallTime::now();

    if (_refine_enabled_)
    {
      refineLocalTour(start_coord, velocity, glob_path, tour_ids);
    }
    ROS_INFO("[MrsExplorer]:   refine         %8.3fms", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
    t_checkpoint = ros::WallTime::now();

    std::vector<octomap::point3d> path(0);
    std::vector<octomap::point3d> sub_global_path(0);
    if (!buildLocalPath(start_coord, glob_path, velocity, path, sub_global_path))
    {
      ROS_ERROR("[MrsExplorer]: local path build failed, replan local path");
      return false;
    }
    ROS_INFO("[MrsExplorer]:   local path     %8.3fms", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());

    bool succ = finalizePath(path, sub_global_path);
    if (!succ){
      ROS_ERROR("[MrsExplorer]: finalize path build failed");
      return false;
    }
    return true;
  }

   // calls tsp_solver's ~set_start_out (start = current_viewpoint_) and ~solve_out (with velocity) services to obtain a fresh global viewpoint tour
  // into glob_path (and the parallel frontier ids into tour_ids); returns false if either call fails or the tour has <= 1 point
  bool Explorer::solveTsp(const octomath::Vector3& velocity, std::vector<octomap::point3d>& glob_path, std::vector<unsigned long>& tour_ids)
  {
    // ROS_INFO("[MrsExplorer]: TSP solve start");

    tsp_solver::SetStart set_start_srv;
    set_start_srv.request.position.x = current_viewpoint_.x();
    set_start_srv.request.position.y = current_viewpoint_.y();
    set_start_srv.request.position.z = current_viewpoint_.z();
    if (!sc_tsp_set_start_.call(set_start_srv) || !set_start_srv.response.success)
    {
      ROS_WARN("[MrsExplorer]: failed to set TSP start");
      return false;
    }
    // ROS_INFO("[MrsExplorer]: TSP start set");

    tsp_solver::Solve solve_srv;
    solve_srv.request.velocity.x = velocity.x();
    solve_srv.request.velocity.y = velocity.y();
    solve_srv.request.velocity.z = velocity.z();
    if (!sc_tsp_solve_.call(solve_srv) || !solve_srv.response.success)
    {
      ROS_WARN("[MrsExplorer]: TSP tour not found");
      return false;
    }
    for (auto &p : solve_srv.response.path)
    {
      glob_path.emplace_back(p.x, p.y, p.z);
    }
    tour_ids.assign(solve_srv.response.ids.begin(), solve_srv.response.ids.end());
    // ROS_INFO("[MrsExplorer]: TSP solve end");

    if (glob_path.size() <= 1)
    {
      ROS_WARN("[MrsExplorer]: TSP tour not found");
      return false;
    }
    return true;
  }

  // local viewpoint refinement (FUEL-style, Euclidean instead of time lower bounds): takes the consecutive tour clusters after the start whose
  // tour viewpoint is within local_refinement/radius of start_coord (at most max_clusters, stopping at the first one not in the frontier cache),
  // and re-picks one viewpoint per cluster out of its top max_viewpoints (by coverage). The candidates form a layered DAG
  // start -> cluster 1 -> ... -> cluster N -> next tour point (fixed anchor), solved exactly by DP over the layers. Cost =
  // sum of Euclidean edge lengths + w_consistency * angle(velocity, first edge) - w_coverage * (coverage / layer's max coverage).
  // The tour's own viewpoint is always candidate 0 of its layer, so the result is never worse than the TSP tour under this cost.
  // Rewrites the chosen viewpoints into glob_path in place and queues the refined segment as orange rays on bv_path_
  void Explorer::refineLocalTour(const octomap::point3d& start_coord, const octomath::Vector3& velocity, std::vector<octomap::point3d>& glob_path,
                                 const std::vector<unsigned long>& tour_ids)
  {
    if (tour_ids.size() != glob_path.size())
    {
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: TSP tour ids missing, skipping local viewpoint refinement");
      return;
    }

    // same leading-point skip as buildLocalPath()
    size_t first = 0;
    while (first < glob_path.size() && start_coord.distance(glob_path[first]) < _skip_path_point_distance_)
    {
      first++;
    }

    // layers[k] = candidates of tour point first+k, the tour's own viewpoint first; coverage normalized to [0, 1] per layer
    struct Candidate_t
    {
      octomap::point3d position;
      double           coverage;
    };
    std::vector<std::vector<Candidate_t>> layers;
    {
      std::scoped_lock lock(mutex_frontier_viewpoints_);
      for (size_t i = first; i < glob_path.size() && static_cast<int>(layers.size()) < _refine_max_clusters_; i++)
      {
        if (start_coord.distance(glob_path[i]) > _refine_radius_)
        {
          break;
        }
        auto it = frontier_viewpoints_.find(tour_ids[i]);
        if (it == frontier_viewpoints_.end() || it->second.empty())
        {
          break;
        }

        std::vector<Candidate_t> layer;
        layer.push_back({glob_path[i], 0.0});
        double max_coverage = 1.0;
        const int n_vp = std::min(static_cast<int>(it->second.size()), _refine_max_viewpoints_);
        for (int v = 0; v < n_vp; v++)
        {
          const auto& vp = it->second[v];
          max_coverage = std::max(max_coverage, static_cast<double>(vp.coverage));
          if (vp.position.distance(glob_path[i]) < 1e-3)
          {
            layer[0].coverage = vp.coverage;
          } else
          {
            layer.push_back({vp.position, static_cast<double>(vp.coverage)});
          }
        }
        for (auto& c : layer)
        {
          c.coverage /= max_coverage;
        }
        layers.push_back(std::move(layer));
      }
    }

    if (layers.empty())
    {
      return;
    }

    const size_t last = first + layers.size(); // index of the anchor (first tour point after the refined segment), if any
    const bool   has_anchor = last < glob_path.size();
    const bool   use_consistency = velocity.norm() > 0.2;

    auto nodeCost = [&](size_t k, const Candidate_t& c) {
      double cost = -_refine_w_coverage_ * c.coverage;
      if (k == 0 && use_consistency)
      {
        const octomath::Vector3 dir = c.position - start_coord;
        if (dir.norm() > 1e-3)
        {
          cost += _refine_w_consistency_ * velocity.angleTo(dir);
        }
      }
      return cost;
    };

    // forward pass over the layers; cost[b] = cheapest cost of reaching candidate b of the current layer from start_coord
    std::vector<double>           cost = {0.0};
    std::vector<octomap::point3d> prev = {start_coord};
    std::vector<std::vector<int>> parents(layers.size());
    double baseline = 0.0; // cost of keeping the TSP tour's own viewpoints (candidate 0 everywhere), for logging
    for (size_t k = 0; k < layers.size(); k++)
    {
      const auto&         layer = layers[k];
      std::vector<double> next_cost(layer.size(), std::numeric_limits<double>::infinity());
      parents[k].assign(layer.size(), 0);
      for (size_t b = 0; b < layer.size(); b++)
      {
        const double node_cost = nodeCost(k, layer[b]);
        for (size_t a = 0; a < prev.size(); a++)
        {
          const double c = cost[a] + prev[a].distance(layer[b].position) + node_cost;
          if (c < next_cost[b])
          {
            next_cost[b]  = c;
            parents[k][b] = static_cast<int>(a);
          }
        }
      }
      baseline += prev[0].distance(layer[0].position) + nodeCost(k, layer[0]);

      cost = std::move(next_cost);
      prev.clear();
      for (const auto& c : layer)
      {
        prev.push_back(c.position);
      }
    }

    // close with the edge to the anchor and pick the best final candidate
    int    best      = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t b = 0; b < prev.size(); b++)
    {
      const double c = cost[b] + (has_anchor ? prev[b].distance(glob_path[last]) : 0.0);
      if (c < best_cost)
      {
        best_cost = c;
        best      = static_cast<int>(b);
      }
    }
    if (has_anchor)
    {
      baseline += prev[0].distance(glob_path[last]);
    }

    // backtrack and write the chosen viewpoints into the tour
    int changed = 0;
    for (int k = static_cast<int>(layers.size()) - 1; k >= 0; k--)
    {
      glob_path[first + k] = layers[k][best].position;
      changed += best != 0;
      best = parents[k][best];
    }

    octomap::point3d v0 = start_coord;
    for (size_t i = first; i < (has_anchor ? last + 1 : last); i++)
    {
      const octomap::point3d& v1 = glob_path[i];
      bv_path_->addRay(mrs_lib::geometry::Ray(Eigen::Vector3d(v0.x(), v0.y(), v0.z()),
                                              Eigen::Vector3d(v1.x(), v1.y(), v1.z())),
                                              1.0, 0.5, 0.0, 1.0);
      v0 = v1;
    }

    ROS_INFO("[MrsExplorer]: local refinement over %lu clusters changed %d viewpoints, cost %.2f -> %.2f", layers.size(), changed, baseline, best_cost);
  }

  // walks glob_path from start_coord, skipping tour points closer than planning/skip_path_point_distance, collecting the visited tour points into
  // sub_global_path and planning a path_planner sub-path to each (current velocity only on the first segment); sub-path points are appended to
  // path (deduplicated by skip_path_point_distance). Stops once path extends planning/max_length_after_first_viewpoint of arc length past the
  // first tour viewpoint (the overshooting edge is cut exactly there, keeps the trajectory generation request short), or at the tour's end.
  // On a planner failure returns false, except after kMaxTempPathFailures failures in a row, when it instead returns a 1 m step in a random
  // direction (randomStepPath()) to break the deadlock
  bool Explorer::buildLocalPath(const octomap::point3d& start_coord, const std::vector<octomap::point3d>& glob_path, const octomath::Vector3& velocity,
                                std::vector<octomap::point3d>& path, std::vector<octomap::point3d>& sub_global_path)
  {
    sub_global_path.assign({start_coord});
    path.assign({start_coord});

    const double max_tail_length      = _max_length_after_first_viewpoint_;
    double       tail_length          = 0.0;    // arc length of path past the first tour viewpoint
    bool         past_first_viewpoint = false;  // true once the first segment (start -> first tour viewpoint) is appended
    size_t       i                    = 0;      // index of the next candidate tour point in glob_path

    while (tail_length < max_tail_length)
    {
      // advance to the next tour point far enough from the last accepted one; stop once the tour has no further distinct point
      while (i + 1 < glob_path.size() && sub_global_path.back().distance(glob_path[i]) < _skip_path_point_distance_)
      {
        i++;
      }
      if (i + 1 >= glob_path.size() && sub_global_path.back().distance(glob_path[i]) < 1e-6)
      {
        break;
      }

      const octomath::Vector3 seg_velocity = past_first_viewpoint ? octomath::Vector3(0.0f, 0.0f, 0.0f) : velocity;
      std::vector<octomap::point3d> segment;
      if (!findSimplifiedPath(sub_global_path.back(), glob_path[i], seg_velocity, segment))
      {
        ROS_ERROR("[MrsExplorer]: temp path not found");
        if (++temp_path_fail_count_ < kMaxTempPathFailures)
        {
          return false;
        }
        ROS_ERROR("[MrsExplorer]: temp path not found %d times in a row, flying 1 m in a random direction", temp_path_fail_count_);
        temp_path_fail_count_ = 0;
        randomStepPath(start_coord, path, sub_global_path);
        return true;
      }
      sub_global_path.push_back(glob_path[i]);

      for (const auto& p : segment)
      {
        const double edge = p.distance(path.back());
        if (edge <= _skip_path_point_distance_)
        {
          continue;
        }
        if (past_first_viewpoint && tail_length + edge >= max_tail_length)
        {
          // cut the overshooting edge exactly at the cap
          path.push_back(path.back() + (p - path.back()) * static_cast<float>((max_tail_length - tail_length) / edge));
          tail_length = max_tail_length;
          break;
        }
        path.push_back(p);
        if (past_first_viewpoint)
        {
          tail_length += edge;
        }
      }
      past_first_viewpoint = true;
    }

    temp_path_fail_count_ = 0;
    return true;
  }

  // calls path_planner's ~find_simplified_path_out from start to goal (planned with the given initial velocity) and returns its points in segment;
  // false if the call or the planner fails
  bool Explorer::findSimplifiedPath(const octomap::point3d& start, const octomap::point3d& goal, const octomath::Vector3& velocity,
                                    std::vector<octomap::point3d>& segment)
  {
    path_planner::FindSimplifiedPath find_path_srv;
    find_path_srv.request.start.x = start.x();
    find_path_srv.request.start.y = start.y();
    find_path_srv.request.start.z = start.z();
    geometry_msgs::Point goal_point;
    goal_point.x = goal.x();
    goal_point.y = goal.y();
    goal_point.z = goal.z();
    find_path_srv.request.goals.push_back(goal_point);
    find_path_srv.request.velocity.x = velocity.x();
    find_path_srv.request.velocity.y = velocity.y();
    find_path_srv.request.velocity.z = velocity.z();

    if (!sc_path_planner_find_simplified_path_.call(find_path_srv) || !find_path_srv.response.success[0])
    {
      return false;
    }

    segment.clear();
    for (const auto& p : find_path_srv.response.paths[0].points)
    {
      segment.emplace_back(p.x, p.y, p.z);
    }
    return true;
  }

  // fail-safe for buildLocalPath(): sets path and sub_global_path to a 1 m step from start_coord in a uniformly random direction
  void Explorer::randomStepPath(const octomap::point3d& start_coord, std::vector<octomap::point3d>& path, std::vector<octomap::point3d>& sub_global_path)
  {
    std::normal_distribution<float> gauss(0.0f, 1.0f);
    octomath::Vector3 dir(gauss(rng_), gauss(rng_), gauss(rng_));
    if (dir.norm() < 1e-6f)
    {
      dir = octomath::Vector3(1.0f, 0.0f, 0.0f);
    }
    dir.normalize();

    const octomap::point3d random_point = start_coord + dir;
    path.assign({start_coord, random_point});
    sub_global_path.assign({start_coord, random_point});
  }

  // trims the leading (already-occupied) start point and commits path into path_ (new id); sets goal_ to the first tour viewpoint (sub_global_path[1])
  // and next_goal_ to the path's end, which nearGoal() then watches; returns false if nothing is left after trimming
  bool Explorer::finalizePath(std::vector<octomap::point3d>& path, const std::vector<octomap::point3d>& sub_global_path)
  {
    path.erase(path.begin());
    path_.points = path;
    path_.id     = next_path_id_++;

    if (path_.points.size() > 0)
    {
      goal_ = sub_global_path[1]; // path_.points.back();
      next_goal_ = path.back();
      first_path_planend_ = true;
      return true;
    }

    ROS_WARN("[MRsExplorer] no suitable frontiers");
    return false;
  }  

  // converts the given path (in the octree frame, fly_now=false) into a trajectory via the trajectory_generation_out (mrs_msgs/GetPathSrv) service;
  // nullopt on failure
  std::optional<mrs_msgs::TrajectoryReference> Explorer::makeTrajectory(const Path_t& path)
  {
    // phase timing, logged indented beneath pathAndTrajectory()'s makeTrajectory line
    ros::WallTime t_checkpoint = ros::WallTime::now();

    mrs_msgs::GetPathSrv srv_get_path;
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    srv_get_path.request.path.header.frame_id = octree_frame;
    srv_get_path.request.path.header.stamp    = ros::Time::now();
    srv_get_path.request.path.fly_now         = false;
    srv_get_path.request.path.relax_heading   = true;
    srv_get_path.request.path.use_heading     = false;

    // requested path length (sum of waypoint-to-waypoint distances), for the get_path timing print
    double path_length = 0.0;
    for (size_t i = 0; i < path.points.size(); i++)
    {
      const auto& point = path.points[i];
      mrs_msgs::Reference reference;
      reference.position.x = point.x();
      reference.position.y = point.y();
      reference.position.z = point.z();
      srv_get_path.request.path.points.push_back(reference);
      if (i > 0)
      {
        path_length += point.distance(path.points[i-1]);
      }
    }
    // ROS_INFO("[MrsExplorer]:   request        %8.3fms", 1000 * (ros::WallTime::now() - t_checkpoint).toSec());
    // t_checkpoint = ros::WallTime::now();

    bool success = sc_get_trajectory_.call(srv_get_path);
    const double get_path_ms = 1000 * (ros::WallTime::now() - t_checkpoint).toSec();
    ROS_INFO("[MrsExplorer]:   trajectory length %.2fm, %.3fms/m", path_length, path_length > 0.0 ? get_path_ms / path_length : 0.0);

    if (!success) {
      ROS_WARN("[MrsExplorer]: service call for trajectory failed");
      return std::nullopt;
    }

    if (!srv_get_path.response.success) {
      ROS_WARN("[MrsExplorer]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
      return std::nullopt;
    }

    return srv_get_path.response.trajectory;
  }

  // publishes the given trajectory (fly_now=true) via trajectory_reference_out (mrs_msgs/TrajectoryReferenceSrv), using id as the input_id
  bool Explorer::publishTrajectory(const mrs_msgs::TrajectoryReference& trajectory, int id)
  {
    mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
    srv_trajectory_reference.request.trajectory          = trajectory;
    srv_trajectory_reference.request.trajectory.fly_now   = true;
    srv_trajectory_reference.request.trajectory.input_id  = id;

    bool success = sc_trajectory_reference_.call(srv_trajectory_reference);

    if (!success) {
      ROS_WARN("[Explorer]: service call for trajectory reference failed");
      return false;
    }

    if (!srv_trajectory_reference.response.success) {
      ROS_WARN("[Explorer]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
      return false;
    }

    return true;
  }

    // returns true once the UAV is within planning/replanning_distance of goal_ (first tour viewpoint) or next_goal_ (end of path_), i.e. a replan is due;
  // before the first successful plan, goal_/next_goal_ are pinned to the current position so this returns true immediately
  bool Explorer::nearGoal(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction)
  {
    if (!first_path_planend_){
      goal_ = start_coord;
      next_goal_ = goal_;
      // ros::Duration(5.0).sleep();
    }
    geometry_msgs::Point p = prediction.position.back();
    if (start_coord.distance(goal_) > _replanning_distance_ && start_coord.distance(next_goal_) > _replanning_distance_) // octomap::point3d(p.x,p.y,p.z)
    {
      ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: not planning, too far from goal %f", start_coord.distance(octomap::point3d(p.x,p.y,p.z)));
      return false;
    }
    return true;
  }

  // returns true once the UAV has moved less than planning/stuck_distance for planning/stuck_duration seconds straight (timerReplanner() then forces
  // STATE_IDLE); tracker is (re)armed by stuck_check_reset_, which requestReplan() sets whenever a new trajectory execution actually starts
  bool Explorer::checkStuck(const octomap::point3d& start_coord)
  {
    if (stuck_check_reset_ || start_coord.distance(last_moved_position_) > _stuck_distance_)
    {
      last_moved_position_ = start_coord;
      last_moved_time_      = ros::Time::now();
      stuck_check_reset_    = false;
      return false;
    }

    return (ros::Time::now() - last_moved_time_).toSec() > _stuck_duration_;
  }

  // returns true if any position sample of the MPC prediction lies within planning/flight_free_distance of non-free space in octree_
  // (called by timerReplanner() each tick in STATE_TRAJ_EXEC to trigger a replan); start_coord is unused
  bool Explorer::checkTrajectoryCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction)
  {
    // check if current path from predition is in collision
    bool isFreeSpace = true;
    // held for the whole check: octree_ is merged into in place by callbackOctomap
    std::scoped_lock lock(mutex_octree_);
    const OcTree_t* tree = octree_.get();
    for (auto & point : prediction.position)
    {
      isFreeSpace = explorer_utils::isFreeSpace(octomap::point3d(point.x,point.y,point.z), _flight_free_distance_, *tree);
      if (!isFreeSpace)
      {
        // goal_ = start_coord;
        // ROS_WARN("[MrsExplorer]: collision detected in trajectory, replanning");
        break;
      }
    }
    return !isFreeSpace;
  }

  // gets current position and MPC full-state prediction, deriving start_coord/prediction/velocity (velocity = first prediction sample);
  // returns false (with a throttled warning) if either is unavailable; used by both timerReplanner() and makePath()
  bool Explorer::getCurrentStateAndPrediction(octomap::point3d& start_coord, mrs_msgs::MpcPredictionFullState& prediction, octomath::Vector3& velocity)
  {
    // get current position
    auto res_position = getPosition();
    if (!res_position){
      ROS_WARN_THROTTLE(1.0,"[MRsExplorer] has no reference");
      return false;
    }
    start_coord = explorer_utils::pointToOctomap(res_position.value().reference.position);

    // extract tracker predition
    auto res_prediction = getFullStatePrediction();
    if (!res_prediction){
      ROS_WARN_THROTTLE(1.0,"[MRsExplorer] has no full state prediction");
      return false;
    }

    prediction = res_prediction.value();
    velocity   = octomath::Vector3(prediction.velocity[0].x,prediction.velocity[0].y,prediction.velocity[0].z);

    return true;
  }

  // advances start_coord/velocity along the MPC prediction by latency_estimate_ seconds (measured from the prediction's first
  // stamp), stopping early at the last sample still in free space (a collision-triggered replan has a prediction running into an obstacle);
  // leaves them untouched if compensation is disabled or the prediction is malformed
  void Explorer::compensateLatency(const mrs_msgs::MpcPredictionFullState& prediction, octomap::point3d& start_coord, octomath::Vector3& velocity)
  {
    const std::size_t n = prediction.position.size();
    if (_latency_compensation_ <= 0.0 || n == 0 || prediction.stamps.size() != n || prediction.velocity.size() != n)
    {
      return;
    }

    std::scoped_lock lock(mutex_octree_);
    if (!octree_)
    {
      return;
    }

    const ros::Time t0 = prediction.stamps[0];
    for (std::size_t k = 1; k < n; k++)
    {
      const octomap::point3d p(prediction.position[k].x, prediction.position[k].y, prediction.position[k].z);
      if (!explorer_utils::isFreeSpace(p, _flight_free_distance_, *octree_))
      {
        break;
      }
      start_coord = p;
      velocity    = octomath::Vector3(prediction.velocity[k].x, prediction.velocity[k].y, prediction.velocity[k].z);
      if ((prediction.stamps[k] - t0).toSec() >= latency_estimate_)
      {
        break;
      }
    }
  }

  // tracker's full-state MPC prediction (in the octree frame), delegating to explorer_utils::getFullStatePrediction using the tracker_cmd/diagnostics subscribers
  std::optional<mrs_msgs::MpcPredictionFullState> Explorer::getFullStatePrediction()
  {
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    return explorer_utils::getFullStatePrediction(sh_control_manager_diag_, sh_tracker_cmd_, octree_frame, *transformer_, "[MrsExplorer]");
  }

  // current UAV position (in the octree frame), delegating to explorer_utils::getPosition using the tracker_cmd/diagnostics subscribers
  std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> Explorer::getPosition()
  {
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    return explorer_utils::getPosition(sh_control_manager_diag_, sh_tracker_cmd_, octree_frame, *transformer_, "[MrsExplorer]");
  }


  // octomap_in SubscribeHandler timeout (5 s); only logs a throttled warning once at least one map was received
  void Explorer::timeoutOctomap(const std::string& topic, const ros::Time&   last_msg)
  {
    if (!is_initialized_) {
      return;
    }

    if (!sh_octomap_.hasMsg()) {
      return;
    }

    ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: octomap timeout!");
  }


    /* changeState() //{ */
  // logs and sets state_; called from requestReplan() (PATH_PLANNING/TRAJ_EXEC/IDLE) and timerReplanner() (IDLE when stuck), which branches on state_
  void Explorer::changeState(const State_t new_state) {

    const State_t old_state = state_;

    ROS_INFO("[MrsExplorer]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
  }


  // deserializes an octomap_msgs/Octomap (binary or full, per msg->binary) into an OcTree_t; returns nullopt if the message decodes to an empty/null tree
  std::optional<OcTreeSharedPtr_t> Explorer::msgToMap(const octomap_msgs::OctomapConstPtr octomap)
  {
    octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*octomap);

    if (!abstract_tree) {
      ROS_WARN("[MrsExplorer]: Octomap message is empty! can not convert to OcTree");
      return {};
    }
    else {
      return { OcTreeSharedPtr_t(dynamic_cast<OcTree_t*>(abstract_tree)) };
    }
  }


      /* timeoutTrackerCmd() // */
  // tracker_cmd_in SubscribeHandler timeout (3 s); only logs a throttled warning once at least one command was received
  void Explorer::timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg) 
  {

    if (!is_initialized_) {
      return;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      return;
  }

  ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: position cmd timeouted!");

  // if (state_ != STATE_IDLE) {
  // ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: position cmd timeouted!");
  //   // ready_to_plan_ = false;
  //   changeState(STATE_IDLE);
  //   // hover();
  // }
  }


  // merges the incoming local-zone octomap into the persistent global octree_ (seeding it on the first message; guarded by mutex_octree_), the same
  // local-to-global fusion frontier_detector and path_planner use; latches the map frame onto bv_path_ once, and marks map_ready_;
  // frontier extraction happens out-of-process in frontier_detector, not here
  void Explorer::callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    ROS_INFO_THROTTLE(5.0, "[MrsExplorer]: getting octomap");

    std::optional<OcTreeSharedPtr_t> octree_local = msgToMap(msg);

    if (!octree_local) {
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: received map is empty!");
      return;
    }

    {
      std::scoped_lock lock(mutex_octree_);
      if (!octree_) {
        // seed the persistent global tree from the first local map received
        octree_ = std::move(octree_local.value());
      }
      else {
        octree_local.value()->expand();
        explorer_utils::mergeInto(*octree_local.value(), *octree_);
      }
      octree_frame_ = msg->header.frame_id;
    }

    if (!bv_map_frame_set_) {
      bv_path_->setParentFrame(msg->header.frame_id);
      bv_map_frame_set_ = true;
    }

    map_ready_  = true;

  }
  // caches every frontier's viewpoints (by id) for refineLocalTour(), records the frontier count for the end-of-exploration check, and sets tsp_ready_ on the first frontier message, which unblocks
  // timerReplanner() (tsp_solver consumes the same topic itself for the global tour)
  void Explorer::callbackFrontiers(const frontier_detector::FrontierArray::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    std::unordered_map<unsigned long, std::vector<Viewpoint_t>> frontier_viewpoints;
    frontier_viewpoints.reserve(msg->frontiers.size());
    for (const auto& f : msg->frontiers)
    {
      auto& vps = frontier_viewpoints[f.id];
      vps.reserve(f.viewpoints.size());
      for (const auto& v : f.viewpoints)
      {
        vps.push_back({octomap::point3d(v.position.x, v.position.y, v.position.z), v.coverage});
      }
    }
    {
      std::scoped_lock lock(mutex_frontier_viewpoints_);
      frontier_viewpoints_ = std::move(frontier_viewpoints);
    }

    num_frontiers_ = msg->frontiers.size();
    if (!msg->frontiers.empty())
    {
      had_frontiers_ = true;
    }

    tsp_ready_ = true;
  }


  // diagnostics_in callback; currently a no-op besides the init guard (only feeds sh_control_manager_diag_'s own freshness checks used elsewhere via hasMsg()/lastMsgTime())
  void Explorer::controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr diagnostics)
  {
    if (!is_initialized_){
        return;
    }
  }

  // queues the freshly solved global TSP tour as green debug rays on bv_path_ (published later, together with path_, by pathAndTrajectory())
  void Explorer::visualizeGlobalPath(const std::vector<octomap::point3d>& glob_path)
  {
    // ROS_ERROR("glob path size %i",glob_path.size());
    for (int i=0; i<glob_path.size()-1; i++)
    {
      // ROS_ERROR("%.1f %.1f %.1f", glob_path[i].x(), glob_path[i].y(), glob_path[i].z());
      auto v0 = glob_path[i];
      auto v1 = glob_path[i+1];
      bv_path_->addRay(mrs_lib::geometry::Ray(Eigen::Vector3d(v0.x(), v0.y(), v0.z()),
                                              Eigen::Vector3d(v1.x(), v1.y(), v1.z())),
                                              0.0, 1.0, 0.0, 1.0);
    }
  }



}
  

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(explorer::Explorer,
                       nodelet::Nodelet)

