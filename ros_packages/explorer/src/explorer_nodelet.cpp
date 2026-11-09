#include <explorer/explorer.hpp>
// octomap_methods.h defines non-inline checkType<> specializations, so it may only be included in one translation unit
#include <mrs_octomap_tools/octomap_methods.h>

namespace explorer
{

  // nodelet entry point: loads params from explorer.yaml, wires up subscribers/publishers/service clients and the replan_request_in server,
  // starts timer_main_/timer_replanner_, and sets state_ to STATE_IDLE
  void Explorer::onInit()
  {
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
    timing_ = std::make_unique<explorer_utils::TimingPublisher>(nh_);

    ros::Time::waitForValid();

    ROS_INFO("[MrsExplorer]: initializing");

    mrs_lib::ParamLoader param_loader(nh_, "MrsExplorer");

    param_loader.loadParam("uav_name", _uav_name_);

    // param_loader.loadParam("planning/replanning_distance",_replanning_distance_);
    param_loader.loadParam("planning/flight_free_distance",_flight_free_distance_);
    param_loader.loadParam("planning/skip_path_point_distance",_skip_path_point_distance_);
    param_loader.loadParam("planning/max_length_after_first_viewpoint", _max_length_after_first_viewpoint_);
    param_loader.loadParam("planning/replan_request_cooldown",_replan_request_cooldown_);
    param_loader.loadParam("planning/stuck_distance",_stuck_distance_);
    param_loader.loadParam("planning/stuck_duration",_stuck_duration_);
    param_loader.loadParam("planning/goal_reached_tolerance", _goal_reached_tolerance_);
    param_loader.loadParam("planning/latency_compensation",_latency_compensation_);
    param_loader.loadParam("planning/latency_ema_alpha",_latency_ema_alpha_);

    param_loader.loadParam("local_refinement/enabled",        _refine_enabled_);
    param_loader.loadParam("local_refinement/radius",         _refine_radius_);
    param_loader.loadParam("local_refinement/max_clusters",   _refine_max_clusters_);
    param_loader.loadParam("local_refinement/max_viewpoints", _refine_max_viewpoints_);
    param_loader.loadParam("local_refinement/w_consistency",  _refine_w_consistency_);
    param_loader.loadParam("local_refinement/w_coverage",     _refine_w_coverage_);

    param_loader.loadParam("precomputed/enabled",            _pre_enabled_);
    param_loader.loadParam("precomputed/sample_step",        _pre_sample_step_);
    param_loader.loadParam("precomputed/clearance",          _pre_clearance_);
    param_loader.loadParam("precomputed/reach_tolerance",    _pre_reach_tolerance_);
    param_loader.loadParam("precomputed/min_path_length",    _pre_min_path_length_);
    param_loader.loadParam("precomputed/failure_backoff",    _pre_failure_backoff_);
    param_loader.loadParam("precomputed/replan_period",      _pre_replan_period_);
    param_loader.loadParam("precomputed/join_distance",      _pre_join_distance_);
    param_loader.loadParam("precomputed/follow_clearance",   _pre_follow_clearance_);
    param_loader.loadParam("precomputed/follow_horizon",     _pre_follow_horizon_);
    param_loader.loadParam("precomputed/max_known_gap",      _pre_max_known_gap_);
    param_loader.loadParam("precomputed/off_path_distance",  _pre_off_path_distance_);
    param_loader.loadParam("precomputed/max_entry_attempts", _pre_max_entry_attempts_);
    param_loader.loadParam("precomputed/max_no_progress",    _pre_max_no_progress_);
    param_loader.loadParam("precomputed/done_duration",      _pre_done_duration_);

    // world_info.yaml (loaded by explorer.launch from the world_info_file_config arg)
    param_loader.loadParam("zone_x", _flight_zone_width_x_);
    param_loader.loadParam("zone_y", _flight_zone_width_y_);
    param_loader.loadParam("zone_z", _flight_zone_height_);
    param_loader.loadParam("zone/global/floor", _flight_zone_floor_);

    param_loader.loadParam("viz/scale/points", _scale_points_);
    param_loader.loadParam("viz/scale/lines", _scale_lines_);

    param_loader.loadParam("timer_rates/main",       _rate_main_timer_);
    param_loader.loadParam("timer_rates/path",       _rate_path_timer_);
    param_loader.loadParam("timer_rates/fast",       _rate_fast_timer_);


    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[MrsExplorer]: Could not load all parameters");
      ros::shutdown();
    }

    // same construction as path_planner's flight_zone_
    flight_zone_ = {.min=octomap::point3d(-_flight_zone_width_x_/2.0, -_flight_zone_width_y_/2.0, _flight_zone_floor_),
                    .max=octomap::point3d( _flight_zone_width_x_/2.0,  _flight_zone_width_y_/2.0, _flight_zone_floor_+_flight_zone_height_)};

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
    sh_precomputed_path_      = mrs_lib::SubscribeHandler<skeleton_estimator::PrecomputedPath>(shopts, "precomputed_path_in", &Explorer::callbackPrecomputedPath, this);

    pub_reference_            = nh_.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);
    pub_exploration_finished_ = nh_.advertise<std_msgs::Bool>("exploration_finished_out", 1, true);

    sc_get_trajectory_            = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
    sc_trajectory_reference_      = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");
    sc_tsp_set_start_             = mrs_lib::ServiceClientHandler<tsp_solver::SetStart>(nh_, "set_start_out");
    sc_tsp_solve_                 = mrs_lib::ServiceClientHandler<tsp_solver::Solve>(nh_, "solve_out");
    sc_path_planner_find_simplified_path_ = mrs_lib::ServiceClientHandler<path_planner::FindSimplifiedPath>(nh_, "find_simplified_path_out");
    sc_replan_request_            = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "replan_request_out");
    sc_tsp_set_guiding_           = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "set_tsp_guiding_out");

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
    // first_path_planend_ = false;
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

}  // namespace explorer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(explorer::Explorer,
                       nodelet::Nodelet)
