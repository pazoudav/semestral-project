#include <explorer/explorer.hpp>

namespace explorer
{

  // periodic main-loop timer (timer_rates/main); currently a no-op placeholder (all real work is driven by timerReplanner())
  void Explorer::timerMain([[maybe_unused]] const ros::TimerEvent& evt)
  {
    // ROS_ERROR_THROTTLE(1.0, "[MrsExplorer]: MAIN LOOP in '%s' STATE", _state_names_[state_].c_str());
    if (!is_initialized_ || !map_ready_) {
      return;
    }

  }

  // driver of the whole replanning loop (rate timer_rates/fast in explorer.yaml); waits for is_initialized_/map_ready_/tsp_ready_, then:
  // - once finished (sticky): does nothing; it becomes finished when frontier_detector has no frontiers left (after having had some), or the
  //   last replan found no suitable frontiers (finalizePath()), and checkStuck() reports the UAV is not moving, which sets exploration_finished_
  //   and publishes true on exploration_finished_out
  // - STATE_IDLE: requestReplan() (retried every tick, rate-limited by planning/replan_request_cooldown, until a replan succeeds)
  // - STATE_TRAJ_EXEC: requestReplan() once goalWithinLatency() or checkTrajectoryCollision() fires on the current MPC prediction,
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

    // end of exploration: frontier_detector has no frontiers left (after having had some), or none of the remaining ones is reachable
    // any further (every tour viewpoint within skip distance), the precomputed path has no entry into unknown space left, and the UAV is not moving
    if (((had_frontiers_ && num_frontiers_ == 0) || no_suitable_frontiers_) && !precomputed_has_targets_ && stuck)
    {
      ROS_INFO("[MrsExplorer]: END OF EXPLORATION, %s and UAV is not moving",
               no_suitable_frontiers_ ? "no suitable frontiers" : "no frontiers left");
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

    if (goalWithinLatency(prediction))
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

    // following the precomputed path: move the goal on with the growing map before the UAV brakes at the anchor
    if (following_precomputed_ && (ros::Time::now() - last_replan_request_time_).toSec() >= _pre_replan_period_)
    {
      requestReplan(true);
      return;
    }
  }

  // called from timerReplanner(); switches to STATE_PATH_PLANNING and makes the blocking replan_request_out call, which explorer.launch remaps onto
  // this node's own replan_request_in (-> callbackReplanRequest() -> pathAndTrajectory()); moves to STATE_TRAJ_EXEC (and re-arms checkStuck()) on
  // success or back to STATE_IDLE on failure
  // no-ops if called again within planning/replan_request_cooldown seconds of the last actual call, to avoid hammering the service (e.g. while sitting in STATE_IDLE at the fast timer's rate)
  // advance_only: periodic precomputed advance replan (makePath() -> makePathPrecomputed(advance_only)); on failure the current trajectory
  // is kept, so the state goes back to STATE_TRAJ_EXEC instead of STATE_IDLE
  void Explorer::requestReplan(bool advance_only)
  {
    const ros::Time now = ros::Time::now();
    if ((now - last_replan_request_time_).toSec() < _replan_request_cooldown_) {
      return;
    }
    last_replan_request_time_ = now;

    changeState(STATE_PATH_PLANNING);
    advance_replan_ = advance_only;
    std_srvs::Trigger replan_request_srv;
    sc_replan_request_.call(replan_request_srv);
    advance_replan_ = false;
    bool success = replan_request_srv.response.success;

    if (success) {
      stuck_check_reset_ = true;
      changeState(STATE_TRAJ_EXEC);
    }
    else {
      changeState(advance_only ? STATE_TRAJ_EXEC : STATE_IDLE);
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
    ROS_INFO("[MrsExplorer]: makePath         %8.3fms", timing_->publishSince("pathAndTrajectory/makePath", t_checkpoint));
    t_checkpoint = ros::WallTime::now();

    for (int i=0; i+1<path_.points.size(); i++)
    {
      bv_path_->addRay(mrs_lib::geometry::Ray(Eigen::Vector3d(path_.points[i].x(),   path_.points[i].y(),   path_.points[i].z()),
                                              Eigen::Vector3d(path_.points[i+1].x(), path_.points[i+1].y(), path_.points[i+1].z())),
                                                1.0, 0.0, 1.0, 1.0);
    }
    bv_path_->publish();
    bv_path_->clearBuffers();
    ROS_INFO("[MrsExplorer]: visualize        %8.3fms", timing_->publishSince("pathAndTrajectory/visualize", t_checkpoint));
    t_checkpoint = ros::WallTime::now();


    auto trajectory = makeTrajectory(path_);

    if (!trajectory)
    {
      ROS_WARN("[MrsExplorer]: trajectory generation falied");
      return false;
    }
    ROS_INFO("[MrsExplorer]: makeTrajectory   %8.3fms", timing_->publishSince("pathAndTrajectory/makeTrajectory", t_checkpoint));
    t_checkpoint = ros::WallTime::now();

    if (!publishTrajectory(*trajectory, path_.id))
    {
      ROS_WARN("[MrsExplorer]: trajectory reference publishing failed");
      return false;
    }
    ROS_INFO("[MrsExplorer]: publish          %8.3fms", timing_->publishSince("pathAndTrajectory/publish", t_checkpoint));

    // ROS_WARN("[MrsExplorer]: trajectory replan succesfull");

    ros::Duration dt = ros::Time::now() - t0;

    // state sampled in makePath() -> trajectory published is how far ahead the next replan should start (see compensateLatency())
    if (_latency_compensation_ > 0.0)
    {
      latency_estimate_ = _latency_ema_alpha_*dt.toSec() + (1.0 - _latency_ema_alpha_)*latency_estimate_;
    }

    ROS_INFO("[MrsExplorer]: total            %8.3fms (sim %.1fms), latency estimate %.1fms", timing_->publishSince("pathAndTrajectory/total", t_total), dt.toSec()*1000.0, latency_estimate_*1000.0);
    ROS_INFO("[MrsExplorer]: ---------------------------");
    return true;
  }

    /* changeState() //{ */
  // logs and sets state_; called from requestReplan() (PATH_PLANNING/TRAJ_EXEC/IDLE) and timerReplanner() (IDLE when stuck), which branches on state_
  void Explorer::changeState(const State_t new_state) {

    const State_t old_state = state_;

    ROS_INFO("[MrsExplorer]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
  }

}  // namespace explorer
