
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

#include <octomap_planner_utils/utils.hpp>
#include <frontier_detection/FrontierArray.h>
#include <tsp_solver/SetStart.h>
#include <tsp_solver/Solve.h>
#include <prm_solver/FindSimplifiedPath.h>

typedef enum
{
  STATE_IDLE,
  STATE_PATH_PLANNING,
  STATE_TRAJ_EXEC,
  STATE_COLLISION,
} State_t;


const std::string _state_names_[] = {"IDLE", "STATE_PATH_PLANNING", "STATE_TRAJ_EXEC", "STATE_COLLISION", "TSP_UPDATED", "WAITING", "FLYING"};


namespace mrs_octomap_planner
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
    double _free_space_dia_;
    double _scale_points_;
    double _scale_lines_;
    double _rate_main_timer_;
    double _rate_path_timer_;
    double _rate_fast_timer_;
    double _replan_request_cooldown_;
    double _stuck_distance_;
    double _stuck_duration_;

    struct Path_t
    {
      int                            id = 0;
      std::vector<octomap::point3d> points;
    };

    Path_t            path_;
    int               next_path_id_ = 0;
    octomap::point3d current_viewpoint_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_viewpoint_candidates_;


    std::mutex                                mutex_octree_;
    std::shared_ptr<OcTree_t>                 octree_ = nullptr;
    std::string                               octree_frame_;
    std::shared_ptr<mrs_lib::BatchVisualizer> bv_path_;

    std::atomic<State_t> state_;
    void                 changeState(const State_t new_state);

    std::atomic<bool> map_update_;
    std::atomic<bool> map_ready_;
    std::atomic<bool> tsp_ready_;
    bool              first_path_planend_;
    bool              bv_map_frame_set_;
    bool              braking_;
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
    mrs_lib::SubscribeHandler<frontier_detection::FrontierArray> sh_frontiers_;

    // publishers
    ros::Publisher pub_reference_;
    mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>             sc_get_trajectory_;
    mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference_;
    mrs_lib::ServiceClientHandler<tsp_solver::SetStart>             sc_tsp_set_start_;
    mrs_lib::ServiceClientHandler<tsp_solver::Solve>                sc_tsp_solve_;
    mrs_lib::ServiceClientHandler<prm_solver::FindSimplifiedPath>   sc_prm_find_simplified_path_;
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
    void callbackFrontiers(const frontier_detection::FrontierArray::ConstPtr msg);
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
    bool  checkImminentCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction);
    bool  nearGoal(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction);
    bool  checkStuck(const octomap::point3d& start_coord);
    bool  solveTsp(const octomath::Vector3& velocity, std::vector<octomap::point3d>& glob_path);
    void  visualizeGlobalPath(const std::vector<octomap::point3d>& glob_path);
    bool  buildLocalPath(const octomap::point3d& start_coord, const std::vector<octomap::point3d>& glob_path, const octomath::Vector3& velocity,
                          std::vector<octomap::point3d>& path, std::vector<octomap::point3d>& sub_global_path);
    bool  finalizePath(std::vector<octomap::point3d>& path, const std::vector<octomap::point3d>& sub_global_path, const ros::Time& t0);
    std::optional<mrs_msgs::TrajectoryReference>                     makeTrajectory(const Path_t& path);
    bool                                                              publishTrajectory(const mrs_msgs::TrajectoryReference& trajectory, int id);
    bool                                                              checkTrajectoryCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction);
    void                                                              requestReplan();
    bool pathAndTrajectory();

  };

  // nodelet entry point: loads params from explorer.yaml, wires up subscribers/publishers/service clients, starts timer_main_/timer_path_, and sets state_ to STATE_IDLE
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
    param_loader.loadParam("planning/replan_request_cooldown",_replan_request_cooldown_);
    param_loader.loadParam("planning/stuck_distance",_stuck_distance_);
    param_loader.loadParam("planning/stuck_duration",_stuck_duration_);

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
    sh_frontiers_             = mrs_lib::SubscribeHandler<frontier_detection::FrontierArray>(shopts, "frontiers_in", &Explorer::callbackFrontiers, this);

    pub_reference_            = nh_.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);

    sc_get_trajectory_            = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
    sc_trajectory_reference_      = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");
    sc_tsp_set_start_             = mrs_lib::ServiceClientHandler<tsp_solver::SetStart>(nh_, "set_start_out");
    sc_tsp_solve_                 = mrs_lib::ServiceClientHandler<tsp_solver::Solve>(nh_, "solve_out");
    sc_prm_find_simplified_path_  = mrs_lib::ServiceClientHandler<prm_solver::FindSimplifiedPath>(nh_, "find_simplified_path_out");
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

    is_initialized_ = true;
    bv_map_frame_set_ = false;
    state_ = STATE_IDLE;

    ROS_INFO("[MrsExplorer]: initialized");
  }


  // periodic main-loop timer; currently a no-op placeholder once the map is ready (all real work happens in callbackFrontiers()/timerPath())
  void Explorer::timerMain([[maybe_unused]] const ros::TimerEvent& evt)
  {
    // ROS_ERROR_THROTTLE(1.0, "[MrsExplorer]: MAIN LOOP in '%s' STATE", _state_names_[state_].c_str());
    if (!is_initialized_ || !map_ready_) {
      return;
    }

  }


  // periodic fast timer (rate set by timer_rates/fast in explorer.yaml); currently a no-op placeholder
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
    

    if (state_ == STATE_IDLE)
    {
      ROS_WARN_THROTTLE(0.5, "[MrsExplorer]: idle state, start planning");
      requestReplan();
      return;
    }

    if (state_ != STATE_TRAJ_EXEC)
    {
      return;
    }

    octomap::point3d start_coord;
    mrs_msgs::MpcPredictionFullState prediction;
    octomath::Vector3 velocity;

    if (!getCurrentStateAndPrediction(start_coord, prediction, velocity))
    {
      ROS_WARN_THROTTLE(0.5, "[MrsExplorer]: cannot get state and prediction");
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
    if (checkStuck(start_coord))
    {
      ROS_ERROR("[MrsExplorer]: UAV stuck (moved < %.2f m in %.2f s) -> forcing IDLE", _stuck_distance_, _stuck_duration_);
      changeState(STATE_IDLE);
      return;
    }
  }


bool Explorer::pathAndTrajectory()
  {
    if (!is_initialized_) {
      return false;
    }

    ros::Time t0 = ros::Time::now();

    // dont generate trajectory if path generation unsuccesful of no new path needed
    bool new_path = makePath();

    if (!new_path)
    {
      return false;
    }

    for (int i=0; i+1<path_.points.size(); i++)
    {
      bv_path_->addRay(mrs_lib::geometry::Ray(Eigen::Vector3d(path_.points[i].x(),   path_.points[i].y(),   path_.points[i].z()),
                                              Eigen::Vector3d(path_.points[i+1].x(), path_.points[i+1].y(), path_.points[i+1].z())),
                                                1.0, 0.0, 1.0, 1.0);
    }
    bv_path_->publish();
    bv_path_->clearBuffers();


    auto trajectory = makeTrajectory(path_);

    if (!trajectory)
    {
      ROS_WARN("[MrsExplorer]: trajectory generation falied");
      return false;
    }

    if (!publishTrajectory(*trajectory, path_.id))
    {
      ROS_WARN("[MrsExplorer]: trajectory reference publishing failed");
      return false;
    }

    // ROS_WARN("[MrsExplorer]: trajectory replan succesfull");

    ros::Duration dt = ros::Time::now() - t0;
    ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: PATH update %.1fms", dt.toNSec()/1000000.0);
    return true;
  }


  // converts the incoming octomap message into octree_ (guarded by mutex_octree_), latches the map frame onto bv_path_ once, and marks map_ready_; frontier extraction happens out-of-process in frontier_detection, not here
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
    mrs_lib::set_mutexed(mutex_octree_, octree_local.value(), octree_);
    mrs_lib::set_mutexed(mutex_octree_, msg->header.frame_id, octree_frame_);
    

    if (!bv_map_frame_set_) {
      bv_path_->setParentFrame(msg->header.frame_id);
      bv_map_frame_set_ = true;
    }

    map_ready_  = true;

  }


  // signals that new frontiers are available, so a TSP re-solve is warranted
  // (PRM roadmap updates from these frontiers now happen internally in the prm_solver nodelet)
  void Explorer::callbackFrontiers(const frontier_detection::FrontierArray::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    tsp_ready_ = true;
  }



// (re)plans the flight path: checks the current MPC prediction for imminent collisions (triggers an emergency brake to last_free_point_ if so), otherwise, once close enough to goal_/next_goal_,
// calls tsp_solver's ~set_start_out/~solve_out services for a fresh global viewpoint tour and then prm_solver's ~find_simplified_path_out per tour sub-segment to build path_; returns true iff path_ was updated
bool Explorer::makePath()
{
  // ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: starting path (re)plannig");

  ros::Time t0 = ros::Time::now();

  octomap::point3d start_coord;
  mrs_msgs::MpcPredictionFullState prediction;
  octomath::Vector3 velocity;
  if (!getCurrentStateAndPrediction(start_coord, prediction, velocity))
  {
    ROS_WARN("[MrsExplorer-Replanner]: cannot get state and prediction");
    return false;
  }

  // ROS_WARN("[MrsExplorer]: replanning");
  goal_ = start_coord;

  current_viewpoint_ = start_coord;

  std::vector<octomap::point3d> glob_path;
  if (!solveTsp(velocity, glob_path))
  {
    ROS_ERROR("[MrsExplorer]: TSP solve failed");
    return false;
  }

  visualizeGlobalPath(glob_path);

  std::vector<octomap::point3d> path(0);
  std::vector<octomap::point3d> sub_global_path(0);
  if (!buildLocalPath(start_coord, glob_path, velocity, path, sub_global_path))
  {
    ROS_ERROR("[MrsExplorer]: local path build failed, replan local path");
    return false;
  }

  bool succ = finalizePath(path, sub_global_path, t0);
  if (!succ){
    ROS_ERROR("[MrsExplorer]: finalize path build failed");
    return false;
  }
  return true;
}


// gets current position and MPC full-state prediction, deriving start_coord/prediction/velocity; returns false (with a throttled warning) if either is unavailable
bool Explorer::getCurrentStateAndPrediction(octomap::point3d& start_coord, mrs_msgs::MpcPredictionFullState& prediction, octomath::Vector3& velocity)
{
  // get current position
  auto res_position = getPosition();
  if (!res_position){
    ROS_WARN_THROTTLE(1.0,"[MRsExplorer] has no reference");
    return false;
  }
  start_coord = octomap_planner_utils::pointToOctomap(res_position.value().reference.position);

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

// checks if current path from prediction is in collision; on collision, sets an emergency brake path to last_free_point_ and returns true
bool Explorer::checkImminentCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction)
{
  bool isInFreeSpace = true;
  std::shared_ptr<OcTree_t> tree;
  {
    std::scoped_lock lock(mutex_octree_);
    tree = std::make_shared<OcTree_t>(*octree_);
  }
  if (octomap_planner_utils::isFreeSpace(start_coord, _free_space_dia_, tree)){
    last_free_point_ = start_coord;
  }
  for (auto & point : prediction.position)
  {
    isInFreeSpace = octomap_planner_utils::isFreeSpace(octomap::point3d(point.x,point.y,point.z), _flight_free_distance_, tree);
    if (!isInFreeSpace)
    {
      ROS_ERROR("[MrsExplorer]: EMERGENCY REPLAN");
      path_.points = {start_coord, last_free_point_};
      path_.id     = next_path_id_++;
      goal_ = path_.points.back();
      ROS_WARN("[MrsExplorer]: collision detected in trajectory, replanning");
      braking_ = true;
      return true;
    }
    else // if (isFreeSpace(octomap::point3d(point.x,point.y,point.z), _free_space_dia_, tree));
    {
      last_free_point_ = octomap::point3d(point.x,point.y,point.z);
    }
  }
  return false;
}

// if is in collision recalculate path, otherwise check if drone is near end of the path and replan, othervise do nothing
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

// returns true once the UAV has moved less than planning/stuck_distance for planning/stuck_duration seconds straight; tracker is (re)armed by
// stuck_check_reset_, which requestReplan() sets whenever a new trajectory execution actually starts
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

// calls tsp_solver's ~set_start_out/~solve_out services to obtain a fresh global viewpoint tour into glob_path
bool Explorer::solveTsp(const octomath::Vector3& velocity, std::vector<octomap::point3d>& glob_path)
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
  // ROS_INFO("[MrsExplorer]: TSP solve end");

  if (glob_path.size() <= 1)
  {
    ROS_WARN("[MrsExplorer]: TSP tour not found");
    return false;
  }
  return true;
}

// draws the freshly solved global TSP tour as debug rays on bv_path_
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

// finds a path to first reachable viewpoint on global path, calling prm_solver's ~find_simplified_path_out per tour sub-segment;
// adds only the path to that first viewpoint to make into a trajectory later
bool Explorer::buildLocalPath(const octomap::point3d& start_coord, const std::vector<octomap::point3d>& glob_path, const octomath::Vector3& velocity,
                               std::vector<octomap::point3d>& path, std::vector<octomap::point3d>& sub_global_path)
{
  int i = 0;
  int j = 0;
  double path_distance = 0.0;
  // while((path.size() == 0 && i < glob_path.size()) || start_coord.distance(path.back()) < _skip_path_point_distance_)
  sub_global_path.push_back(start_coord);
  path.push_back(start_coord);

  while(sub_global_path.size() <= 3 || path_distance < 4.0)
  {
    while (sub_global_path.back().distance(glob_path[i]) < _skip_path_point_distance_)
    {
      i++;
    }
    path_distance += path_distance + sub_global_path.back().distance(glob_path[i]);
    sub_global_path.push_back(glob_path[i]);

    prm_solver::FindSimplifiedPath find_path_srv;
    find_path_srv.request.start.x = sub_global_path[j].x();
    find_path_srv.request.start.y = sub_global_path[j].y();
    find_path_srv.request.start.z = sub_global_path[j].z();
    find_path_srv.request.goal.x  = sub_global_path[j+1].x();
    find_path_srv.request.goal.y  = sub_global_path[j+1].y();
    find_path_srv.request.goal.z  = sub_global_path[j+1].z();
    octomath::Vector3 seg_velocity = j == 0 ? velocity : velocity*0.0;
    find_path_srv.request.velocity.x = seg_velocity.x();
    find_path_srv.request.velocity.y = seg_velocity.y();
    find_path_srv.request.velocity.z = seg_velocity.z();
    if (!sc_prm_find_simplified_path_.call(find_path_srv) || !find_path_srv.response.success)
    {
      ROS_ERROR("[MrsExplorer]: temp path not found");
      return false;
    }
    std::vector<octomap::point3d> temp_path;
    for (auto &p : find_path_srv.response.path)
    {
      temp_path.emplace_back(p.x, p.y, p.z);
    }

    for (auto &p : temp_path)
    {
      if (p.distance(path.back()) > _skip_path_point_distance_)
      {
        path.push_back(p);
      }
    }
    j++;
  }
  return true;
}

// trims the leading (already-occupied) start point, commits path into path_/goal_/next_goal_, and logs replanning time
bool Explorer::finalizePath(std::vector<octomap::point3d>& path, const std::vector<octomap::point3d>& sub_global_path, const ros::Time& t0)
{
  path.erase(path.begin());
  path_.points = path;
  path_.id     = next_path_id_++;

  if (path_.points.size() > 0)
  {
    goal_ = sub_global_path[1]; // path_.points.back();
    next_goal_ = path.back();
    first_path_planend_ = true;
    ros::Duration dt = ros::Time::now() - t0;
    ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: time to replan %.1fms", dt.toNSec()/1000000.0);
    return true;
  }

  ROS_WARN("[MRsExplorer] no suitable frontiers");
  return false;
}


// checks the current MPC prediction against octree_ for imminent collisions; dead code, never called (equivalent logic is inlined directly in makePath())
bool Explorer::checkTrajectoryCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction)
{
  // check if current path from predition is in collision
  bool isFreeSpace = true;
  std::shared_ptr<OcTree_t> tree;
  {
    std::scoped_lock lock(mutex_octree_);
    tree = std::make_shared<OcTree_t>(*octree_);
  }
  for (auto & point : prediction.position)
  {
    isFreeSpace = octomap_planner_utils::isFreeSpace(octomap::point3d(point.x,point.y,point.z), _flight_free_distance_, tree);
    if (!isFreeSpace)
    {
      // goal_ = start_coord;
      // ROS_WARN("[MrsExplorer]: collision detected in trajectory, replanning");
      break;
    }
  }
  return !isFreeSpace;
}
  

// converts the given path into a trajectory via the trajectory_generation_out (mrs_msgs/GetPathSrv) service
std::optional<mrs_msgs::TrajectoryReference> Explorer::makeTrajectory(const Path_t& path)
{
  mrs_msgs::GetPathSrv srv_get_path;
  auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
  srv_get_path.request.path.header.frame_id = octree_frame;
  srv_get_path.request.path.header.stamp    = ros::Time::now();
  srv_get_path.request.path.fly_now         = false;
  srv_get_path.request.path.relax_heading   = true;
  srv_get_path.request.path.use_heading     = false;

  for (auto &point : path.points)
  {
    mrs_msgs::Reference reference;
    reference.position.x = point.x();
    reference.position.y = point.y();
    reference.position.z = point.z();
    srv_get_path.request.path.points.push_back(reference);
  }

  bool success = sc_get_trajectory_.call(srv_get_path);

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


// tracker's full-state MPC prediction (in the octree frame), delegating to octomap_planner_utils::getFullStatePrediction using the tracker_cmd/diagnostics subscribers
std::optional<mrs_msgs::MpcPredictionFullState> Explorer::getFullStatePrediction()
{
  auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
  return octomap_planner_utils::getFullStatePrediction(sh_control_manager_diag_, sh_tracker_cmd_, octree_frame, *transformer_, "[MrsExplorer]");
}

// current UAV position (in the octree frame), delegating to octomap_planner_utils::getPosition using the tracker_cmd/diagnostics subscribers
std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> Explorer::getPosition()
{
  auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
  return octomap_planner_utils::getPosition(sh_control_manager_diag_, sh_tracker_cmd_, octree_frame, *transformer_, "[MrsExplorer]");
}


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
// logs and sets state_; only ever called with STATE_FRONTIERS_UPDATED (from callbackFrontiers()) — state_ is otherwise not read/branched on anywhere, so this is effectively just a diagnostic label
void Explorer::changeState(const State_t new_state) {

  const State_t old_state = state_;

  ROS_INFO("[MrsExplorer]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

  state_ = new_state;
}

// switches to STATE_PATH_PLANNING, calls the replan_request_out service, and moves to STATE_TRAJ_EXEC on success or back to STATE_IDLE on failure
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

// deserializes an octomap_msgs/Octomap (binary or full, per msg->binary) into an OcTree_t; returns nullopt if the message decodes to an empty/null tree
std::optional<OcTreeSharedPtr_t> Explorer::msgToMap(const octomap_msgs::OctomapConstPtr octomap)
{
  octomap::AbstractOcTree* abstract_tree;

  if (octomap->binary) {
    abstract_tree = octomap_msgs::binaryMsgToMap(*octomap);
  }
  else {
    abstract_tree = octomap_msgs::fullMsgToMap(*octomap);
  }

  if (!abstract_tree) {
    ROS_WARN("[MrsExplorer]: Octomap message is empty! can not convert to OcTree");
    return {};
  }
  else {
    return { OcTreeSharedPtr_t(dynamic_cast<OcTree_t*>(abstract_tree)) };
  }
}


    /* timeoutTrackerCmd() // */
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
// ROS_WARN_THROTTLE(1.0, "[MrsOctomapPlanner]: position cmd timeouted!");
//   // ready_to_plan_ = false;
//   changeState(STATE_IDLE);
//   // hover();
// }
}


// replan_request_in callback; only acknowledges the request for now, no replanning logic is triggered from here yet
bool Explorer::callbackReplanRequest([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("[MrsExplorer]: replan request received");

  bool success = pathAndTrajectory();

  res.success = success;
  res.message = "acknowledged";
  ROS_WARN("[MrsExplorer]: replan request processed: %s", success ? "SUCCESS" : "FAIL");
  return true;
}


// diagnostics_in callback; currently a no-op besides the init guard (only feeds sh_control_manager_diag_'s own freshness checks used elsewhere via hasMsg()/lastMsgTime())
void Explorer::controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr diagnostics)
{
  if (!is_initialized_){
      return;
  }
}


}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_planner::Explorer,
                       nodelet::Nodelet)

