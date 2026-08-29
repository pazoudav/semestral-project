
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_srvs/Trigger.h>

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

#include <mrs_octomap_planner/Path.h>
#include <mrs_octomap_tools/octomap_methods.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
  STATE_MAP_UPDATED,
  STATE_FRONTIERS_UPDATED,
  STATE_PRM_UPDATED,
  STATE_TSP_UPDATED,
  STATE_WAITING,
  STATE_FLYING
  
} State_t;


const std::string _state_names_[] = {"IDLE", "MAP_UPDATED", "FRONTIERS_UPDATED", "PRM_UPDATED", "TSP_UPDATED", "WAITING", "FLYING"};


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
    double _big_distance_;
    double _heading_weight_;
    double _skip_path_point_distance_;
    double _free_space_dia_;
    double _scale_points_;
    double _scale_lines_;
    double _rate_main_timer_;
    double _rate_path_timer_;

    std::vector<octomap::point3d> path_;
    int              path_id_ = 0;
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
    ros::Publisher pub_big_ocotmap_;
    mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>             sc_get_trajectory_;
    mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference_;
    mrs_lib::ServiceClientHandler<tsp_solver::SetStart>             sc_tsp_set_start_;
    mrs_lib::ServiceClientHandler<tsp_solver::Solve>                sc_tsp_solve_;
    mrs_lib::ServiceClientHandler<prm_solver::FindSimplifiedPath>   sc_prm_find_simplified_path_;

    // timeout callbacks
    void timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg);
    void timeoutOctomap(const std::string& topic,
                        const ros::Time&   last_msg);

    // subscriber callbacks
    void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
    void callbackFrontiers(const frontier_detection::FrontierArray::ConstPtr msg);
    void controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);


    // service servers
    ros::ServiceServer service_server_get_path_;
    ros::ServiceServer service_server_explore_;

    // service server callbacks
    bool callbackGetPath(mrs_octomap_planner::Path::Request&  req,
                         mrs_octomap_planner::Path::Response& res);

    bool callbackExplore(std_srvs::Trigger::Request& req,
                         std_srvs::Trigger::Response& res);

    // timers
    ros::Timer timer_main_;
    void       timerMain([[maybe_unused]] const ros::TimerEvent& evt);
    // ros::Timer timer_frontiers_;
    // void       timer_frontiers_([[maybe_unused]] const ros::TimerEvent& evt);
    ros::Timer timer_path_;
    void       timerPath([[maybe_unused]] const ros::TimerEvent& evt);
    // void       timerPath([[maybe_unused]] const ros::TimerEvent& evt);

    

    std::unique_ptr<mrs_lib::Transformer> transformer_;

    std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> getPosition();
    std::optional<mrs_msgs::MpcPredictionFullState>                   getFullStatePrediction();
    std::optional<OcTreeSharedPtr_t>                                  msgToMap(const octomap_msgs::OctomapConstPtr octomap);
    bool                                                              makePath();
    bool                                                              makeTrajectory();
    bool                                                              checkTrajectoryCollision();

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
    param_loader.loadParam("planning/big_distance",_big_distance_);
    param_loader.loadParam("planning/heading_weight",_heading_weight_);
    param_loader.loadParam("planning/skip_path_point_distance",_skip_path_point_distance_);

    param_loader.loadParam("prm/free_space_diameter",_free_space_dia_);

    param_loader.loadParam("viz/scale/points", _scale_points_);
    param_loader.loadParam("viz/scale/lines", _scale_lines_);

    param_loader.loadParam("timer_rates/main",       _rate_main_timer_);
    param_loader.loadParam("timer_rates/path",       _rate_path_timer_);


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

    service_server_get_path_ = nh_.advertiseService("get_path_in", &Explorer::callbackGetPath, this);
    service_server_explore_= nh_.advertiseService("explore_in", &Explorer::callbackExplore, this);


    pub_reference_            = nh_.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);
    pub_big_ocotmap_          = nh_.advertise<visualization_msgs::MarkerArray>("big_octomap_out", 1);

    sc_get_trajectory_            = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
    sc_trajectory_reference_      = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");
    sc_tsp_set_start_             = mrs_lib::ServiceClientHandler<tsp_solver::SetStart>(nh_, "set_start_out");
    sc_tsp_solve_                 = mrs_lib::ServiceClientHandler<tsp_solver::Solve>(nh_, "solve_out");
    sc_prm_find_simplified_path_  = mrs_lib::ServiceClientHandler<prm_solver::FindSimplifiedPath>(nh_, "find_simplified_path_out");


    timer_main_       = nh_.createTimer(ros::Rate(_rate_main_timer_), &Explorer::timerMain,      this);
    timer_path_       = nh_.createTimer(ros::Rate(_rate_path_timer_), &Explorer::timerPath,      this);


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

    // frontier sync (STATE_MAP_UPDATED -> STATE_FRONTIERS_UPDATED) now happens in callbackFrontiers()

  }


  // periodic path-update timer: replans via makePath(), publishes the new path as debug rays on bv_path_, then requests a trajectory via makeTrajectory()
  void Explorer::timerPath([[maybe_unused]] const ros::TimerEvent& evt)
  {
    if (!is_initialized_) 
    {
      return;
    }

    if (!map_ready_)
    {
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: map not ready, cannot calculate PATH");
      return;
    }  
    
    if (!tsp_ready_)
    {
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: TSP not ready, cannot calculate PATH");
      return;
    }  

    ros::Time t0 = ros::Time::now();

    // dont generate trajectory if path generation unsuccesful of no new path needed
    bool new_path = makePath();

    if (!new_path)
    {
      return;
    }

    for (int i=0; i+1<path_.size(); i++)
    {
      bv_path_->addRay(mrs_lib::geometry::Ray(Eigen::Vector3d(path_[i].x(),   path_[i].y(),   path_[i].z()),
                                              Eigen::Vector3d(path_[i+1].x(), path_[i+1].y(), path_[i+1].z())),
                                                1.0, 0.0, 1.0, 1.0);  
    } 
    bv_path_->publish();
    bv_path_->clearBuffers();


    bool success = makeTrajectory();

    if (!success)
    {
      ROS_WARN("[MrsExplorer]: trajectory generation falied");
      return;
    }

    ros::Duration dt = ros::Time::now() - t0;
    ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: PATH update %.1fms", dt.toNSec()/1000000.0);

  }


  // converts the incoming octomap message into octree_ (guarded by mutex_octree_), latches the map frame onto bv_path_ once, and marks map_ready_; frontier extraction happens out-of-process in frontier_detection, not here
  void Explorer::callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }
    
    ROS_INFO("[MrsExplorer]: getting octomap");

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
    changeState(STATE_FRONTIERS_UPDATED);
  }




// (re)plans the flight path: checks the current MPC prediction for imminent collisions (triggers an emergency brake to last_free_point_ if so), otherwise, once close enough to goal_/next_goal_,
// calls tsp_solver's ~set_start_out/~solve_out services for a fresh global viewpoint tour and then prm_solver's ~find_simplified_path_out per tour sub-segment to build path_; returns true iff path_ was updated
bool Explorer::makePath()
{
  // ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: starting path (re)plannig");

  ros::Time t0 = ros::Time::now();

  // get current position
  auto res_position = getPosition();
  if (!res_position){
    ROS_WARN_THROTTLE(1.0,"[MRsExplorer] has no reference");
    return false;
  }
  auto pos = res_position.value().reference.position;
  octomap::point3d start_coord(pos.x, pos.y, pos.z);

  // if ((start_coord -octomap::point3d(0.0,0.0,0.0)).norm()<0.01){
  //     ROS_ERROR("invalid drone position");
  //     return false;
  // }
  
  // extract tracker predition
  auto res_prediction = getFullStatePrediction();
  if (!res_prediction){
    ROS_WARN_THROTTLE(1.0,"[MRsExplorer] has no full state prediction");
    return false;
  }
  auto prediction = res_prediction.value();
  octomath::Vector3 velocity(prediction.velocity[0].x,prediction.velocity[0].y,prediction.velocity[0].z);
  
  // check if current path from predition is in collision
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
      ROS_ERROR("EMERGENCY REPLAN");
      path_ = {start_coord, last_free_point_};
      goal_ = path_.back();
      ROS_WARN("[MrsExplorer]: collision detected in trajectory, replanning");
      braking_ = true;
      return true;
    }
    else // if (isFreeSpace(octomap::point3d(point.x,point.y,point.z), _free_space_dia_, tree));
    {
      last_free_point_ = octomap::point3d(point.x,point.y,point.z);
    }
  }
  if (!first_path_planend_){
    goal_ = start_coord;
    next_goal_ = goal_;
    // ros::Duration(5.0).sleep();
  }
  // if is in collision recalculate path, otherwise check if drone is near end of the path and replan, othervise do nothing 
  geometry_msgs::Point p = prediction.position.back();
  if (start_coord.distance(goal_) > _replanning_distance_ && start_coord.distance(next_goal_) > _replanning_distance_) // octomap::point3d(p.x,p.y,p.z)
  {
    ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: not planning, too far from goal %f", start_coord.distance(octomap::point3d(p.x,p.y,p.z)));
    return false;
  }
  ROS_ERROR("replanning");
  goal_ = start_coord;

  current_viewpoint_ = start_coord;
  
  std::vector<octomap::point3d> glob_path;
  {
    ROS_ERROR("TSP solve start");

    tsp_solver::SetStart set_start_srv;
    set_start_srv.request.position.x = current_viewpoint_.x();
    set_start_srv.request.position.y = current_viewpoint_.y();
    set_start_srv.request.position.z = current_viewpoint_.z();
    if (!sc_tsp_set_start_.call(set_start_srv) || !set_start_srv.response.success)
    {
      ROS_WARN("[MrsExplorer]: failed to set TSP start");
      return false;
    }
    ROS_ERROR("TSP start set");

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
    ROS_ERROR("TSP solve end");
  }

  if (glob_path.size() <= 1)
  {
    ROS_WARN("[MrsExplorer]: TSP tour not found");
    return false;
  }
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

  // find a path to first reachable viewpoint on global path
  // add only path to first viewpoint to make into trajectory later
  std::vector<octomap::point3d> path(0);
  int i = 0;
  int j = 0;
  double path_distance = 0.0;
  // while((path.size() == 0 && i < glob_path.size()) || start_coord.distance(path.back()) < _skip_path_point_distance_)
  std::vector<octomap::point3d> sub_global_path(0);
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
      ROS_WARN("temp path not found");
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

  path.erase(path.begin());
  path_ = path;

  if (path_.size() > 0)
  {
    goal_ = sub_global_path[1]; // path_.back();
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
bool Explorer::checkTrajectoryCollision()
{
  // extract tracker predition
  auto res_prediction = getFullStatePrediction();
  if (!res_prediction){
    ROS_WARN_THROTTLE(1.0,"[MRsExplorer] has no full state prediction");
    return false;
  }
  auto prediction = res_prediction.value();
  
  // check if current path from predition is in collision
  bool isInFreeSpace = true;
  std::shared_ptr<OcTree_t> tree;
  {
    std::scoped_lock lock(mutex_octree_);
    tree = std::make_shared<OcTree_t>(*octree_);
  }
  for (auto & point : prediction.position)
  {
    isInFreeSpace = octomap_planner_utils::isFreeSpace(octomap::point3d(point.x,point.y,point.z), _flight_free_distance_, tree);
    if (!isInFreeSpace)
    {
      // goal_ = start_coord;
      ROS_WARN("[MrsExplorer]: collision detected in trajectory, replanning");
      return true;
    }
  }
}
  

// converts path_ into a trajectory via the trajectory_generation_out (mrs_msgs/GetPathSrv) service, then publishes it fly_now=true via trajectory_reference_out (mrs_msgs/TrajectoryReferenceSrv), incrementing path_id_ as the input_id
bool  Explorer::makeTrajectory()
{
  mrs_msgs::GetPathSrv srv_get_path;
  auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
  srv_get_path.request.path.header.frame_id = octree_frame;
  srv_get_path.request.path.header.stamp    = ros::Time::now();
  srv_get_path.request.path.fly_now         = false;
  srv_get_path.request.path.relax_heading   = true;
  srv_get_path.request.path.use_heading     = false;

  for (auto &point : path_)
  {
    mrs_msgs::Reference reference;
    reference.position.x = point.x();
    reference.position.y = point.y();
    reference.position.z = point.z();
    srv_get_path.request.path.points.push_back(reference);
  }

  {
    bool success = sc_get_trajectory_.call(srv_get_path);

    if (!success) {
      ROS_WARN("[Explorer]: service call for trajectory failed");
      return false;
    } else {
      if (!srv_get_path.response.success) {
        ROS_WARN("[Explorer]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
        return false;
      }
    }
  }

  auto trajectory = srv_get_path.response.trajectory;
  mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
  srv_trajectory_reference.request.trajectory         = srv_get_path.response.trajectory;
  srv_trajectory_reference.request.trajectory.fly_now = true;
  path_id_++;
  srv_trajectory_reference.request.trajectory.input_id = path_id_;
  {
    bool success = sc_trajectory_reference_.call(srv_trajectory_reference);

    if (!success) {
      ROS_WARN("[Explorer]: service call for trajectory reference failed");
      return false;
    } else {
      if (!srv_trajectory_reference.response.success) {
        ROS_WARN("[Explorer]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
        return false;
      }
    }
  }

  return true;
}

// returns the tracker's full-state MPC prediction transformed into the octree frame, or nullopt if diagnostics/tracker_cmd are stale (>2s) or the frame transform is unavailable
std::optional<mrs_msgs::MpcPredictionFullState> Explorer::getFullStatePrediction()
{
  const bool got_control_manager_diag = sh_control_manager_diag_.hasMsg() && (ros::Time::now() - sh_control_manager_diag_.lastMsgTime()).toSec() < 2.0;
  const bool got_tracker_cmd   = sh_tracker_cmd_.hasMsg() && (ros::Time::now() - sh_tracker_cmd_.lastMsgTime()).toSec() < 2.0;
  mrs_msgs::MpcPredictionFullState prediction;
  if (got_control_manager_diag && got_tracker_cmd)
  {

    prediction  = sh_tracker_cmd_.getMsg()->full_state_prediction;
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    auto ret = transformer_->getTransform(prediction.header.frame_id, octree_frame, prediction.header.stamp);

    if (!ret) {
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: could not transform position cmd to the map frame! can not check for potential collisions!");
      return {};
    }
  } 
  else
  {
    ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: could not get controller prediction");
    return {};
  }
  return prediction;
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


// diagnostics_in callback; currently a no-op besides the init guard (only feeds sh_control_manager_diag_'s own freshness checks used elsewhere via hasMsg()/lastMsgTime())
void Explorer::controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr diagnostics)
{

if (!is_initialized_){
    return;
}
      // mrs_msgs::ControlManagerDiagnosticsConstPtr diagnostics = wrp.getMsg();

      // if (uav_state_ == UAVState::FLYING && !diagnostics->tracker_status.have_goal) {

      //     changeState(UAVState::HOVERING);

      // }
}

// service handler for ~explore_in (std_srvs/Trigger); currently unimplemented — just checks init state and reports success, no actual side effects
bool Explorer::callbackExplore(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response& res)
{
  if (!is_initialized_) {
    return false;
  }
  return true;


}





// service handler for ~get_path_in (mrs_octomap_planner/Path); currently unimplemented — always returns false without populating res
bool Explorer::callbackGetPath(mrs_octomap_planner::Path::Request&  req,
                                              mrs_octomap_planner::Path::Response& res)
  {

    return false;
  }


}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_planner::Explorer,
                       nodelet::Nodelet)

