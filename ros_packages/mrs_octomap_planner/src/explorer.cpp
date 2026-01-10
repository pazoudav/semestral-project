
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

#include <unordered_set>
#include <vector>
#include <queue>
#include <iostream>
#include <memory>
#include <bits/stdc++.h>

#include "frontier_manager.hpp"
#include "utils.hpp"
#include "prm.hpp"
#include "tsp_solver.hpp"

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
    double _flight_zone_width_;
    double _flight_zone_height_;
    double _flight_zone_floor_;
    double _local_zone_width_;
    double _local_zone_height_;
    double _replanning_distance_;
    double _flight_free_distance_;
    double _big_distance_;
    double _heading_weight_;
    double _skip_path_point_distance_;
    int    _max_tsp_duration_;
    double _free_space_dia_;
    double _ovelap_coefficient_;
    double _resample_factor_;
    double _max_cost_;
    int    _node_max_age_;
    int    _max_neighbors_;
    double _min_neighbor_distance_;
    double _max_neighbor_distance_;
    int    _init_matrix_size_;
    int    _frontier_min_size_;
    int    _min_eigen_;
    double _size_decrease_ratio_;
    int    _sample_attemps_;
    int    _min_viewpoint_cnt_;
    int    _max_viewpoint_cnt_;
    int    _min_viewpoint_coverage_;
    double _viewpoint_sample_radius_;
    double _viewpoint_sample_height_;
    double _viewpoint_max_distance_;
    double _viewpoint_max_angle_;
    double _viewpoint_resample_probability_;
    double _scale_points_;
    double _scale_lines_;
    double _rate_main_timer_;
    double _rate_PRM_timer_;
    double _rate_path_timer_;

    AABB flight_zone_;
    AABB local_zone_;
    std::vector<octomap::point3d> path_;
    int              path_id_ = 0;
    TSPsolver tsp_solver_;
    octomap::point3d current_viewpoint_;

    bool bv_map_frame_set_;

    std::mutex                                mutex_octree_;
    std::mutex                                mutex_frontiers_;
    std::mutex                                mutex_local_zone_;
    std::mutex                                mutex_PRM_;
    std::mutex                                mutex_TSP_;
    std::shared_ptr<OcTree_t>                 octree_ = nullptr;
    std::string                               octree_frame_;
    std::shared_ptr<mrs_lib::BatchVisualizer> bv_frontiers_;
    std::shared_ptr<mrs_lib::BatchVisualizer> bv_path_;
    std::shared_ptr<mrs_lib::BatchVisualizer> bv_prm_;

    std::atomic<State_t> state_;
    void                 changeState(const State_t new_state);

    std::atomic<bool> map_update_;
    std::atomic<bool> map_ready_;
    std::atomic<bool> tsp_ready_;
    bool              first_path_planend_;
    // std::atomic<bool> valid_path_;
    octomap::point3d position_before_map_update_;
    octomap::point3d goal_;

    // subscribers
    mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand> sh_tracker_cmd_;
    mrs_lib::SubscribeHandler<octomap_msgs::Octomap>    sh_octomap_;
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

    // publishers
    ros::Publisher pub_reference_;
    mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>             sc_get_trajectory_;
    mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference_;

    // managers
    std::unique_ptr<FrontierManager>  frontier_manager_;
    std::unique_ptr<PRM>              prm_manager_;

    // timeout callbacks
    void timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg);
    void timeoutOctomap(const std::string& topic,
                        const ros::Time&   last_msg);

    // subscriber callbacks
    void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
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
    ros::Timer timer_PRM_update_;
    void       timerPRMupdate([[maybe_unused]] const ros::TimerEvent& evt);
    ros::Timer timer_path_;
    void       timerPath([[maybe_unused]] const ros::TimerEvent& evt);

    

    std::unique_ptr<mrs_lib::Transformer> transformer_;

    std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> getPosition();
    std::optional<OcTreeSharedPtr_t>                                  msgToMap(const octomap_msgs::OctomapConstPtr octomap);
    bool                                                              getFrontiers();
    bool                                                              makePath();
    bool                                                              makeTrajectory();

    

  };

  void Explorer::onInit()
  {
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ros::Time::waitForValid();

    ROS_INFO("[MrsExplorer]: initializing");

    mrs_lib::ParamLoader param_loader(nh_, "MrsExplorer");

    param_loader.loadParam("uav_name", _uav_name_);

    param_loader.loadParam("zone/global/width",_flight_zone_width_);
    param_loader.loadParam("zone/global/height",_flight_zone_height_);
    param_loader.loadParam("zone/global/floor",_flight_zone_floor_);
    param_loader.loadParam("zone/local/width",_local_zone_width_);
    param_loader.loadParam("zone/local/height",_local_zone_height_);

    param_loader.loadParam("planning/replanning_distance",_replanning_distance_); 
    param_loader.loadParam("planning/flight_free_distance",_flight_free_distance_);  
    param_loader.loadParam("planning/big_distance",_big_distance_);
    param_loader.loadParam("planning/heading_weight",_heading_weight_);
    param_loader.loadParam("planning/skip_path_point_distance",_skip_path_point_distance_);

    param_loader.loadParam("tsp/max_duration",_max_tsp_duration_);

    param_loader.loadParam("prm/free_space_diameter",_free_space_dia_);
    param_loader.loadParam("prm/overlap_coefficient",_ovelap_coefficient_);
    param_loader.loadParam("prm/resample_factor",_resample_factor_);
    param_loader.loadParam("prm/max_cost",_max_cost_);
    param_loader.loadParam("prm/node_max_age",_node_max_age_);
    param_loader.loadParam("prm/max_neighbors",_max_neighbors_);
    param_loader.loadParam("prm/min_neighbor_distance",_min_neighbor_distance_);
    param_loader.loadParam("prm/max_neighbor_distance",_max_neighbor_distance_);
    param_loader.loadParam("prm/init_matrix_size",_init_matrix_size_);


    param_loader.loadParam("frontiers/min_size",_frontier_min_size_);
    param_loader.loadParam("frontiers/min_svd_eigen_value",_min_eigen_);
    param_loader.loadParam("frontiers/size_decrease_ratio",_size_decrease_ratio_);

    param_loader.loadParam("frontiers/viewpoint/sample_attempts",_sample_attemps_);
    param_loader.loadParam("frontiers/viewpoint/min_count",_min_viewpoint_cnt_);
    param_loader.loadParam("frontiers/viewpoint/max_count",_max_viewpoint_cnt_);
    param_loader.loadParam("frontiers/viewpoint/min_coverage",_min_viewpoint_coverage_);
    param_loader.loadParam("frontiers/viewpoint/sample_radius",_viewpoint_sample_radius_);
    param_loader.loadParam("frontiers/viewpoint/sample_height",_viewpoint_sample_height_);
    param_loader.loadParam("frontiers/viewpoint/max_distance",_viewpoint_max_distance_);
    param_loader.loadParam("frontiers/viewpoint/max_angle",_viewpoint_max_angle_);
    param_loader.loadParam("frontiers/viewpoint/resample_probability",_viewpoint_resample_probability_);

    param_loader.loadParam("viz/scale/points", _scale_points_);
    param_loader.loadParam("viz/scale/lines", _scale_lines_);

    param_loader.loadParam("timer_rates/main",       _rate_main_timer_);
    param_loader.loadParam("timer_rates/PRM_update", _rate_PRM_timer_);
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
    sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "diagnostics_in", &Explorer::controlManagerDiagCallback, this);


    service_server_get_path_ = nh_.advertiseService("get_path_in", &Explorer::callbackGetPath, this);
    service_server_explore_= nh_.advertiseService("explore_in", &Explorer::callbackExplore, this);


    pub_reference_           = nh_.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);
    sc_get_trajectory_       = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
    sc_trajectory_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");


    timer_main_       = nh_.createTimer(ros::Rate(_rate_main_timer_), &Explorer::timerMain,      this);
    timer_PRM_update_ = nh_.createTimer(ros::Rate(_rate_PRM_timer_),  &Explorer::timerPRMupdate, this);
    timer_path_       = nh_.createTimer(ros::Rate(_rate_path_timer_), &Explorer::timerPath,      this);


    transformer_ = std::make_unique<mrs_lib::Transformer>("MrsExplorer");
    transformer_->setDefaultPrefix(_uav_name_);
    transformer_->retryLookupNewest(true);


    bv_frontiers_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_frontiers", "");
    bv_frontiers_->setPointsScale(_scale_points_);
    bv_frontiers_->setLinesScale(_scale_lines_);

    bv_path_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_path", "");
    bv_path_->setPointsScale(_scale_points_);
    bv_path_->setLinesScale(_scale_lines_*2.0);

    bv_prm_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_prm", "");
    bv_prm_->setPointsScale(_scale_points_);
    bv_prm_->setLinesScale(_scale_lines_/2.0);


    frontier_manager_ = std::make_unique<FrontierManager>(bv_frontiers_,
                                                          _free_space_dia_,
                                                          _frontier_min_size_,
                                                          _min_eigen_,
                                                          _size_decrease_ratio_,
                                                          _sample_attemps_,
                                                          _viewpoint_sample_radius_,
                                                          _viewpoint_sample_height_,
                                                          _viewpoint_max_distance_,
                                                          _viewpoint_max_angle_,
                                                          _max_viewpoint_cnt_,
                                                          _min_viewpoint_cnt_,
                                                          _viewpoint_resample_probability_,
                                                          _min_viewpoint_coverage_);
    prm_manager_      = std::make_unique<PRM>(bv_prm_,
                                              _free_space_dia_,
                                              _ovelap_coefficient_,
                                              _resample_factor_,
                                              _node_max_age_,
                                              _max_neighbors_,
                                              _min_neighbor_distance_,
                                              _max_neighbor_distance_);
    flight_zone_ = {.min=octomap::point3d(-_flight_zone_width_/2.0, -_flight_zone_width_/2.0, _flight_zone_floor_), 
                    .max=octomap::point3d( _flight_zone_width_/2.0 , _flight_zone_width_/2.0, _flight_zone_floor_+_flight_zone_height_)};

    auto distace_func = bind(&PRM::distance, prm_manager_.get(), std::placeholders::_1, std::placeholders::_2);
    tsp_solver_ = TSPsolver(_max_tsp_duration_, distace_func); 

    map_update_ = false;
    map_ready_ = false;
    tsp_ready_ = false;
    first_path_planend_ = false;
    
    is_initialized_ = true;
    bv_map_frame_set_ = false;
    state_ = STATE_IDLE;

    ROS_INFO("[MrsExplorer]: initialized");
  }


  void Explorer::timerMain([[maybe_unused]] const ros::TimerEvent& evt) 
  {
    // ROS_ERROR_THROTTLE(1.0, "[MrsExplorer]: MAIN LOOP in '%s' STATE", _state_names_[state_].c_str());
    if (!is_initialized_ || !map_ready_) {
      return;
    }

    if (state_ == STATE_MAP_UPDATED)
    {
      bool frontiers_found = getFrontiers();

      if (!frontiers_found)
      {
        ROS_WARN("[MrsExplorer]: frontiers were not found");
        return;
      }
      ROS_INFO("[MrsExplorer]: new frontiers succesfuly found");
      bv_frontiers_->publish();
      bv_frontiers_->clearBuffers();
      changeState(STATE_FRONTIERS_UPDATED);
    }
    if (state_ == STATE_PRM_UPDATED){
      {
        std::scoped_lock lock(mutex_TSP_, mutex_PRM_, mutex_frontiers_);
        tsp_solver_.removeFrontiers(frontier_manager_->removed_frontiers_);
        tsp_solver_.addFrontiers(frontier_manager_->added_frontiers_);
      }
      tsp_ready_  = true;
      changeState(STATE_WAITING);
    }
  }


  // updates the PRM map
  void Explorer::timerPRMupdate([[maybe_unused]] const ros::TimerEvent& evt) 
  {
    if (!is_initialized_) 
    {
      return;
    }

    if (!map_ready_)
    {
      ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: map not ready, cannot update PRM map");
      return;
    }
    // ROS_ERROR("PRM UPDATE 1");
    ros::Time t0 = ros::Time::now();

    std::shared_ptr<OcTree_t> tree;
    {
      std::scoped_lock lock(mutex_octree_);
      tree = std::make_shared<OcTree_t>(*octree_);
    }

    AABB local_zone = mrs_lib::get_mutexed(mutex_local_zone_, local_zone_);
    
    if (state_ == STATE_FRONTIERS_UPDATED)
    {
      // ROS_ERROR("PRM UPDATE 2");
      std::scoped_lock lock(mutex_PRM_, mutex_frontiers_);
      for (auto &viewpoint_position : frontier_manager_->added_frontiers_)
      {
        prm_manager_->addNode(viewpoint_position);
      }
      prm_manager_->updateZone(tree, local_zone, true);
      changeState(STATE_PRM_UPDATED);
    }
    else
    {
      std::scoped_lock lock(mutex_PRM_);
      prm_manager_->updateZone(tree, local_zone, false);
    }


    // ROS_ERROR("PRM DONE");
    
    ros::Duration dt = ros::Time::now() - t0;
    ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: PRM zone update %.1fms", dt.toNSec()/1000000.0);
    bv_prm_->publish();
    bv_prm_->clearBuffers();
    // ROS_ERROR("PRM DONE");
  }


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

    // ROS_ERROR("MAKE PATH");


    // ROS_ERROR("PLANNIG PATH timer out");
    ros::Time t0 = ros::Time::now();

    // dont generate trajectory if path generation unsuccesful of no new path needed
    // ROS_ERROR("PLANNIG PATH pre call makePath");
    bool new_path = makePath();
    // ROS_ERROR("PATH MADE");
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


  // processes new global map and extracts new frontiers
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
      bv_frontiers_->setParentFrame(msg->header.frame_id);
      bv_path_->setParentFrame(msg->header.frame_id);
      bv_prm_->setParentFrame(msg->header.frame_id);
      bv_map_frame_set_ = true;
    }

    map_ready_  = true;
    // map_update_ = true;
    changeState(STATE_MAP_UPDATED);
    
  }


  bool Explorer::getFrontiers(){

    ROS_INFO("[MrsExplorer]: starting FR extraction");
    ros::Time t0 = ros::Time::now();

    auto res = getPosition();
    if (!res){
      ROS_ERROR("[MRsExplorer] has no reference");
      return false;
    }

    std::shared_ptr<OcTree_t> tree;
    {
      std::scoped_lock lock(mutex_octree_);
      tree = std::make_shared<OcTree_t>(*octree_);
    }

    auto pos = res.value().reference.position;
    octomap::point3d start_coord(pos.x, pos.y, pos.z);
    octomap::OcTreeKey start_key = tree->coordToKey(start_coord);
    octomap::OcTreeNode* start_node = tree->search(start_key);
    start_coord = tree->keyToCoord(start_key);

    if (!start_node || tree->isNodeOccupied(start_node))
    {
      ROS_WARN("[MRsExplorer] start not present in tree or occupied");
      return false;
    }
    // extract frontiers only from local area, frontiers outside cannot change
    AABB local_zone = aabbFromCenter(start_coord, _local_zone_width_, _local_zone_width_, _local_zone_height_);
    local_zone = makeIntersection(local_zone, flight_zone_);
    mrs_lib::set_mutexed(mutex_local_zone_, local_zone, local_zone_);

    {
      std::scoped_lock lock(mutex_frontiers_);
      frontier_manager_->processNewMap(tree, local_zone, start_key);
    }

    ros::Duration dt = ros::Time::now() - t0;
    ROS_INFO("[MrsExplorer]: time to find frontiers %.1fms", dt.toNSec()/1000000.0);
    return true;
    
}
  


bool Explorer::makePath()
{
  // ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: starting path (re)plannig");

  ros::Time t0 = ros::Time::now();

  auto res = getPosition();
  if (!res){
    ROS_WARN_THROTTLE(1.0,"[MRsExplorer] has no reference");
    return false;
  }
  // get cuurrent position
  auto pos = res.value().reference.position;
  octomap::point3d start_coord(pos.x, pos.y, pos.z);
  
  // extract tracker predition
  octomath::Vector3 velocity(0.0,0.0,0.0);
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
      return false;
    }
    else 
    {  
      velocity = octomath::Vector3(prediction.velocity[0].x,prediction.velocity[0].y,prediction.velocity[0].z);
    }
  } 
  else
  {
    ROS_WARN_THROTTLE(1.0, "[MrsExplorer]: could not get controller prediction");
    return false;
  }
  
  // check if current path from predition is in collision
  bool isInFreeSpace = false;
  std::shared_ptr<OcTree_t> tree;
  {
    std::scoped_lock lock(mutex_octree_);
    tree = std::make_shared<OcTree_t>(*octree_);
  }
  for (auto & point : prediction.position)
  {
    isInFreeSpace = isFreeSpace(octomap::point3d(point.x,point.y,point.z), _flight_free_distance_, tree);
    if (!isInFreeSpace)
    {
      ROS_WARN("[MrsExplorer]: collision detected in trajectory, replanning");
      std::scoped_lock lock(mutex_TSP_);
      // std::vector<octomap::point3d> to_remove =  {goal_};
      // tsp_solver_.removeFrontiers(std::vector<octomap::point3d> {goal_});
      break;
    }
  }
  // if is in collision recalculate path, otherwise check if drone is near end of the path and replan, othervise do nothing 
  geometry_msgs::Point p = prediction.position.back();
  if (isInFreeSpace && start_coord.distance(octomap::point3d(p.x,p.y,p.z)) > _replanning_distance_)
  {
    ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: not planning, too far from goal %f", start_coord.distance(octomap::point3d(p.x,p.y,p.z)));
    return false;
  }

  // ROS_ERROR("PREGLOB PATH");
  // if (!first_path_planend_){
  //   current_viewpoint_ = start_coord;
  // }
  current_viewpoint_ = start_coord;
  
  std::vector<octomap::point3d> glob_path;
  {
    // ROS_ERROR("IN MUTEX");
    std::scoped_lock lock(mutex_TSP_, mutex_PRM_);
    // ROS_ERROR("pre start");
    tsp_solver_.setStart(current_viewpoint_);
    // ROS_ERROR("post start - pre solve");
    
  }
  {
    std::scoped_lock lock(mutex_TSP_);
    glob_path = tsp_solver_.solve(velocity);
    // ROS_ERROR("post solve");
  }
  // ROS_ERROR("POSTGLOB PATH");
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
  std::vector<octomap::point3d> path(0);
  int i = 1;
  while((path.size() == 0 && i < glob_path.size()) || start_coord.distance(path.back()) < _replanning_distance_)
  {
    {
      std::scoped_lock lock(mutex_PRM_);
      path = prm_manager_->findPath(glob_path[0], glob_path[i++], velocity);
    }
  }
  
  // add only path to fisrt viewpoint to make into trajectory later
  if (path.size() > 0)
  {
    {
      std::scoped_lock lock(mutex_PRM_);
      path = prm_manager_->simplifyFreeSpacePath(path);
      std::reverse(path.begin(), path.end());
      path = prm_manager_->simplifyFreeSpacePath(path);
      std::reverse(path.begin(), path.end());
      goal_ = path.back();
    }
    // int i = 1;
    // while (i < glob_path.size()-1 && start_coord.distance(glob_path[i]) < _replanning_distance_){
    //   i++;
    // }
    // current_viewpoint_ = glob_path[i];
    first_path_planend_ = true;

    path_ = std::vector<octomap::point3d>(0);
    for (int i=1; i<path.size(); i++)
    {
      // dont insert path points that are too close to each other for better trajetory following 
      auto point = path[i];
      if (path_.size() > 0 && path_.back().distance(point) < _skip_path_point_distance_){
        continue;
      }
      path_.push_back(point);
    }
  }
  else
  {
    ROS_WARN("[MrsExplorer]: path was not found");
    return false;
  }

  if (path_.size() > 0)
  {
    ros::Duration dt = ros::Time::now() - t0;
    ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: time to replan %.1fms", dt.toNSec()/1000000.0);
    return true;
  }
  
  ROS_WARN("[MRsExplorer] no suitable frontiers");
  return false;
}

      
  

// make trajectory from found path
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



std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> Explorer::getPosition()
{

  const bool got_control_manager_diag = sh_control_manager_diag_.hasMsg() && (ros::Time::now() - sh_control_manager_diag_.lastMsgTime()).toSec() < 2.0;
  const bool got_tracker_cmd   = sh_tracker_cmd_.hasMsg() && (ros::Time::now() - sh_tracker_cmd_.lastMsgTime()).toSec() < 2.0;
  if (!got_control_manager_diag || !got_tracker_cmd){
    ROS_WARN("[MrsExplorer]: tracker not redy");
    return {};
  }

  mrs_msgs::TrackerCommandConstPtr tracker_cmd  = sh_tracker_cmd_.getMsg();
  auto                             octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
  // transform the position cmd to the map frame
  mrs_msgs::ReferenceStamped position_cmd_ref;
  position_cmd_ref.header               = tracker_cmd->header;
  position_cmd_ref.reference.position.x = tracker_cmd->position.x;
  position_cmd_ref.reference.position.y = tracker_cmd->position.y;
  position_cmd_ref.reference.position.z = tracker_cmd->position.z;
  position_cmd_ref.reference.heading    = tracker_cmd->heading;
  std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> res = transformer_->transformSingle(position_cmd_ref, octree_frame);
  if (!res) {
      ROS_WARN("[MrsExplorer]: could not transform position cmd to the map frame");
      return {};
    }
  return res;
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
void Explorer::changeState(const State_t new_state) {

  const State_t old_state = state_;

  ROS_INFO("[MrsExplorer]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

  state_ = new_state;
}

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



bool Explorer::callbackExplore(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response& res)
{
  if (!is_initialized_) {
    return false;
  }
  return true;

  // const bool got_octomap = sh_octomap_.hasMsg() && (ros::Time::now() - sh_octomap_.lastMsgTime()).toSec() < 2.0;

  // if (!got_octomap) {
  //   ROS_INFO_THROTTLE(1.0,
  //                     "[MrsExplorer]: waiting for data: octomap = %s",
  //                     got_octomap ? "TRUE" : "FALSE");
  //   return false;
  // }
  // res.success         = true;
  // res.message = "STARTING EXPLORATION";

  // changeState(STATE_EXPLORING);

  // return true;

}





bool Explorer::callbackGetPath(mrs_octomap_planner::Path::Request&  req,
                                              mrs_octomap_planner::Path::Response& res)
  {
   
    return false;
  }


}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_planner::Explorer,
                       nodelet::Nodelet)

