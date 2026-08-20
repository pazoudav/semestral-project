
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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
// #include <tf2_eigen/tf2_eigen.h>

#include <unordered_set>
#include <vector>
#include <queue>
#include <iostream>
#include <memory>
#include <bits/stdc++.h>
#include <set>

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


struct trans_cost_t
{
  Eigen::Matrix4f matrix;
  Eigen::Matrix4f matrix_inv;
  float cost;
};


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
    double _flight_zone_width_x_;
    double _flight_zone_width_y_;
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr  source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr  target_cloud_;
    std::vector<geometry_msgs::Point> og_skeleton_msg_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr  source_viewpoint_candidates_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_viewpoint_candidates_;
    

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
    bool              bv_map_frame_set_;
    bool              braking_;
    octomap::point3d last_free_point_;
    // std::atomic<bool> valid_path_;
    octomap::point3d position_before_map_update_;
    octomap::point3d goal_;
    octomap::point3d next_goal_;
    std::shared_ptr<OcTree_t> big_tree_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr skeleton_kdtree_;
    std::vector<trans_cost_t> tranfomation_history_;


    // subscribers
    mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand> sh_tracker_cmd_;
    mrs_lib::SubscribeHandler<octomap_msgs::Octomap>    sh_octomap_;
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
    mrs_lib::SubscribeHandler<visualization_msgs::Marker> sh_source_skeleton;
    mrs_lib::SubscribeHandler<visualization_msgs::Marker> sh_target_skeleton;
    mrs_lib::SubscribeHandler<visualization_msgs::MarkerArray> sh_source_viewpoints;

    // publishers
    ros::Publisher pub_reference_;
    ros::Publisher pub_transfomed_skeleton_;
    ros::Publisher pub_ransac_skeleton_;
    ros::Publisher pub_big_ocotmap_;
    ros::Publisher pub_candidate_viewpoints_;
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
    void sourceSkeletonCallback(const visualization_msgs::Marker::ConstPtr msg);
    void targetSkeletonCallback(const visualization_msgs::Marker::ConstPtr msg);
    void sourceViewpointsCallback(const visualization_msgs::MarkerArray::ConstPtr msg);


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
    // void       timerPath([[maybe_unused]] const ros::TimerEvent& evt);

    

    std::unique_ptr<mrs_lib::Transformer> transformer_;

    std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> getPosition();
    std::optional<mrs_msgs::MpcPredictionFullState>                   getFullStatePrediction();
    std::optional<OcTreeSharedPtr_t>                                  msgToMap(const octomap_msgs::OctomapConstPtr octomap);
    bool                                                              getFrontiers();
    bool                                                              makePath();
    bool                                                              makeTrajectory();
    bool                                                              setPath(std::vector<octomap::point3d> path);
    bool                                                              checkTrajectoryCollision();
    
    
    void  loadSkeleton(const visualization_msgs::Marker::ConstPtr msg, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
    Eigen::Matrix4f runRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cs,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr ct,
                              const std::vector<std::vector<int>>& candidates,
                              int iterations);
    Eigen::Matrix4f estimateTransform(const std::vector<Eigen::Vector3f>& src,
                                      const std::vector<Eigen::Vector3f>& tgt);

  };

  void Explorer::onInit()
  {
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ros::Time::waitForValid();

    ROS_INFO("[MrsExplorer]: initializing");

    mrs_lib::ParamLoader param_loader(nh_, "MrsExplorer");

    param_loader.loadParam("uav_name", _uav_name_);

    param_loader.loadParam("zone_x",_flight_zone_width_x_);
    param_loader.loadParam("zone_y",_flight_zone_width_y_);
    param_loader.loadParam("zone_z",_flight_zone_height_);
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
    sh_control_manager_diag_  = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "diagnostics_in", &Explorer::controlManagerDiagCallback, this);
    sh_source_skeleton        = mrs_lib::SubscribeHandler<visualization_msgs::Marker>(shopts, "source_skeleton_in", &Explorer::sourceSkeletonCallback, this);
    sh_target_skeleton        = mrs_lib::SubscribeHandler<visualization_msgs::Marker>(shopts, "target_skeleton_in", &Explorer::targetSkeletonCallback, this);  
    sh_source_viewpoints      = mrs_lib::SubscribeHandler<visualization_msgs::MarkerArray>(shopts, "source_viewpoints_in", &Explorer::sourceViewpointsCallback, this);

    service_server_get_path_ = nh_.advertiseService("get_path_in", &Explorer::callbackGetPath, this);
    service_server_explore_= nh_.advertiseService("explore_in", &Explorer::callbackExplore, this);


    pub_reference_            = nh_.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);
    pub_transfomed_skeleton_  = nh_.advertise<visualization_msgs::Marker>("transformed_skeleton_out", 1);
    pub_ransac_skeleton_      = nh_.advertise<visualization_msgs::Marker>("ransac_skeleton_out", 1);  
    pub_big_ocotmap_          = nh_.advertise<visualization_msgs::MarkerArray>("big_octomap_out", 1);  
    pub_candidate_viewpoints_ = nh_.advertise<sensor_msgs::PointCloud2>("candidate_viewpoints_out", 1);

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
    flight_zone_ = {.min=octomap::point3d(-_flight_zone_width_x_/2.0, -_flight_zone_width_y_/2.0, _flight_zone_floor_), 
                    .max=octomap::point3d( _flight_zone_width_x_/2.0 , _flight_zone_width_y_/2.0, _flight_zone_floor_+_flight_zone_height_)};

    auto distace_func = bind(&PRM::extraDistance, prm_manager_.get(), std::placeholders::_1, std::placeholders::_2);
    tsp_solver_ = TSPsolver(_max_tsp_duration_, distace_func); 

    map_update_ = false;
    map_ready_ = false;
    tsp_ready_ = false;
    first_path_planend_ = false;
    braking_ = false;
    source_cloud_        = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    target_cloud_        = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    skeleton_kdtree_     = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>(); // pcl::KdTreeFLANN<pcl::PointXYZ>
    source_viewpoint_candidates_ = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    tranfomation_history_ = std::vector<trans_cost_t>(0);;
    
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
      {
        std::scoped_lock lock(mutex_TSP_, mutex_PRM_, mutex_frontiers_);
        tsp_solver_.removeFrontiers();
        tsp_solver_.addFrontiers(frontier_manager_->fis_c_);
      }
      tsp_ready_  = true;
      changeState(STATE_FRONTIERS_UPDATED);
    }

    // ROS_WARN("main runnig");

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
      // ROS_ERROR("PRM UPDATE --------------------------------------------------------");

      changeState(STATE_WAITING);
    }
    else
    {
      // ROS_ERROR("PRM UPDATE ========================================================");
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
    getFrontiers();
    map_update_ = true;
    
    changeState(STATE_MAP_UPDATED);
    ROS_INFO("[MrsExplorer]: new frontiers succesfuly found");
    
    bv_frontiers_->publish();
    bv_frontiers_->clearBuffers();
    
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
  if (isFreeSpace(start_coord, _free_space_dia_, tree)){
    last_free_point_ = start_coord;
  }
  for (auto & point : prediction.position)
  {
    isInFreeSpace = isFreeSpace(octomap::point3d(point.x,point.y,point.z), _flight_free_distance_, tree);
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
    std::scoped_lock lock(mutex_TSP_);
    {
      std::scoped_lock lock(mutex_PRM_);
      tsp_solver_.setStart(current_viewpoint_);
      ROS_ERROR("TSP start set");
    }
    glob_path = tsp_solver_.solve(velocity);
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

    auto temp_path = prm_manager_->findPath(sub_global_path[j], sub_global_path[j+1],  j == 0 ? velocity : velocity*0.0);
    if (temp_path.size() < 2)
    {
      ROS_WARN("temp path not found");
      return false;
    }
    {
      std::scoped_lock lock(mutex_PRM_);
      temp_path = prm_manager_->simplifyFreeSpacePath(temp_path);
      std::reverse(temp_path.begin(), temp_path.end());
      temp_path = prm_manager_->simplifyFreeSpacePath(temp_path);
      std::reverse(temp_path.begin(), temp_path.end());
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
    isInFreeSpace = isFreeSpace(octomap::point3d(point.x,point.y,point.z), _flight_free_distance_, tree);
    if (!isInFreeSpace)
    {
      // goal_ = start_coord;
      ROS_WARN("[MrsExplorer]: collision detected in trajectory, replanning");
      return true;
    }
  }
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

bool Explorer::setPath(std::vector<octomap::point3d> path)
{
  if (path.size() == 0)
  {
    return false;
  }
  
  {
    std::scoped_lock lock(mutex_PRM_);
    path = prm_manager_->simplifyFreeSpacePath(path);
    std::reverse(path.begin(), path.end());
    path = prm_manager_->simplifyFreeSpacePath(path);
    std::reverse(path.begin(), path.end());
  }

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
  return true;
}


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

void Explorer::sourceViewpointsCallback(const visualization_msgs::MarkerArray::ConstPtr msg) 
{
  source_viewpoint_candidates_->clear();

  for (auto &p_ : msg->markers){
    if (p_.ns != "init_vps_pos" && p_.ns != "text")
    {
      continue;
    }
    auto p = p_.pose.position;
    auto scale = p_.scale;
    double x;
    double y;
    double z;
    if (p_.ns == "init_vps_pos")
    {
      x = p.x*1.0/scale.x;
      y = p.y*1.0/scale.y;
      z = p.z*1.0/scale.z;
    } else {
      x = p.x*scale.x;
      y = p.y*scale.y;
      z = p.z*scale.z;
    }
    

    ROS_INFO("Viewpoint Point: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    source_viewpoint_candidates_->push_back(pcl::PointXYZ(x, y, z));
  }
}

void Explorer::sourceSkeletonCallback(const visualization_msgs::Marker::ConstPtr msg) 
{

  loadSkeleton(msg, source_cloud_);
  // for (auto &p : msg->points){
  //   double x = p.x;
  //   double y = p.y;
  //   double z = p.z;

  //   ROS_INFO("Skeleton Point: x=%.2f, y=%.2f, z=%.2f", x, y, z);
  // }
  og_skeleton_msg_ = msg->points;
  skeleton_kdtree_->setInputCloud(source_cloud_);
}

void Explorer::targetSkeletonCallback(const visualization_msgs::Marker::ConstPtr msg) 
{

  if (!is_initialized_){
      return;
  }

  loadSkeleton(msg, target_cloud_);
  std::vector<std::vector<int>> candidates = std::vector<std::vector<int>>(0);

  for (int i = 0; i < source_cloud_->size(); i++){
    candidates.push_back(std::vector<int>(0));
    for (int j = 0; j < target_cloud_->size(); j++){
      candidates[i].push_back(j);
    }
  }

  Eigen::Matrix4f T = runRANSAC(source_cloud_, target_cloud_, candidates, 2000);

  // source_viewpoint_candidates_
  pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*source_viewpoint_candidates_, *transformed_pointcloud, T);

  {
    std::scoped_lock lock(mutex_TSP_);
    ROS_ERROR("loading tsp kd tree");
    tsp_solver_.setKDtreeInput(transformed_pointcloud);
  }

  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*transformed_pointcloud, output_msg);
  output_msg.header.frame_id = msg->header.frame_id; // Set frame
  output_msg.header.stamp = ros::Time::now();
  pub_candidate_viewpoints_.publish(output_msg);

  // tsp_solver_.guiding_viewpoints_ = transformed_viewpoint_candidates_;


  std::vector<geometry_msgs::Point> transformed_points = std::vector<geometry_msgs::Point>(0);

  for (auto &p : og_skeleton_msg_)
  {
    Eigen::Vector4f point(p.x, p.y, p.z, 1.0f);
    Eigen::Vector4f point_trans = T * point;

    geometry_msgs::Point new_point;
    new_point.x = point_trans(0);
    new_point.y = point_trans(1);
    new_point.z = point_trans(2);

    transformed_points.push_back(new_point);
  }

  // ROS_WARN("SKELETON TRANSFORMED");
  // ROS_WARN("in message frame id: %s", msg->header.frame_id);

  visualization_msgs::Marker lines;
  lines.header.frame_id = msg->header.frame_id; //"uav1/world_origin"; 
  lines.header.stamp = ros::Time::now();
  lines.id = msg->id;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;

  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.3;

  lines.color.r = 0.8;
  lines.color.g = 0.1;
  lines.color.b = 0.1;
  lines.color.a = 1.0;
  
  lines.points = transformed_points;

  pub_transfomed_skeleton_.publish(lines);

  // ROS_WARN("SKELETON PUBLISHED");

} 


void Explorer::loadSkeleton(const visualization_msgs::Marker::ConstPtr msg,  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
  // ROS_ERROR("SKELETON RECIEVED %d", msg->points.size());

  pointcloud->clear();
  float sample_size = 0.2;

  for (int i=0; i< msg->points.size(); i+=2)
  {
    Eigen::Vector3f p1(msg->points[i].x,   msg->points[i].y,   msg->points[i].z);
    Eigen::Vector3f p2(msg->points[i+1].x, msg->points[i+1].y, msg->points[i+1].z);
    Eigen::Vector3f dir = p2-p1;

    if (dir.norm() < sample_size)
    {
      Eigen::Vector3f np = p1 + dir/2.0;
      pointcloud->push_back(pcl::PointXYZ(np.x(), np.y(), np.z()));
    }
    else
    {
      int div_cnt = (int)((dir.norm())/sample_size)+1;

      for (int i=1; i<=div_cnt; i++)
      {
        Eigen::Vector3f np = p1 + ((i/(div_cnt+1)) * dir);
        pointcloud->push_back(pcl::PointXYZ(np.x(), np.y(), np.z()));
      }
    }
  }

  // ROS_ERROR("NEW CLOUD SIZE %d", pointcloud->points.size());
}

Eigen::Matrix4f Explorer::runRANSAC(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cs,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ct,
    const std::vector<std::vector<int>>& candidates,
    int iterations)
{
    std::mt19937 rng;
    std::uniform_int_distribution<> dist(0, cs->size() - 1);
    Eigen::Matrix4f bestT = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f bestT_inv = Eigen::Matrix4f::Identity();

    float best_score = 100000000.0;
    float best_scale = 0.0;

    std::vector<int> idx(1);
    std::vector<float> tree_dist(1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    for (int it = 0; it < iterations; ++it) {
        std::vector<Eigen::Vector3f> src, tgt;

        for (int k = 0; k < 3; ++k) {
            int i = dist(rng);
            if (candidates[i].empty()) continue;

            int j = candidates[i][rng() % candidates[i].size()];
            Eigen::Vector3f ps(cs->points[i].x, cs->points[i].y, cs->points[i].z);
            Eigen::Vector3f pt(ct->points[j].x, ct->points[j].y, ct->points[j].z);
            src.push_back(ps);
            tgt.push_back(pt);
        }

        if (src.size() < 3) continue;

        Eigen::Matrix4f T = estimateTransform(src, tgt);

        float scale = T(0,0);
        if (scale < 0.1 || scale > 10.0) continue;
        if (!T.allFinite()) continue;

        // Eigen::Matrix4f inv_T = T.inverse();
        Eigen::Matrix4f inv_T = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f R_T   = T.block<3,3>(0,0).transpose();
        R_T = R_T/scale;
        inv_T.block<3,3>(0,0) = R_T/scale;
        inv_T.block<3,1>(0,3) = -R_T * T.block<3,1>(0,3);

        transformed_pointcloud->clear();
        pcl::transformPointCloud(*ct, *transformed_pointcloud, inv_T);
        
        float score = 0; 

        for (const auto& point : transformed_pointcloud->points) {
          if (skeleton_kdtree_->nearestKSearch(point, 1, idx, tree_dist) > 0)
          {
            score += tree_dist[0];
          }
        }

        if (score < best_score)
        {
          best_score = score;
          best_scale = scale;
          bestT = T;
          bestT_inv = inv_T;
          // ROS_INFO("scale %f, score %f", scale, score);
        } 
    }

    

    if (tranfomation_history_.size() < 10) 
    {
      tranfomation_history_.push_back((trans_cost_t){.matrix=bestT, .matrix_inv=bestT_inv, .cost=best_score});
    }else{

      tranfomation_history_.insert(tranfomation_history_.begin(), (trans_cost_t){.matrix=bestT, .matrix_inv=bestT_inv, .cost=best_score});
      tranfomation_history_.pop_back();
    }

    best_score = 1000000000.0;
    for (auto tc : tranfomation_history_)
    {
      transformed_pointcloud->clear();
      pcl::transformPointCloud(*ct, *transformed_pointcloud, tc.matrix_inv);
      
      float score = 0; 

      for (const auto& point : transformed_pointcloud->points) {
        if (skeleton_kdtree_->nearestKSearch(point, 1, idx, tree_dist) > 0)
        {
          score += tree_dist[0];
        }
      }

      if (score < best_score)
      {
        best_score = score;
        bestT = tc.matrix;
      } 
    }

    ROS_WARN("best score %f", best_score);

    return bestT;
    
}

Eigen::Matrix4f Explorer::estimateTransform(
    const std::vector<Eigen::Vector3f>& src,
    const std::vector<Eigen::Vector3f>& tgt)
{
    const int n = src.size();

    // Compute centroids (only XY matters)
    Eigen::Vector3f centroid_tgt    = Eigen::Vector3f::Zero();
    Eigen::Vector3f centroid_src    = Eigen::Vector3f::Zero();
    
    for (int i = 0; i < n; ++i) {
        centroid_tgt   += tgt[i];
        centroid_src   += src[i];
    }
    centroid_tgt /= n;
    centroid_src /= n;

    // Compute 2x2 covariance matrix
    Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
    float var_src = 0.0f;
    float var_tgt = 0.0f;

    for (int i = 0; i < n; ++i) {
        Eigen::Vector3f pt    = tgt[i] - centroid_tgt;
        Eigen::Vector3f ps    = src[i] - centroid_src;
        

        H += ps.head<2>() * pt.head<2>().transpose();
        var_src += ps.squaredNorm();
        var_tgt += pt.squaredNorm();
    }

    // SVD on 2x2
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2f U = svd.matrixU();
    Eigen::Matrix2f V = svd.matrixV();

    // --- Reflection handling ---
    Eigen::Matrix2f S = Eigen::Matrix2f::Identity();
    if ((V * U.transpose()).determinant() < 0) {
        S(1,1) = -1;
    }

    Eigen::Matrix2f R2 = V * S * U.transpose();

    float scale = std::sqrt(var_tgt/var_src);

    // Translation in XY
    Eigen::Vector2f t2 = centroid_tgt.head<2>() - scale * R2 * centroid_src.head<2>();

    // Build full 4x4 transform
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

    // XY rotation + scale
    T(0,0) = scale * R2(0,0);
    T(0,1) = scale * R2(0,1);
    T(1,0) = scale * R2(1,0);
    T(1,1) = scale * R2(1,1);

    // Z untouched
    T(2,2) = scale;

    // Translation only in XY
    T(0,3) = t2(0);
    T(1,3) = t2(1);
    T(2,3) = 0.0f;

    return T;
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

