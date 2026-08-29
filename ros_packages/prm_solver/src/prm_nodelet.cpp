#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <prm_solver/prm.hpp>
#include <prm_solver/FindSimplifiedPath.h>
#include <octomap_planner_utils/utils.hpp>
#include <frontier_detection/FrontierArray.h>

#include <optional>
#include <memory>
#include <mutex>


namespace prm_solver
{

  using OcTree_t          = octomap::OcTree;
  using OcTreeSharedPtr_t = std::shared_ptr<octomap::OcTree>;


  class PRMNodelet : public nodelet::Nodelet
  {
    public:
      virtual void onInit();

    private:
      ros::NodeHandle nh_;

      bool        is_initialized_ = false;
      std::string _uav_name_;

      // params
      double _flight_zone_width_x_;
      double _flight_zone_width_y_;
      double _flight_zone_height_;
      double _flight_zone_floor_;
      double _local_zone_width_;
      double _local_zone_height_;
      double _free_space_dia_;
      double _ovelap_coefficient_;
      double _resample_factor_;
      double _max_cost_;
      int    _node_max_age_;
      int    _max_neighbors_;
      double _min_neighbor_distance_;
      double _max_neighbor_distance_;
      double _min_node_distance_;
      int    _init_matrix_size_;
      double _scale_points_;
      double _scale_lines_;
      double _rate_update_;

      octomap_planner_utils::AABB flight_zone_;

      std::mutex                                mutex_octree_;
      std::mutex                                mutex_prm_;
      std::shared_ptr<OcTree_t>                 octree_ = nullptr;
      std::string                               octree_frame_;
      std::shared_ptr<mrs_lib::BatchVisualizer> bv_prm_;
      bool                                       bv_map_frame_set_ = false;
      std::atomic<bool>                          map_ready_;
      std::atomic<bool>                          map_updated_;

      std::unique_ptr<PRM>                  prm_;
      std::unique_ptr<mrs_lib::Transformer> transformer_;

      mrs_lib::SubscribeHandler<octomap_msgs::Octomap>                sh_octomap_;
      mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>              sh_tracker_cmd_;
      mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>   sh_control_manager_diag_;
      mrs_lib::SubscribeHandler<frontier_detection::FrontierArray>     sh_frontiers_;

      ros::Timer timer_update_;
      void       timerUpdate([[maybe_unused]] const ros::TimerEvent& evt);

      ros::ServiceServer service_server_find_simplified_path_;

      void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
      void callbackFrontiers(const frontier_detection::FrontierArray::ConstPtr msg);
      void controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);

      void timeoutOctomap(const std::string& topic, const ros::Time& last_msg);
      void timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg);

      bool callbackFindSimplifiedPath(prm_solver::FindSimplifiedPath::Request&  req,
                                       prm_solver::FindSimplifiedPath::Response& res);

      std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void>>> getPosition();
      std::optional<OcTreeSharedPtr_t>                                 msgToMap(const octomap_msgs::OctomapConstPtr octomap);
  };


  // nodelet entry point: loads params, sets up subscribers/service/visualizer, and constructs the PRM instance
  void PRMNodelet::onInit()
  {
    ros::Time::waitForValid();

    ROS_INFO("[PRMNodelet]: initializing...");
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh_, "PRMNodelet");

    param_loader.loadParam("uav_name", _uav_name_);

    param_loader.loadParam("zone_x", _flight_zone_width_x_);
    param_loader.loadParam("zone_y", _flight_zone_width_y_);
    param_loader.loadParam("zone_z", _flight_zone_height_);
    param_loader.loadParam("zone/global/floor", _flight_zone_floor_);
    param_loader.loadParam("zone/local/width", _local_zone_width_);
    param_loader.loadParam("zone/local/height", _local_zone_height_);

    param_loader.loadParam("prm/free_space_diameter", _free_space_dia_);
    param_loader.loadParam("prm/overlap_coefficient", _ovelap_coefficient_);
    param_loader.loadParam("prm/resample_factor", _resample_factor_);
    param_loader.loadParam("prm/max_cost", _max_cost_);
    param_loader.loadParam("prm/node_max_age", _node_max_age_);
    param_loader.loadParam("prm/max_neighbors", _max_neighbors_);
    param_loader.loadParam("prm/min_neighbor_distance", _min_neighbor_distance_);
    param_loader.loadParam("prm/max_neighbor_distance", _max_neighbor_distance_);
    param_loader.loadParam("prm/min_node_distance", _min_node_distance_);
    param_loader.loadParam("prm/init_matrix_size", _init_matrix_size_);

    param_loader.loadParam("viz/scale/points", _scale_points_);
    param_loader.loadParam("viz/scale/lines", _scale_lines_);

    param_loader.loadParam("timer_rates/update", _rate_update_);

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[PRMNodelet]: Could not load all parameters");
      ros::shutdown();
    }

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = "PRMNodelet";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 1;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", ros::Duration(5.0),
                                                                    &PRMNodelet::timeoutOctomap, this,
                                                                    &PRMNodelet::callbackOctomap, this);
    sh_tracker_cmd_          = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in", ros::Duration(3.0), &PRMNodelet::timeoutTrackerCmd, this);
    sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "diagnostics_in", &PRMNodelet::controlManagerDiagCallback, this);
    sh_frontiers_            = mrs_lib::SubscribeHandler<frontier_detection::FrontierArray>(shopts, "frontiers_in", &PRMNodelet::callbackFrontiers, this);

    service_server_find_simplified_path_ = nh_.advertiseService("find_simplified_path_in", &PRMNodelet::callbackFindSimplifiedPath, this);

    transformer_ = std::make_unique<mrs_lib::Transformer>("PRMNodelet");
    transformer_->setDefaultPrefix(_uav_name_);
    transformer_->retryLookupNewest(true);

    bv_prm_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_prm", "");
    bv_prm_->setPointsScale(_scale_points_);
    bv_prm_->setLinesScale(_scale_lines_/2.0);

    prm_ = std::make_unique<PRM>(bv_prm_,
                                  _free_space_dia_,
                                  _ovelap_coefficient_,
                                  _resample_factor_,
                                  _node_max_age_,
                                  _max_neighbors_,
                                  _min_neighbor_distance_,
                                  _max_neighbor_distance_,
                                  _min_node_distance_);

    flight_zone_ = {.min=octomap::point3d(-_flight_zone_width_x_/2.0, -_flight_zone_width_y_/2.0, _flight_zone_floor_),
                    .max=octomap::point3d( _flight_zone_width_x_/2.0,  _flight_zone_width_y_/2.0, _flight_zone_floor_+_flight_zone_height_)};

    timer_update_ = nh_.createTimer(ros::Rate(_rate_update_), &PRMNodelet::timerUpdate, this);

    map_ready_ = false;
    map_updated_ = false;

    is_initialized_ = true;
    ROS_INFO("[PRMNodelet]: initialized!");
  }

  // octomap subscription callback: converts the incoming message to an OcTree and stores it (mutexed) for the update timer to consume
  void PRMNodelet::callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    ROS_INFO("[PRMNodelet]: octomap recieved");

    std::optional<OcTreeSharedPtr_t> octree_local = msgToMap(msg);

    if (!octree_local) {
      ROS_WARN_THROTTLE(1.0, "[PRMNodelet]: received map is empty!");
      return;
    }

    mrs_lib::set_mutexed(mutex_octree_, octree_local.value(), octree_);
    mrs_lib::set_mutexed(mutex_octree_, msg->header.frame_id, octree_frame_);

    if (!bv_map_frame_set_) {
      bv_prm_->setParentFrame(msg->header.frame_id);
      bv_map_frame_set_ = true;
    }

    map_ready_   = true;
    map_updated_ = true;

    ROS_INFO("[PRMNodelet]: octomap processed");
  }

  // periodic timer (timer_rates/update): snapshots the current octree, computes the local zone around the UAV, and calls PRM::updateZone on it, then republishes the roadmap visualization
  void PRMNodelet::timerUpdate([[maybe_unused]] const ros::TimerEvent& evt)
  {
    if (!is_initialized_) {
      return;
    }

    if (!map_ready_) {
      ROS_WARN_THROTTLE(1.0, "[PRMNodelet]: map not ready, cannot update PRM map");
      return;
    }

    ROS_INFO("[PRMNodelet]: timer start");

    bool map_update = map_updated_.exchange(false);

    std::shared_ptr<OcTree_t> tree;
    {
      std::scoped_lock lock(mutex_octree_);
      tree = std::make_shared<OcTree_t>(*octree_);
    }

    octomap_planner_utils::AABB local_zone = flight_zone_;
    auto res = getPosition();
    if (res) {
      local_zone = octomap_planner_utils::localZoneFromPosition(res.value().reference.position, flight_zone_, _local_zone_width_, _local_zone_height_);
    }

    {
      std::scoped_lock lock(mutex_prm_);
      prm_->updateZone(tree, local_zone, map_update);
    }

    bv_prm_->publish();
    bv_prm_->clearBuffers();

    ROS_INFO("[PRMNodelet]: timer end");
  }

  // frontiers subscription callback: for every frontier in the message (not just newly-added ones), calls PRM::addNode on its first viewpoint, adding/deduplicating a roadmap node there
  void PRMNodelet::callbackFrontiers(const frontier_detection::FrontierArray::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    ROS_INFO("[PRMNodelet]: frontiers recieved");

    std::scoped_lock lock(mutex_prm_);
    for (auto &f : msg->frontiers)
    {
      if (!f.viewpoints.empty())
      {
        prm_->addNode(octomap::point3d(f.viewpoints[0].position.x, f.viewpoints[0].position.y, f.viewpoints[0].position.z));
      }
    }

    ROS_INFO("[PRMNodelet]: frontiers processed");
  }

  // ~find_simplified_path_in service handler, the package's only externally-callable entry point: wraps PRM::findSimplifiedPath between req.start/req.goal and returns the flyable path (res.success false/path empty if none found)
  bool PRMNodelet::callbackFindSimplifiedPath(prm_solver::FindSimplifiedPath::Request&  req,
                                               prm_solver::FindSimplifiedPath::Response& res)
  {
    if (!is_initialized_) {
      res.success = false;
      return true;
    }

    ROS_INFO("[PRMNodelet]: callbackFindSimplifiedPath recieved");

    octomap::point3d start(req.start.x, req.start.y, req.start.z);
    octomap::point3d goal(req.goal.x, req.goal.y, req.goal.z);
    octomath::Vector3 velocity(req.velocity.x, req.velocity.y, req.velocity.z);

    std::vector<octomap::point3d> path;
    {
      std::scoped_lock lock(mutex_prm_);
      path = prm_->findSimplifiedPath(start, goal, velocity);
    }

    for (auto &p : path)
    {
      geometry_msgs::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      res.path.push_back(point);
    }

    res.success = res.path.size() > 1;
    ROS_INFO("[PRMNodelet]: callbackFindSimplifiedPath processed");
    return true;
  }

  void PRMNodelet::controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg)
  {
    // only used for freshness checks via the SubscribeHandler, see getPosition()
  }

  // SubscribeHandler timeout callback for octomap_in: logs a throttled warning if a message was previously received but has since gone stale
  void PRMNodelet::timeoutOctomap(const std::string& topic, const ros::Time& last_msg)
  {
    if (!is_initialized_ || !sh_octomap_.hasMsg()) {
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[PRMNodelet]: octomap timeout!");
  }

  // SubscribeHandler timeout callback for tracker_cmd_in: logs a throttled warning if a message was previously received but has since gone stale
  void PRMNodelet::timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg)
  {
    if (!is_initialized_ || !sh_tracker_cmd_.hasMsg()) {
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[PRMNodelet]: position cmd timeouted!");
  }

  // returns the UAV's current reference position transformed into the octree's frame, or nullopt if unavailable
  std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void>>> PRMNodelet::getPosition()
  {
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    return octomap_planner_utils::getPosition(sh_control_manager_diag_, sh_tracker_cmd_, octree_frame, *transformer_, "[PRMNodelet]");
  }

  // converts an octomap_msgs::Octomap (binary or full) into an OcTree, or nullopt if the message can't be decoded
  std::optional<OcTreeSharedPtr_t> PRMNodelet::msgToMap(const octomap_msgs::OctomapConstPtr octomap)
  {
    octomap::AbstractOcTree* abstract_tree;

    if (octomap->binary) {
      abstract_tree = octomap_msgs::binaryMsgToMap(*octomap);
    }
    else {
      abstract_tree = octomap_msgs::fullMsgToMap(*octomap);
    }

    if (!abstract_tree) {
      ROS_WARN("[PRMNodelet]: Octomap message is empty! can not convert to OcTree");
      return {};
    }
    else {
      return { OcTreeSharedPtr_t(dynamic_cast<OcTree_t*>(abstract_tree)) };
    }
  }


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(prm_solver::PRMNodelet,
                       nodelet::Nodelet)
