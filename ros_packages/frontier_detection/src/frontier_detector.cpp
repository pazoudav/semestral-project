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

#include <frontier_detection/frontier_manager.hpp>
#include <octomap_planner_utils/utils.hpp>
#include <frontier_detection/FrontierArray.h>

#include <optional>
#include <memory>
#include <mutex>


namespace frontier_detection
{

  using OcTree_t          = octomap::OcTree;
  using OcTreeSharedPtr_t = std::shared_ptr<octomap::OcTree>;


  class Detector : public nodelet::Nodelet
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
      int    _frontier_min_size_;
      int    _min_eigen_;
      double _size_decrease_ratio_;
      int    _sample_attemps_;
      int    _max_viewpoint_cnt_;
      int    _min_viewpoint_coverage_;
      double _viewpoint_sample_radius_;
      double _viewpoint_sample_height_;
      double _viewpoint_max_distance_;
      double _viewpoint_max_angle_;
      double _scale_points_;
      double _scale_lines_;

      octomap_planner_utils::AABB flight_zone_;

      std::mutex                                mutex_octree_;
      std::mutex                                mutex_frontiers_;
      std::shared_ptr<OcTree_t>                 octree_ = nullptr;
      std::string                               octree_frame_;
      std::shared_ptr<mrs_lib::BatchVisualizer> bv_frontiers_;
      bool                                       bv_map_frame_set_ = false;
      std::atomic<bool>                          map_ready_;

      std::unique_ptr<FrontierManager> frontier_manager_;
      std::unique_ptr<mrs_lib::Transformer> transformer_;

      mrs_lib::SubscribeHandler<octomap_msgs::Octomap>                sh_octomap_;
      mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>              sh_tracker_cmd_;
      mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>   sh_control_manager_diag_;

      ros::Publisher pub_frontiers_;

      void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
      void controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);

      void timeoutOctomap(const std::string& topic, const ros::Time& last_msg);
      void timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg);

      // current UAV reference position in the octomap's frame, derived from tracker command + control-manager diagnostics freshness
      std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void>>> getPosition();
      // decodes an incoming octomap message (binary or full) into an OcTree, or nullopt if the message is empty
      std::optional<OcTreeSharedPtr_t>                                 msgToMap(const octomap_msgs::OctomapConstPtr octomap);
      // builds and publishes a FrontierArray with the single best (highest-coverage) viewpoint per valid frontier
      void                                                             publishFrontiers();
  };


  void Detector::onInit()
  {
    ros::Time::waitForValid();

    ROS_INFO("[Detector]: initializing...");
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh_, "FrontierDetector");

    param_loader.loadParam("uav_name", _uav_name_);

    param_loader.loadParam("zone_x", _flight_zone_width_x_);
    param_loader.loadParam("zone_y", _flight_zone_width_y_);
    param_loader.loadParam("zone_z", _flight_zone_height_);
    param_loader.loadParam("zone/global/floor", _flight_zone_floor_);
    param_loader.loadParam("zone/local/width", _local_zone_width_);
    param_loader.loadParam("zone/local/height", _local_zone_height_);

    param_loader.loadParam("prm/free_space_diameter", _free_space_dia_);

    param_loader.loadParam("frontiers/min_size", _frontier_min_size_);
    param_loader.loadParam("frontiers/min_svd_eigen_value", _min_eigen_);
    param_loader.loadParam("frontiers/size_decrease_ratio", _size_decrease_ratio_);

    param_loader.loadParam("frontiers/viewpoint/sample_attempts", _sample_attemps_);
    param_loader.loadParam("frontiers/viewpoint/max_count", _max_viewpoint_cnt_);
    param_loader.loadParam("frontiers/viewpoint/min_coverage", _min_viewpoint_coverage_);
    param_loader.loadParam("frontiers/viewpoint/sample_radius", _viewpoint_sample_radius_);
    param_loader.loadParam("frontiers/viewpoint/sample_height", _viewpoint_sample_height_);
    param_loader.loadParam("frontiers/viewpoint/max_distance", _viewpoint_max_distance_);
    param_loader.loadParam("frontiers/viewpoint/max_angle", _viewpoint_max_angle_);

    param_loader.loadParam("viz/scale/points", _scale_points_);
    param_loader.loadParam("viz/scale/lines", _scale_lines_);

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[Detector]: Could not load all parameters");
      ros::shutdown();
    }

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = "FrontierDetector";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 1;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", ros::Duration(5.0),
                                                                    &Detector::timeoutOctomap, this,
                                                                    &Detector::callbackOctomap, this);
    sh_tracker_cmd_          = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in", ros::Duration(3.0), &Detector::timeoutTrackerCmd, this);
    sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "diagnostics_in", &Detector::controlManagerDiagCallback, this);

    pub_frontiers_ = nh_.advertise<frontier_detection::FrontierArray>("frontiers_out", 1);

    transformer_ = std::make_unique<mrs_lib::Transformer>("FrontierDetector");
    transformer_->setDefaultPrefix(_uav_name_);
    transformer_->retryLookupNewest(true);

    bv_frontiers_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_frontiers", "");
    bv_frontiers_->setPointsScale(_scale_points_);
    bv_frontiers_->setLinesScale(_scale_lines_);

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
                                                           _min_viewpoint_coverage_);

    flight_zone_ = {.min=octomap::point3d(-_flight_zone_width_x_/2.0, -_flight_zone_width_y_/2.0, _flight_zone_floor_),
                    .max=octomap::point3d( _flight_zone_width_x_/2.0,  _flight_zone_width_y_/2.0, _flight_zone_floor_+_flight_zone_height_)};

    map_ready_ = false;

    is_initialized_ = true;
    ROS_INFO("[Detector]: initialized!");
  }

  // on each new octomap: decodes it, locates the UAV, derives a local search zone around it, runs FrontierManager::processNewMap, then publishes viz + frontiers
  void Detector::callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    ROS_INFO("[Detector]: octomap recieved");

    std::optional<OcTreeSharedPtr_t> octree_local = msgToMap(msg);

    if (!octree_local) {
      ROS_WARN_THROTTLE(1.0, "[Detector]: received map is empty!");
      return;
    }

    mrs_lib::set_mutexed(mutex_octree_, octree_local.value(), octree_);
    mrs_lib::set_mutexed(mutex_octree_, msg->header.frame_id, octree_frame_);

    if (!bv_map_frame_set_) {
      bv_frontiers_->setParentFrame(msg->header.frame_id);
      bv_map_frame_set_ = true;
    }

    map_ready_ = true;

    auto res = getPosition();
    if (!res) {
      ROS_WARN_THROTTLE(1.0, "[Detector]: has no reference");
      return;
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

    if (!start_node || tree->isNodeOccupied(start_node)) {
      ROS_WARN("[Detector]: start not present in tree or occupied");
      return;
    }

    // extract frontiers only from local area, frontiers outside cannot change
    geometry_msgs::Point start_point;
    start_point.x = start_coord.x();
    start_point.y = start_coord.y();
    start_point.z = start_coord.z();
    octomap_planner_utils::AABB local_zone = octomap_planner_utils::localZoneFromPosition(start_point, flight_zone_, _local_zone_width_, _local_zone_height_);

    {
      std::scoped_lock lock(mutex_frontiers_);
      frontier_manager_->processNewMap(tree, local_zone, start_key);
    }

    bv_frontiers_->publish();
    bv_frontiers_->clearBuffers();

    publishFrontiers();
  }

  void Detector::publishFrontiers()
  {
    frontier_detection::FrontierArray msg;
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);

    {
      std::scoped_lock lock(mutex_frontiers_);
      for (auto &fis : frontier_manager_->fis_c_)
      {
        if (!fis->valid_ || fis->viewpoints_.empty()) {
          continue;
        }

        frontier_detection::Frontier frontier_msg;
        frontier_msg.id = static_cast<uint32_t>(fis->id_);
        frontier_msg.size = fis->cellCnt();
        for (auto &v : fis->viewpoints_)
        {
          frontier_detection::Viewpoint viewpoint_msg;
          viewpoint_msg.position.x = v.position.x();
          viewpoint_msg.position.y = v.position.y();
          viewpoint_msg.position.z = v.position.z();
          viewpoint_msg.coverage   = v.coverage;
          frontier_msg.viewpoints.push_back(viewpoint_msg);
          break;
        }
        msg.frontiers.push_back(frontier_msg);
      }
    }
    

    pub_frontiers_.publish(msg);
  }

  void Detector::controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg)
  {
    // only used for freshness checks via the SubscribeHandler, see getPosition()
  }

  void Detector::timeoutOctomap(const std::string& topic, const ros::Time& last_msg)
  {
    if (!is_initialized_ || !sh_octomap_.hasMsg()) {
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[Detector]: octomap timeout!");
  }

  void Detector::timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg)
  {
    if (!is_initialized_ || !sh_tracker_cmd_.hasMsg()) {
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[Detector]: position cmd timeouted!");
  }

  std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void>>> Detector::getPosition()
  {
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    return octomap_planner_utils::getPosition(sh_control_manager_diag_, sh_tracker_cmd_, octree_frame, *transformer_, "[Detector]");
  }

  std::optional<OcTreeSharedPtr_t> Detector::msgToMap(const octomap_msgs::OctomapConstPtr octomap)
  {
    octomap::AbstractOcTree* abstract_tree;

    if (octomap->binary) {
      abstract_tree = octomap_msgs::binaryMsgToMap(*octomap);
    }
    else {
      abstract_tree = octomap_msgs::fullMsgToMap(*octomap);
    }

    if (!abstract_tree) {
      ROS_WARN("[Detector]: Octomap message is empty! can not convert to OcTree");
      return {};
    }
    else {
      return { OcTreeSharedPtr_t(dynamic_cast<OcTree_t*>(abstract_tree)) };
    }
  }


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frontier_detection::Detector,
                       nodelet::Nodelet)
