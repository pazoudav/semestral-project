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

#include <path_planning/FindSimplifiedPath.h>
#include <path_planning/path_planner.hpp>
#include <octomap_planner_utils/utils.hpp>
#include <frontier_detection/FrontierArray.h>

#include <optional>
#include <memory>
#include <mutex>
// #include <shared_mutex>
#include <thread>
#include <condition_variable>


namespace path_planning
{

  using OcTree_t          = octomap::OcTree;
  using OcTreeUniquePtr_t = std::unique_ptr<octomap::OcTree>;


  // a self-contained snapshot of what bv_roadmap_ should currently show; built cheaply on the octomap-callback
  // thread and handed off to the visualization worker thread, which does the marker construction + publish,
  // mirroring frontier_detection::Detector's VisualizationSnapshot/visualizationWorker split
  struct RoadmapVisSnapshot
  {
    std::string                                                 frame_id;
    std::vector<octomap::point3d>                               nodes;
    std::vector<std::pair<octomap::point3d, octomap::point3d>>  edges;
  };


  // Skeleton nodelet: mirrors prm_solver::PRMNodelet's outward ROS interface (same in-topic
  // handle names + FindSimplifiedPath-shaped service), but fuses the octomap input like
  // frontier_detection::Detector does (local-zone map merged into a persistent global tree)
  // instead of consuming an already-global map. Actual path-planning logic (roadmap/search or
  // whatever replaces it) is not yet implemented -- everything below is wiring to be filled in.
  class PathPlanningNodelet : public nodelet::Nodelet
  {
    public:
      virtual void onInit();
      ~PathPlanningNodelet();

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
      double _rate_update_;

      RoadmapParams _roadmap_params_;
      double        _scale_points_;
      double        _scale_lines_;

      octomap_planner_utils::AABB flight_zone_;

      std::mutex                 mutex_octree_;
      OcTreeUniquePtr_t          octree_ = nullptr; // persistent global map, built up purely by merging in successive local-zone octomaps
      std::string                octree_frame_;
      std::atomic<bool>          map_ready_;

      std::mutex                    mutex_roadmap_;
      std::unique_ptr<PathPlanner>  path_planner_;

      std::shared_ptr<mrs_lib::BatchVisualizer> bv_roadmap_;
      bool                                       bv_map_frame_set_ = false;

      // background visualization worker: keeps all bv_roadmap_ marker construction + publish() off the octomap-callback thread.
      // only ever touched by visualizationWorker() itself once started, so bv_roadmap_/bv_map_frame_set_ need no locking there.
      std::thread             vis_thread_;
      std::mutex              vis_mutex_;
      std::condition_variable vis_cv_;
      std::optional<RoadmapVisSnapshot> pending_vis_snapshot_;
      bool                     vis_shutdown_ = false;

      std::unique_ptr<mrs_lib::Transformer> transformer_;

      // background tree cleanup worker: retired local octrees (once merged into octree_) are handed off here so that
      // freeing every node of a large OcTree happens off the octomap-callback thread instead of inside the callback itself
      std::thread                             tree_cleanup_thread_;
      std::mutex                              tree_cleanup_mutex_;
      std::condition_variable                 tree_cleanup_cv_;
      std::vector<OcTreeUniquePtr_t>          pending_tree_cleanup_;
      bool                                     tree_cleanup_shutdown_ = false;

      mrs_lib::SubscribeHandler<octomap_msgs::Octomap>                sh_octomap_;
      mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>              sh_tracker_cmd_;
      mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>   sh_control_manager_diag_;
      mrs_lib::SubscribeHandler<frontier_detection::FrontierArray>     sh_frontiers_;

      ros::Timer timer_update_;
      void       timerUpdate([[maybe_unused]] const ros::TimerEvent& evt);

      ros::ServiceServer service_server_find_simplified_path_;

      // merges each incoming local-zone octomap into the persistent octree_ (seeding it on the first message)
      void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
      void callbackFrontiers(const frontier_detection::FrontierArray::ConstPtr msg);
      void controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);

      void timeoutOctomap(const std::string& topic, const ros::Time& last_msg);
      void timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg);

      bool callbackFindSimplifiedPath(path_planning::FindSimplifiedPath::Request&  req,
                                       path_planning::FindSimplifiedPath::Response& res);

      std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void>>> getPosition();

      // copies the current roadmap state (under mutex_roadmap_) into a RoadmapVisSnapshot and hands it to the visualization worker
      void enqueueVisualization();
      // background loop: waits for a new snapshot, then draws it into bv_roadmap_ and publishes/clears it
      void visualizationWorker();

      // hands a retired local octree off to the cleanup worker instead of destroying it (and freeing every one of its nodes) on the caller's thread
      void retireTree(OcTreeUniquePtr_t tree);
      // background loop: waits for retired octrees and destroys them off the octomap-callback thread
      void treeCleanupWorker();
  };


  void PathPlanningNodelet::onInit()
  {
    ros::Time::waitForValid();

    ROS_INFO("[PathPlanningNodelet]: initializing...");
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    mrs_lib::ParamLoader param_loader(nh_, "PathPlanningNodelet");

    param_loader.loadParam("uav_name", _uav_name_);

    param_loader.loadParam("zone_x", _flight_zone_width_x_);
    param_loader.loadParam("zone_y", _flight_zone_width_y_);
    param_loader.loadParam("zone_z", _flight_zone_height_);
    param_loader.loadParam("zone/global/floor", _flight_zone_floor_);
    param_loader.loadParam("zone/local/width", _local_zone_width_);
    param_loader.loadParam("zone/local/height", _local_zone_height_);

    param_loader.loadParam("timer_rates/update", _rate_update_);

    param_loader.loadParam("roadmap/inflate_radius",         _roadmap_params_.inflate_radius);
    param_loader.loadParam("roadmap/target_density",         _roadmap_params_.target_density);
    param_loader.loadParam("roadmap/min_spacing",            _roadmap_params_.min_spacing);
    param_loader.loadParam("roadmap/connect_radius",         _roadmap_params_.connect_radius);
    param_loader.loadParam("roadmap/min_obstacle_clearance", _roadmap_params_.min_obstacle_clearance);

    param_loader.loadParam("viz/scale/points", _scale_points_);
    param_loader.loadParam("viz/scale/lines",  _scale_lines_);

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[PathPlanningNodelet]: Could not load all parameters");
      ros::shutdown();
    }

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = "PathPlanningNodelet";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 1;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", ros::Duration(5.0),
                                                                    &PathPlanningNodelet::timeoutOctomap, this,
                                                                    &PathPlanningNodelet::callbackOctomap, this);
    sh_tracker_cmd_          = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in", ros::Duration(3.0), &PathPlanningNodelet::timeoutTrackerCmd, this);
    sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "diagnostics_in", &PathPlanningNodelet::controlManagerDiagCallback, this);
    sh_frontiers_            = mrs_lib::SubscribeHandler<frontier_detection::FrontierArray>(shopts, "frontiers_in", &PathPlanningNodelet::callbackFrontiers, this);

    service_server_find_simplified_path_ = nh_.advertiseService("find_simplified_path_in", &PathPlanningNodelet::callbackFindSimplifiedPath, this);

    transformer_ = std::make_unique<mrs_lib::Transformer>("PathPlanningNodelet");
    transformer_->setDefaultPrefix(_uav_name_);
    transformer_->retryLookupNewest(true);

    bv_roadmap_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_roadmap", "");
    bv_roadmap_->setPointsScale(_scale_points_);
    bv_roadmap_->setLinesScale(_scale_lines_);

    flight_zone_ = {.min=octomap::point3d(-_flight_zone_width_x_/2.0, -_flight_zone_width_y_/2.0, _flight_zone_floor_),
                    .max=octomap::point3d( _flight_zone_width_x_/2.0,  _flight_zone_width_y_/2.0, _flight_zone_floor_+_flight_zone_height_)};

    path_planner_ = std::make_unique<PathPlanner>(flight_zone_, _roadmap_params_);

    timer_update_ = nh_.createTimer(ros::Rate(_rate_update_), &PathPlanningNodelet::timerUpdate, this);

    map_ready_ = false;

    vis_thread_          = std::thread(&PathPlanningNodelet::visualizationWorker, this);
    tree_cleanup_thread_ = std::thread(&PathPlanningNodelet::treeCleanupWorker, this);

    is_initialized_ = true;
    ROS_INFO("[PathPlanningNodelet]: initialized!");
  }

  PathPlanningNodelet::~PathPlanningNodelet()
  {
    {
      std::scoped_lock lock(vis_mutex_);
      vis_shutdown_ = true;
    }
    vis_cv_.notify_all();
    if (vis_thread_.joinable()) {
      vis_thread_.join();
    }

    {
      std::scoped_lock lock(tree_cleanup_mutex_);
      tree_cleanup_shutdown_ = true;
    }
    tree_cleanup_cv_.notify_all();
    if (tree_cleanup_thread_.joinable()) {
      tree_cleanup_thread_.join();
    }
  }

  // octomap subscription callback: merges the incoming local-zone map into the persistent global
  // octree_ (seeding it on the first message), the same local-to-global fusion frontier_detection
  // uses -- unlike prm_solver, which replaces its tree wholesale from an already-global map topic
  void PathPlanningNodelet::callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    // ROS_INFO_THROTTLE(2.0, "[PathPlanningNodelet]: octomap recieved");
    ros::WallTime t_start = ros::WallTime::now();
    ros::WallTime t_start_g = ros::WallTime::now();

    OcTreeUniquePtr_t local_tree(dynamic_cast<OcTree_t*>(octomap_msgs::msgToMap(*msg)));

    if (!local_tree) {
      ROS_WARN_THROTTLE(1.0, "[PathPlanningNodelet]: received map is empty!");
      return;
    }

    // expand first so the leaf iteration below (used both as the roadmap's OctomapDiff and, in the
    // merge branch, as mergeInto's source) sees every individual voxel rather than pruned inner nodes
    local_tree->expand();

    // restrict the diff to a small zone around the UAV -- voxels outside it don't affect the
    // roadmap the drone can currently reach, same rationale frontier_detection uses to limit
    // its own per-map work to a local zone
    OctomapDiff diff;
    {
      std::scoped_lock lock(mutex_octree_);

      auto pos_res = octomap_planner_utils::getPosition(sh_control_manager_diag_, sh_tracker_cmd_, msg->header.frame_id, *transformer_, "[PathPlanningNodelet]");
      if (!pos_res) {
        ROS_WARN_THROTTLE(1.0, "[PathPlanningNodelet]: has no reference, skipping roadmap diff");
      }
      else {
        const auto& pos = pos_res.value().reference.position;
        const octomap::point3d       uav_coord(pos.x, pos.y, pos.z);
        const octomap_planner_utils::AABB local_zone = octomap_planner_utils::localZoneFromPosition(uav_coord, flight_zone_, _local_zone_width_, _local_zone_height_);

        diff.reserve(local_tree->size());
        for (auto it = local_tree->begin_leafs_bbx(local_zone.min, local_zone.max), end = local_tree->end_leafs_bbx(); it != end; ++it) {
          const VoxelState new_state = local_tree->isNodeOccupied(*it) ? VoxelState::OCCUPIED : VoxelState::FREE;
          diff.push_back({it.getCoordinate(), new_state});
        }
      }
    }

    ROS_INFO("[PathPlanningNodelet]: tree diff   %4.3fms,",  1000*(ros::WallTime::now() - t_start).toSec());
    t_start = ros::WallTime::now();

    {
      std::scoped_lock lock(mutex_octree_);

      if (!octree_) {
        // seed the persistent global tree from the first local map received
        octree_ = std::move(local_tree);
      }
      else {
        octomap_planner_utils::mergeInto(*local_tree, *octree_);
      }
      octree_frame_ = msg->header.frame_id;
    }
    retireTree(std::move(local_tree));

    map_ready_ = true;

    ROS_INFO("[PathPlanningNodelet]: tree update %4.3fms,",  1000*(ros::WallTime::now() - t_start).toSec());
    t_start = ros::WallTime::now();

    if (!diff.empty()) {
      std::scoped_lock lock(mutex_octree_, mutex_roadmap_);
      path_planner_->updateRoadmap(*octree_, diff);
    }

    ROS_INFO("[PathPlanningNodelet]: RM update   %4.3fms,",  1000*(ros::WallTime::now() - t_start).toSec());

    enqueueVisualization();
    
    ROS_INFO("[PathPlanningNodelet]: tree done   %4.3fms,",  1000*(ros::WallTime::now() - t_start_g).toSec());
    ROS_INFO("[PathPlanningNodelet]: --------------------------");
    // t_start = ros::WallTime::now();

    // ROS_INFO_THROTTLE(2.0, "[PathPlanningNodelet]: octomap processed %4.3fms,",  1000*(ros::WallTime::now() - t_start).toSec());
  }

  // TODO: drive path-planning map maintenance from here once implemented
  void PathPlanningNodelet::timerUpdate([[maybe_unused]] const ros::TimerEvent& evt)
  {
    if (!is_initialized_ || !map_ready_) {
      return;
    }
  }

  // TODO: wire frontier viewpoints into whatever replaces prm_solver's roadmap
  void PathPlanningNodelet::callbackFrontiers(const frontier_detection::FrontierArray::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    ROS_INFO_THROTTLE(2.0, "[PathPlanningNodelet]: frontiers recieved");
  }

  // ~find_simplified_path_in service handler -- same request/response shape as prm_solver's
  // find_simplified_path: snaps req.start/req.goal onto their nearest roadmap nodes (via
  // PathPlanner::findNearestNode), runs PathPlanner::shortestPath between them, and shortcuts the
  // resulting node-position path via PathPlanner::simplifyPath before returning it
  bool PathPlanningNodelet::callbackFindSimplifiedPath(path_planning::FindSimplifiedPath::Request&  req,
                                                        path_planning::FindSimplifiedPath::Response& res)
  {
    if (!is_initialized_) {
      res.success = false;
      return true;
    }

    ros::WallTime t_start = ros::WallTime::now();
    ros::WallTime t_start_g = ros::WallTime::now();

    ROS_INFO("[PathPlanningNodelet]: callbackFindSimplifiedPath recieved");

    const octomap::point3d start(req.start.x, req.start.y, req.start.z);
    const octomap::point3d goal(req.goal.x, req.goal.y, req.goal.z);

    std::scoped_lock lock(mutex_octree_, mutex_roadmap_);

    const std::optional<NodeId> start_id = path_planner_->findNearestNode(start);
    const std::optional<NodeId> goal_id  = path_planner_->findNearestNode(goal);

    if (!start_id || !goal_id) {
      ROS_WARN_THROTTLE(1.0, "[PathPlanningNodelet]: roadmap has no nodes to search from");
      res.success = false;
      return true;
    }

    ROS_INFO("[PathPlanningNodelet]: preprocess %4.3fms,",  1000*(ros::WallTime::now() - t_start).toSec());
    t_start = ros::WallTime::now();

    const std::optional<PathResult> result = path_planner_->shortestPath(*start_id, *goal_id);
    if (!result) {
      ROS_WARN_THROTTLE(1.0, "[PathPlanningNodelet]: no path found between start and goal");
      res.success = false;
      return true;
    }

    ROS_INFO("[PathPlanningNodelet]: serach     %4.3fms,",  1000*(ros::WallTime::now() - t_start).toSec());
    t_start = ros::WallTime::now();

    const auto& nodes = path_planner_->nodes();
    std::vector<octomap::point3d> raw_path;
    raw_path.reserve(result->path.size());
    for (const NodeId id : result->path) {
      raw_path.push_back(nodes.at(id).position);
    }

    const std::vector<octomap::point3d> simplified_path = octree_ ? path_planner_->simplifyPath(*octree_, raw_path) : raw_path;

    ROS_INFO("[PathPlanningNodelet]: simplify   %4.3fms,",  1000*(ros::WallTime::now() - t_start).toSec());
    // t_start = ros::WallTime::now();

    for (const octomap::point3d& p : simplified_path) {
      geometry_msgs::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      res.path.push_back(point);
    }

    res.success = res.path.size() > 1;
    // ROS_INFO("[PathPlanningNodelet]: callbackFindSimplifiedPath processed");

    ROS_INFO("[PathPlanningNodelet]: clbckFSP   %4.3fms,",  1000*(ros::WallTime::now() - t_start_g).toSec());
    ROS_INFO("[PathPlanningNodelet]: ===========================");
    // t_start = ros::WallTime::now();
    return true;
  }

  void PathPlanningNodelet::controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg)
  {
    // only used for freshness checks via the SubscribeHandler, see getPosition()
  }

  // SubscribeHandler timeout callback for octomap_in: logs a throttled warning if a message was previously received but has since gone stale
  void PathPlanningNodelet::timeoutOctomap(const std::string& topic, const ros::Time& last_msg)
  {
    if (!is_initialized_ || !sh_octomap_.hasMsg()) {
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[PathPlanningNodelet]: octomap timeout!");
  }

  // SubscribeHandler timeout callback for tracker_cmd_in: logs a throttled warning if a message was previously received but has since gone stale
  void PathPlanningNodelet::timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg)
  {
    if (!is_initialized_ || !sh_tracker_cmd_.hasMsg()) {
      return;
    }
    ROS_WARN_THROTTLE(1.0, "[PathPlanningNodelet]: position cmd timeouted!");
  }

  // returns the UAV's current reference position transformed into the octree's frame, or nullopt if unavailable
  std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void>>> PathPlanningNodelet::getPosition()
  {
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    return octomap_planner_utils::getPosition(sh_control_manager_diag_, sh_tracker_cmd_, octree_frame, *transformer_, "[PathPlanningNodelet]");
  }

  void PathPlanningNodelet::enqueueVisualization()
  {
    RoadmapVisSnapshot snapshot;
    snapshot.frame_id = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);

    {
      std::scoped_lock lock(mutex_roadmap_);
      const auto& nodes = path_planner_->nodes();
      snapshot.nodes.reserve(nodes.size());
      for (const auto& [id, node] : nodes) {
        snapshot.nodes.push_back(node.position);
        for (const RoadmapEdge& edge : node.neighbors) {
          if (id >= edge.neighbor_id) {
            continue; // each undirected edge is stored on both endpoints -- draw it once, from the lower id
          }
          const auto nbr_it = nodes.find(edge.neighbor_id);
          if (nbr_it != nodes.end()) {
            snapshot.edges.emplace_back(node.position, nbr_it->second.position);
          }
        }
      }
    }

    {
      std::scoped_lock lock(vis_mutex_);
      pending_vis_snapshot_ = std::move(snapshot);
    }
    vis_cv_.notify_one();
  }

  // runs on its own thread for the lifetime of the nodelet: waits for a fresh snapshot, then does all the (comparatively
  // expensive) BatchVisualizer marker construction + publish/clearBuffers, keeping it off the octomap-callback thread.
  // bv_roadmap_/bv_map_frame_set_ are only ever touched from this thread, so no locking is needed around them here.
  void PathPlanningNodelet::visualizationWorker()
  {
    while (true)
    {
      RoadmapVisSnapshot snapshot;
      {
        std::unique_lock lock(vis_mutex_);
        vis_cv_.wait(lock, [this] { return vis_shutdown_ || pending_vis_snapshot_.has_value(); });
        if (vis_shutdown_) {
          return;
        }
        snapshot = std::move(pending_vis_snapshot_.value());
        pending_vis_snapshot_.reset();
      }

      if (!bv_map_frame_set_) {
        bv_roadmap_->setParentFrame(snapshot.frame_id);
        bv_map_frame_set_ = true;
      }

      for (const octomap::point3d& p : snapshot.nodes) {
        bv_roadmap_->addPoint(Eigen::Vector3d(p.x(), p.y(), p.z()), 0.1, 0.8, 0.3, 1.0);
      }

      for (const auto& [a, b] : snapshot.edges) {
        const mrs_lib::geometry::Ray ray = mrs_lib::geometry::Ray::twopointCast(Eigen::Vector3d(a.x(), a.y(), a.z()), Eigen::Vector3d(b.x(), b.y(), b.z()));
        bv_roadmap_->addRay(ray, 0.9, 0.9, 0.2, 0.1);
      }

      bv_roadmap_->publish();
      bv_roadmap_->clearBuffers();
    }
  }

  void PathPlanningNodelet::retireTree(OcTreeUniquePtr_t tree)
  {
    if (!tree) {
      return;
    }
    {
      std::scoped_lock lock(tree_cleanup_mutex_);
      pending_tree_cleanup_.push_back(std::move(tree));
    }
    tree_cleanup_cv_.notify_one();
  }

  // runs on its own thread for the lifetime of the nodelet: destroys retired octrees here so the (potentially expensive,
  // recursive) node-by-node teardown of a large OcTree never happens on the octomap-callback thread
  void PathPlanningNodelet::treeCleanupWorker()
  {
    while (true)
    {
      std::vector<OcTreeUniquePtr_t> trees;
      {
        std::unique_lock lock(tree_cleanup_mutex_);
        tree_cleanup_cv_.wait(lock, [this] { return tree_cleanup_shutdown_ || !pending_tree_cleanup_.empty(); });
        if (tree_cleanup_shutdown_) {
          return;
        }
        trees = std::move(pending_tree_cleanup_);
      }
      // trees actually destroyed here, outside the lock and off the octomap-callback thread
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(path_planning::PathPlanningNodelet,
                       nodelet::Nodelet)
