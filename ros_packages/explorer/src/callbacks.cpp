#include <explorer/explorer.hpp>
#include <octomap_msgs/conversions.h>

namespace explorer
{

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

  // stores skeleton_estimator's latest precomputed path (latched topic; republished on every live skeleton with the current transform) for
  // makePathPrecomputed(); followed only while msg.stable and a path is selected. A changed path_id invalidates the waypoint indices, so the
  // following state (committed entry, no-progress counts) is reset on the next replan
  void Explorer::callbackPrecomputedPath(const skeleton_estimator::PrecomputedPath::ConstPtr msg)
  {
    if (!is_initialized_) {
      return;
    }

    std::vector<PrecomputedWaypoint_t> wps;
    wps.reserve(msg->waypoints.size());
    for (const auto& w : msg->waypoints)
    {
      wps.push_back({explorer_utils::pointToOctomap(w.position), w.yaw, static_cast<bool>(w.is_viewpoint)});
    }

    std::scoped_lock lock(mutex_precomputed_);
    if (msg->path_id != precomputed_path_id_)
    {
      ROS_INFO("[MrsExplorer]: precomputed path changed %d -> %d", precomputed_path_id_, msg->path_id);
      precomputed_reset_ = true;
    }
    precomputed_path_id_ = msg->path_id;
    precomputed_stable_  = msg->stable && msg->path_id >= 0 && !wps.empty();
    precomputed_frame_   = msg->header.frame_id;
    precomputed_path_    = std::move(wps);
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

}  // namespace explorer
