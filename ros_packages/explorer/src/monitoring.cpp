#include <explorer/explorer.hpp>

namespace explorer
{

  // gets current position and MPC full-state prediction, deriving start_coord/prediction/velocity (velocity = first prediction sample);
  // returns false (with a throttled warning) if either is unavailable; used by both timerReplanner() and makePath()
  bool Explorer::getCurrentStateAndPrediction(octomap::point3d& start_coord, mrs_msgs::MpcPredictionFullState& prediction, octomath::Vector3& velocity)
  {
    // get current position
    auto res_position = getPosition();
    if (!res_position){
      ROS_WARN_THROTTLE(1.0,"[MRsExplorer] has no reference");
      return false;
    }
    start_coord = explorer_utils::pointToOctomap(res_position.value().reference.position);

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

  // advances start_coord/velocity along the MPC prediction by 2x latency_estimate_ seconds (measured from the prediction's first
  // stamp; same horizon goalWithinLatency() triggers the replan at), stopping early at the last sample still in free space (a collision-triggered replan has a prediction running into an obstacle);
  // leaves them untouched if compensation is disabled or the prediction is malformed
  void Explorer::compensateLatency(const mrs_msgs::MpcPredictionFullState& prediction, octomap::point3d& start_coord, octomath::Vector3& velocity)
  {
    const std::size_t n = prediction.position.size();
    if (_latency_compensation_ <= 0.0 || n == 0 || prediction.stamps.size() != n || prediction.velocity.size() != n)
    {
      return;
    }

    std::scoped_lock lock(mutex_octree_);
    if (!octree_)
    {
      return;
    }

    const ros::Time t0      = prediction.stamps[0];
    const double    t_ahead = 2.0 * latency_estimate_;
    // precomputed FOLLOW flies through unknown space, so only occupied voxels stop the compensation (radius = half of
    // flight_free_distance, which isFreeSpace() takes as a diameter)
    const bool occupied_only = occupiedOnlyChecks();
    for (std::size_t k = 1; k < n; k++)
    {
      const octomap::point3d p(prediction.position[k].x, prediction.position[k].y, prediction.position[k].z);
      if (occupied_only ? occupiedNear(*octree_, p, _flight_free_distance_ / 2.0) : !explorer_utils::isFreeSpace(p, _flight_free_distance_, *octree_))
      {
        break;
      }
      start_coord = p;
      velocity    = octomath::Vector3(prediction.velocity[k].x, prediction.velocity[k].y, prediction.velocity[k].z);
      if ((prediction.stamps[k] - t0).toSec() >= t_ahead)
      {
        break;
      }
    }
  }

  // returns true once the earlier of goal_ (first tour viewpoint) / next_goal_ (end of path_) is reached within 2x latency_estimate_ (plus one
  // timer_rates/fast tick) along the MPC prediction, so the new trajectory takes over right at the goal; "reached" = the prediction sample closest
  // to the goal over the full horizon (a goal beyond the horizon puts that at the last sample, i.e. not yet due)
  bool Explorer::goalWithinLatency(const mrs_msgs::MpcPredictionFullState& prediction)
  {
    ros::WallTime t_start = ros::WallTime::now();

    const std::size_t n = prediction.position.size();
    if (n == 0 || prediction.stamps.size() != n)
    {
      return false;
    }

    // time from the prediction's first stamp to its sample closest to goal
    auto time_to_goal = [&](const octomap::point3d& goal) {
      double min_dist = std::numeric_limits<double>::infinity();
      double t_min    = 0.0;
      for (std::size_t k = 0; k < n; k++)
      {
        const double d = goal.distance(octomap::point3d(prediction.position[k].x, prediction.position[k].y, prediction.position[k].z));
        if (d < min_dist)
        {
          min_dist = d;
          t_min    = (prediction.stamps[k] - prediction.stamps[0]).toSec();
        }
      }
      return t_min;
    };

    const double t_goal  = std::min(time_to_goal(goal_), time_to_goal(next_goal_));
    const double t_ahead = 2.0 * latency_estimate_ + 1.0 / _rate_fast_timer_;
    const double calc_ms = timing_->publishSince("goalWithinLatency/total", t_start);

    if (t_goal > t_ahead)
    {
      ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: not planning, goal in %.2fs (replan at %.2fs), computed in %.3fms", t_goal, t_ahead, calc_ms);
      return false;
    }
    return true;
  }

  // // returns true once the UAV is within planning/replanning_distance of goal_ (first tour viewpoint) or next_goal_ (end of path_), i.e. a replan is due;
  // // before the first successful plan, goal_/next_goal_ are pinned to the current position so this returns true immediately
  // bool Explorer::nearGoal(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction)
  // {
  //   if (!first_path_planend_){
  //     goal_ = start_coord;
  //     next_goal_ = goal_;
  //     // ros::Duration(5.0).sleep();
  //   }
  //   geometry_msgs::Point p = prediction.position.back();
  //   if (start_coord.distance(goal_) > _replanning_distance_ && start_coord.distance(next_goal_) > _replanning_distance_) // octomap::point3d(p.x,p.y,p.z)
  //   {
  //     ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: not planning, too far from goal %f", start_coord.distance(octomap::point3d(p.x,p.y,p.z)));
  //     return false;
  //   }
  //   return true;
  // }

  // returns true once the UAV has moved less than planning/stuck_distance for planning/stuck_duration seconds straight (timerReplanner() then forces
  // STATE_IDLE); tracker is (re)armed by stuck_check_reset_, which requestReplan() sets whenever a new trajectory execution actually starts
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

  // returns true if any position sample of the MPC prediction lies within planning/flight_free_distance of non-free space in octree_
  // (called by timerReplanner() each tick in STATE_TRAJ_EXEC to trigger a replan); start_coord is unused.
  // In precomputed FOLLOW (occupiedOnlyChecks()) only occupied voxels count: the precomputed path is flown through unknown space
  bool Explorer::checkTrajectoryCollision(const octomap::point3d& start_coord, const mrs_msgs::MpcPredictionFullState& prediction)
  {
    // check if current path from predition is in collision
    bool isFreeSpace = true;
    // held for the whole check: octree_ is merged into in place by callbackOctomap
    std::scoped_lock lock(mutex_octree_);
    const OcTree_t* tree = octree_.get();
    if (!tree)
    {
      return false;
    }
    const bool occupied_only = occupiedOnlyChecks();
    for (auto & point : prediction.position)
    {
      const octomap::point3d p(point.x, point.y, point.z);
      isFreeSpace = occupied_only ? !occupiedNear(*tree, p, _flight_free_distance_ / 2.0) : explorer_utils::isFreeSpace(p, _flight_free_distance_, *tree);
      if (!isFreeSpace)
      {
        // goal_ = start_coord;
        // ROS_WARN("[MrsExplorer]: collision detected in trajectory, replanning");
        break;
      }
    }
    return !isFreeSpace;
  }

  // tracker's full-state MPC prediction (in the octree frame), delegating to explorer_utils::getFullStatePrediction using the tracker_cmd/diagnostics subscribers
  std::optional<mrs_msgs::MpcPredictionFullState> Explorer::getFullStatePrediction()
  {
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    return explorer_utils::getFullStatePrediction(sh_control_manager_diag_, sh_tracker_cmd_, octree_frame, *transformer_, "[MrsExplorer]");
  }

  // current UAV position (in the octree frame), delegating to explorer_utils::getPosition using the tracker_cmd/diagnostics subscribers
  std::optional<mrs_msgs::ReferenceStamped_<std::allocator<void> >> Explorer::getPosition()
  {
    auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    return explorer_utils::getPosition(sh_control_manager_diag_, sh_tracker_cmd_, octree_frame, *transformer_, "[MrsExplorer]");
  }

}  // namespace explorer
