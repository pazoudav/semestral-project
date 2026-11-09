#include <explorer/explorer.hpp>

namespace explorer
{

  // (re)plans path_ unconditionally (all gating happens in timerReplanner()): getCurrentStateAndPrediction() -> compensateLatency() moves the start
  // ahead along the MPC prediction -> resets goal_/current_viewpoint_ to that start -> makePathPrecomputed() (if precomputed/enabled; returns on
  // success, falls through to frontier exploration otherwise) -> solveTsp() for a fresh global viewpoint tour -> visualizeGlobalPath()
  // -> refineLocalTour() (re-picks the viewpoints of the nearest tour clusters) -> buildLocalPath() (path_planner sub-paths to the next
  // few tour viewpoints) -> finalizePath(); returns true iff path_ was updated
  bool Explorer::makePath()
  {
    // ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: starting path (re)plannig");

    // only finalizePath() sets it again, so it always reflects how the latest replan ended
    no_suitable_frontiers_ = false;

    octomap::point3d start_coord;
    mrs_msgs::MpcPredictionFullState prediction;
    octomath::Vector3 velocity;
    if (!getCurrentStateAndPrediction(start_coord, prediction, velocity))
    {
      ROS_WARN("[MrsExplorer-Replanner]: cannot get state and prediction");
      return false;
    }

    // phase timing, logged indented beneath pathAndTrajectory()'s makePath line
    ros::WallTime t_checkpoint = ros::WallTime::now();

    // the UAV keeps flying the old trajectory while this replan runs, so plan from where it will be by the time the new one takes over
    compensateLatency(prediction, start_coord, velocity);

    // periodic advance replan while following the precomputed path: only re-plan to the committed entry if its anchor moved on, otherwise
    // keep the current trajectory (and goal_/next_goal_ untouched); never falls back to frontier exploration
    if (advance_replan_)
    {
      const bool adv_succ = makePathPrecomputed(start_coord, velocity, true) == PRE_PATH;
      ROS_INFO("[MrsExplorer]:   precomputed advance %8.3fms", timing_->publishSince("makePath/precomputed_advance", t_checkpoint));
      return adv_succ;
    }

    // ROS_WARN("[MrsExplorer]: replanning");
    goal_ = start_coord;
    current_viewpoint_ = start_coord;

    // precomputed coverage path first (once skeleton_estimator's estimate is stable); no TSP involved
    if (_pre_enabled_)
    {
      const PrecomputedResult_t pre_res = makePathPrecomputed(start_coord, velocity, false);
      ROS_INFO("[MrsExplorer]:   precomputed    %8.3fms", timing_->publishSince("makePath/precomputed", t_checkpoint));
      t_checkpoint = ros::WallTime::now();
      if (pre_res == PRE_PATH)
      {
        following_precomputed_ = true;
        return true;
      }
      // waiting at a look-up point for the map to show the vertical path line: no new path, and no frontier fallback either
      if (pre_res == PRE_HOLD)
      {
        return false;
      }
    }

    std::vector<octomap::point3d> glob_path;
    std::vector<unsigned long>    tour_ids;
    if (!solveTsp(velocity, glob_path, tour_ids))
    {
      ROS_ERROR("[MrsExplorer]: TSP solve failed");
      return false;
    }

    visualizeGlobalPath(glob_path);
    ROS_INFO("[MrsExplorer]:   tsp            %8.3fms", timing_->publishSince("makePath/tsp", t_checkpoint));
    t_checkpoint = ros::WallTime::now();

    if (_refine_enabled_)
    {
      refineLocalTour(start_coord, velocity, glob_path, tour_ids);
    }
    ROS_INFO("[MrsExplorer]:   refine         %8.3fms", timing_->publishSince("makePath/refine", t_checkpoint));
    t_checkpoint = ros::WallTime::now();

    std::vector<octomap::point3d> path(0);
    std::vector<octomap::point3d> sub_global_path(0);
    if (!buildLocalPath(start_coord, glob_path, velocity, path, sub_global_path))
    {
      ROS_ERROR("[MrsExplorer]: local path build failed, replan local path");
      return false;
    }
    ROS_INFO("[MrsExplorer]:   local path     %8.3fms", timing_->publishSince("makePath/local_path", t_checkpoint));

    bool succ = finalizePath(path, sub_global_path);
    if (!succ){
      ROS_ERROR("[MrsExplorer]: finalize path build failed");
      return false;
    }
    following_precomputed_ = false;
    return true;
  }

   // calls tsp_solver's ~set_start_out (start = current_viewpoint_) and ~solve_out (with velocity) services to obtain a fresh global viewpoint tour
  // into glob_path (and the parallel frontier ids into tour_ids); returns false if either call fails or the tour has <= 1 point
  bool Explorer::solveTsp(const octomath::Vector3& velocity, std::vector<octomap::point3d>& glob_path, std::vector<unsigned long>& tour_ids)
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
    tour_ids.assign(solve_srv.response.ids.begin(), solve_srv.response.ids.end());
    // ROS_INFO("[MrsExplorer]: TSP solve end");

    if (glob_path.size() <= 1)
    {
      ROS_WARN("[MrsExplorer]: TSP tour not found");
      return false;
    }
    return true;
  }

  // walks glob_path from start_coord, skipping tour points closer than planning/skip_path_point_distance, collecting the visited tour points into
  // sub_global_path and planning a path_planner sub-path to each (current velocity only on the first segment); sub-path points are appended to
  // path (deduplicated by skip_path_point_distance). Stops once path extends planning/max_length_after_first_viewpoint of arc length past the
  // first tour viewpoint (the overshooting edge is cut exactly there, keeps the trajectory generation request short), or at the tour's end.
  // On a planner failure returns false, except after kMaxTempPathFailures failures in a row, when it instead returns a 1 m step in a random
  // direction (randomStepPath()) to break the deadlock
  bool Explorer::buildLocalPath(const octomap::point3d& start_coord, const std::vector<octomap::point3d>& glob_path, const octomath::Vector3& velocity,
                                std::vector<octomap::point3d>& path, std::vector<octomap::point3d>& sub_global_path)
  {
    sub_global_path.assign({start_coord});
    path.assign({start_coord});

    const double max_tail_length      = _max_length_after_first_viewpoint_;
    double       tail_length          = 0.0;    // arc length of path past the first tour viewpoint
    bool         past_first_viewpoint = false;  // true once the first segment (start -> first tour viewpoint) is appended
    size_t       i                    = 0;      // index of the next candidate tour point in glob_path

    while (tail_length < max_tail_length)
    {
      // advance to the next tour point far enough from the last accepted one; stop once the tour has no further distinct point
      while (i + 1 < glob_path.size() && sub_global_path.back().distance(glob_path[i]) < _skip_path_point_distance_)
      {
        i++;
      }
      if (i + 1 >= glob_path.size() && sub_global_path.back().distance(glob_path[i]) < 1e-6)
      {
        break;
      }

      const octomath::Vector3 seg_velocity = past_first_viewpoint ? octomath::Vector3(0.0f, 0.0f, 0.0f) : velocity;
      std::vector<octomap::point3d> segment;
      if (!findSimplifiedPath(sub_global_path.back(), glob_path[i], seg_velocity, segment))
      {
        ROS_ERROR("[MrsExplorer]: temp path not found");
        if (++temp_path_fail_count_ < kMaxTempPathFailures)
        {
          return false;
        }
        ROS_ERROR("[MrsExplorer]: temp path not found %d times in a row, flying 1 m in a random direction", temp_path_fail_count_);
        temp_path_fail_count_ = 0;
        randomStepPath(start_coord, path, sub_global_path);
        return true;
      }
      sub_global_path.push_back(glob_path[i]);

      for (const auto& p : segment)
      {
        const double edge = p.distance(path.back());
        if (edge <= _skip_path_point_distance_)
        {
          continue;
        }
        if (past_first_viewpoint && tail_length + edge >= max_tail_length)
        {
          // cut the overshooting edge exactly at the cap
          path.push_back(path.back() + (p - path.back()) * static_cast<float>((max_tail_length - tail_length) / edge));
          tail_length = max_tail_length;
          break;
        }
        path.push_back(p);
        if (past_first_viewpoint)
        {
          tail_length += edge;
        }
      }
      past_first_viewpoint = true;
    }

    temp_path_fail_count_ = 0;
    return true;
  }

  // calls path_planner's ~find_simplified_path_out from start to goal (planned with the given initial velocity) and returns its points in segment;
  // false if the call or the planner fails
  bool Explorer::findSimplifiedPath(const octomap::point3d& start, const octomap::point3d& goal, const octomath::Vector3& velocity,
                                    std::vector<octomap::point3d>& segment)
  {
    path_planner::FindSimplifiedPath find_path_srv;
    find_path_srv.request.start.x = start.x();
    find_path_srv.request.start.y = start.y();
    find_path_srv.request.start.z = start.z();
    geometry_msgs::Point goal_point;
    goal_point.x = goal.x();
    goal_point.y = goal.y();
    goal_point.z = goal.z();
    find_path_srv.request.goals.push_back(goal_point);
    find_path_srv.request.velocity.x = velocity.x();
    find_path_srv.request.velocity.y = velocity.y();
    find_path_srv.request.velocity.z = velocity.z();

    if (!sc_path_planner_find_simplified_path_.call(find_path_srv) || !find_path_srv.response.success[0])
    {
      return false;
    }

    segment.clear();
    for (const auto& p : find_path_srv.response.paths[0].points)
    {
      segment.emplace_back(p.x, p.y, p.z);
    }
    return true;
  }

  // fail-safe for buildLocalPath(): sets path and sub_global_path to a 1 m step from start_coord in a uniformly random direction
  void Explorer::randomStepPath(const octomap::point3d& start_coord, std::vector<octomap::point3d>& path, std::vector<octomap::point3d>& sub_global_path)
  {
    std::normal_distribution<float> gauss(0.0f, 1.0f);
    octomath::Vector3 dir(gauss(rng_), gauss(rng_), gauss(rng_));
    if (dir.norm() < 1e-6f)
    {
      dir = octomath::Vector3(1.0f, 0.0f, 0.0f);
    }
    dir.normalize();

    const octomap::point3d random_point = start_coord + dir;
    path.assign({start_coord, random_point});
    sub_global_path.assign({start_coord, random_point});
  }

  // trims the leading (already-occupied) start point and commits path into path_ (new id); sets goal_ to the first tour viewpoint (sub_global_path[1])
  // and next_goal_ to the path's end, which goalWithinLatency() then watches; returns false if nothing is left after trimming, which also
  // sets no_suitable_frontiers_ for timerReplanner()'s end-of-exploration check (makePath() clears it at the start of every replan)
  bool Explorer::finalizePath(std::vector<octomap::point3d>& path, const std::vector<octomap::point3d>& sub_global_path)
  {
    path.erase(path.begin());
    path_.points = path;
    path_.headings.clear();
    path_.id     = next_path_id_++;

    if (path_.points.size() > 0)
    {
      goal_ = sub_global_path[1]; // path_.points.back();
      next_goal_ = path.back();
      // first_path_planend_ = true;
      return true;
    }

    ROS_WARN("[MRsExplorer] no suitable frontiers");
    no_suitable_frontiers_ = true;
    return false;
  }  

}  // namespace explorer
