#include <explorer/explorer.hpp>

#include <unordered_map>

// precomputed coverage path following, v2 (GOTO/FOLLOW state machine); the previous anchor-chasing version (path_planner to every anchor,
// look-up-point zigzag on vertical stretches) is kept for reference in backup/precomputed_v1/ (not compiled)

namespace explorer
{

  namespace
  {
    enum : uint8_t { V_FREE, V_UNKNOWN, V_OCCUPIED };
    using VoxelCache_t = std::unordered_map<octomap::OcTreeKey, uint8_t, octomap::OcTreeKey::KeyHash>;

    // FREE/UNKNOWN/OCCUPIED of the voxel at p (outside the tree's key range = UNKNOWN), memoized in cache
    uint8_t voxelState(const octomap::OcTree& tree, const octomap::point3d& p, VoxelCache_t& cache)
    {
      octomap::OcTreeKey key;
      if (!tree.coordToKeyChecked(p, key))
      {
        return V_UNKNOWN;
      }
      const auto it = cache.find(key);
      if (it != cache.end())
      {
        return it->second;
      }
      const octomap::OcTreeNode* node = tree.search(key);
      const uint8_t              s    = !node ? V_UNKNOWN : (tree.isNodeOccupied(node) ? V_OCCUPIED : V_FREE);
      cache.emplace(key, s);
      return s;
    }

    // worst voxel state within radius r of c (path_planner's hasClearance sweep, stepped at the octree resolution): the center's own state if
    // it isn't free, else OCCUPIED as soon as one is found, else UNKNOWN if any, else FREE
    uint8_t clearanceState(const octomap::OcTree& tree, const octomap::point3d& c, double r, VoxelCache_t& cache)
    {
      const uint8_t center = voxelState(tree, c, cache);
      if (center != V_FREE)
      {
        return center;
      }
      const double step    = tree.getResolution();
      bool         unknown = false;
      for (double dx = -r; dx <= r; dx += step)
      {
        for (double dy = -r; dy <= r; dy += step)
        {
          for (double dz = -r; dz <= r; dz += step)
          {
            if (dx * dx + dy * dy + dz * dz > r * r)
            {
              continue;
            }
            const uint8_t s = voxelState(tree, octomap::point3d(c.x() + dx, c.y() + dy, c.z() + dz), cache);
            if (s == V_OCCUPIED)
            {
              return V_OCCUPIED;
            }
            unknown = unknown || s == V_UNKNOWN;
          }
        }
      }
      return unknown ? V_UNKNOWN : V_FREE;
    }

    // any occupied voxel within radius r of c (unknown counts as free), memoized in cache
    bool occupiedWithin(const octomap::OcTree& tree, const octomap::point3d& c, double r, VoxelCache_t& cache)
    {
      const double step = tree.getResolution();
      for (double dx = -r; dx <= r; dx += step)
      {
        for (double dy = -r; dy <= r; dy += step)
        {
          for (double dz = -r; dz <= r; dz += step)
          {
            if (dx * dx + dy * dy + dz * dz <= r * r && voxelState(tree, octomap::point3d(c.x() + dx, c.y() + dy, c.z() + dz), cache) == V_OCCUPIED)
            {
              return true;
            }
          }
        }
      }
      return false;
    }
  }  // namespace

  // occupied-only collision test used while in PRE_MODE_FOLLOW (checkTrajectoryCollision(), compensateLatency()): unknown space is flown
  // through, only occupied voxels within r of c count. Caller holds mutex_octree_
  bool Explorer::occupiedNear(const OcTree_t& tree, const octomap::point3d& c, double r)
  {
    VoxelCache_t cache;
    return occupiedWithin(tree, c, r, cache);
  }

  void Explorer::resetPrecomputedMode()
  {
    if (pre_mode_ != PRE_MODE_NONE)
    {
      ROS_INFO("[MrsExplorer]: precomputed mode -> NONE");
    }
    pre_mode_ = PRE_MODE_NONE;
    committed_entry_.reset();
  }

  void Explorer::enterFollow(double entry_u, double progress_u, int dir)
  {
    ROS_INFO("[MrsExplorer]: precomputed mode -> FOLLOW (joined at u=%.2f, dir %+d)", progress_u, dir);
    pre_mode_       = PRE_MODE_FOLLOW;
    pre_entry_u_    = entry_u;
    pre_progress_u_ = progress_u;
    pre_dir_        = dir;
    committed_entry_.reset();
  }

  // precomputed coverage path following ("path frontiers"), a small state machine on top of skeleton_estimator's (stable) precomputed path:
  // the path is resampled every precomputed/sample_step (resamplePath()) and every sample labelled KNOWN/UNKNOWN/OCCUPIED/OUTSIDE with
  // path_planner's own reachability test (labelSamples()); an entry is a KNOWN sample (anchor) whose next followed sample in a direction is
  // UNKNOWN, i.e. where the path crosses the known/unknown boundary.
  // - NONE/GOTO (planGoto()): pick an entry (the committed one first, then nearest) and fly to its anchor via path_planner (known space only).
  //   Once the UAV is within precomputed/join_distance of the anchor (or of the known stretch the anchor has since advanced along) -> FOLLOW
  // - FOLLOW (planFollow()): the trajectory is built straight from the path samples ahead of the UAV (precomputed/follow_horizon, no
  //   path_planner), cut only at samples with an occupied voxel within precomputed/follow_clearance (unknown space is flown through); the
  //   collision checks while flying it consider occupied voxels only (occupiedOnlyChecks()). Re-extended every precomputed/replan_period
  //   (advance_only). When the segment ahead has no more UNKNOWN samples (within precomputed/max_known_gap, before it is blocked or ends),
  //   the next start is selected -> GOTO
  // Entries are kept by path parameter u (waypoint index + segment fraction), immune to transform drift. A follow that ends without getting
  // anywhere counts as no progress for its entry (skipped at precomputed/max_no_progress until the path changes); a GOTO planning failure
  // backs the entry off for precomputed/failure_backoff.
  // Fills path_ and returns PRE_PATH on success; PRE_NONE -> frontier exploration (makePath()), or keep the current trajectory (advance_only)
  Explorer::PrecomputedResult_t Explorer::makePathPrecomputed(const octomap::point3d& start_coord, const octomath::Vector3& velocity, bool advance_only)
  {
    std::vector<PrecomputedWaypoint_t> wps;
    std::string                        frame;
    {
      std::scoped_lock lock(mutex_precomputed_);
      if (precomputed_reset_)
      {
        resetPrecomputedMode();
        entry_records_.clear();
        precomputed_reset_ = false;
      }
      if (!precomputed_stable_ || precomputed_path_.size() < 2)
      {
        precomputed_has_targets_ = false;
        resetPrecomputedMode();
        return PRE_NONE;
      }
      wps   = precomputed_path_;
      frame = precomputed_frame_;
    }

    auto strip_slash = [](const std::string& f) { return (!f.empty() && f[0] == '/') ? f.substr(1) : f; };
    const std::string octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    if (strip_slash(frame) != strip_slash(octree_frame))
    {
      ROS_WARN_THROTTLE(5.0, "[MrsExplorer]: precomputed path frame '%s' != octree frame '%s', not following it", frame.c_str(), octree_frame.c_str());
      precomputed_has_targets_ = false;
      resetPrecomputedMode();
      return PRE_NONE;
    }

    const std::vector<PathSample_t>    samples = resamplePath(wps);
    const std::vector<WaypointLabel_t> labels  = labelSamples(samples);

    // periodic replan: only extends the followed segment; anything else keeps the current trajectory (and the mode untouched)
    if (advance_only)
    {
      return (pre_mode_ == PRE_MODE_FOLLOW && planFollow(samples, labels, wps, start_coord, true) == FOLLOW_PATH) ? PRE_PATH : PRE_NONE;
    }

    if (pre_mode_ == PRE_MODE_FOLLOW)
    {
      if (planFollow(samples, labels, wps, start_coord, false) == FOLLOW_PATH)
      {
        return PRE_PATH;
      }
      resetPrecomputedMode();  // segment done -> find the next start
    }

    // at most twice: an entry the UAV is already at may turn out not to be followable (no_progress++), then the next one is tried
    for (int tries = 0; tries < 2; tries++)
    {
      const GotoResult_t g = planGoto(samples, labels, start_coord, velocity);
      if (g == GOTO_PATH)
      {
        return PRE_PATH;
      }
      if (g == GOTO_NONE)
      {
        break;
      }
      if (planFollow(samples, labels, wps, start_coord, false) == FOLLOW_PATH)
      {
        return PRE_PATH;
      }
      resetPrecomputedMode();
    }
    resetPrecomputedMode();  // frontier exploration this replan
    return PRE_NONE;
  }

  // GOTO: selects an entry (committed one first, the rest nearest-first, at most precomputed/max_entry_attempts) and plans start -> anchor
  // through path_planner (GOTO_PATH, mode GOTO). If the UAV is already within precomputed/join_distance of the anchor, or of the known
  // stretch between the committed anchor and where it has advanced to since, FOLLOW is entered instead (GOTO_FOLLOW, no path_ yet)
  Explorer::GotoResult_t Explorer::planGoto(const std::vector<PathSample_t>& samples, const std::vector<WaypointLabel_t>& labels,
                                             const octomap::point3d& start_coord, const octomath::Vector3& velocity)
  {
    const int       n   = static_cast<int>(samples.size());
    const ros::Time now = ros::Time::now();

    std::vector<PathEntry_t> entries = findEntries(labels);
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [&](const PathEntry_t& e) { return entryRecord(samples, e.first, e.second).no_progress >= _pre_max_no_progress_; }),
                  entries.end());
    // still has targets while an entry is only backing off, so exploration doesn't end over a transient failure
    precomputed_has_targets_ = !entries.empty();
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [&](const PathEntry_t& e) { return now < entryRecord(samples, e.first, e.second).retry_after; }),
                  entries.end());

    // advance the committed entry through the known stretch in its direction (the boundary moves on with the growing map); if the UAV has
    // reached any point of that stretch, it is on the path -> FOLLOW from there
    std::optional<PathEntry_t> committed;
    if (committed_entry_)
    {
      const auto [u, d]        = *committed_entry_;
      const int  committed_from = nearestSample(samples, u);
      if (committed_from >= 0 && labels[committed_from] == LABEL_KNOWN)
      {
        int    last_known = committed_from;
        int    closest    = -1;
        double closest_d  = _pre_join_distance_;
        for (int j = committed_from; j >= 0 && j < n && labels[j] != LABEL_UNKNOWN; j += d)
        {
          if (labels[j] == LABEL_KNOWN)
          {
            last_known = j;
          }
          const double dist = start_coord.distance(samples[j].position);
          if (dist <= closest_d)
          {
            closest_d = dist;
            closest   = j;
          }
        }
        if (closest >= 0)
        {
          enterFollow(u, samples[closest].u, d);
          return GOTO_FOLLOW;
        }
        const PathEntry_t e{last_known, d};
        if (std::find(entries.begin(), entries.end(), e) != entries.end())
        {
          committed = e;
        }
      }
    }

    if (entries.empty())
    {
      ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: precomputed path has no %sentry into unknown space, frontier exploration",
                        precomputed_has_targets_ ? "available (backing off) " : "");
      committed_entry_.reset();
      return GOTO_NONE;
    }

    std::sort(entries.begin(), entries.end(), [&](const PathEntry_t& e0, const PathEntry_t& e1) {
      return start_coord.distance(samples[e0.first].position) < start_coord.distance(samples[e1.first].position);
    });
    if (committed)
    {
      const auto it = std::find(entries.begin(), entries.end(), *committed);
      std::rotate(entries.begin(), it, it + 1);
    }

    const int attempts = std::min(static_cast<int>(entries.size()), _pre_max_entry_attempts_);
    for (int k = 0; k < attempts; k++)
    {
      const PathEntry_t&  e      = entries[k];
      const PathSample_t& anchor = samples[e.first];

      if (start_coord.distance(anchor.position) <= _pre_join_distance_)
      {
        enterFollow(anchor.u, anchor.u, e.second);
        return GOTO_FOLLOW;
      }

      std::vector<octomap::point3d> path;
      std::vector<double>           headings;
      if (!planEntry(start_coord, velocity, anchor.position, anchor.yaw, path, headings))
      {
        entryRecord(samples, e.first, e.second).retry_after = now + ros::Duration(_pre_failure_backoff_);
        ROS_WARN("[MrsExplorer]: precomputed entry at sample %d (dir %+d) could not be planned, retry in %.1fs", e.first, e.second, _pre_failure_backoff_);
        continue;
      }

      if (pre_mode_ != PRE_MODE_GOTO)
      {
        ROS_INFO("[MrsExplorer]: precomputed mode -> GOTO");
      }
      pre_mode_        = PRE_MODE_GOTO;
      committed_entry_ = std::make_pair(anchor.u, e.second);

      // same commit as finalizePath(): drop the start point, goal_ = next_goal_ = anchor (replan on approaching it -> FOLLOW)
      path.erase(path.begin());
      headings.erase(headings.begin());
      path_.points   = path;
      path_.headings = headings;
      path_.id       = next_path_id_++;
      goal_          = anchor.position;
      next_goal_     = path.back();
      ROS_INFO("[MrsExplorer]: precomputed GOTO entry at sample %d (waypoint ~%d, dir %+d), %lu points", e.first, anchor.wp, e.second, path.size());
      return GOTO_PATH;
    }

    committed_entry_.reset();
    ROS_WARN("[MrsExplorer]: no precomputed entry could be planned (%d tried), frontier exploration", attempts);
    return GOTO_NONE;
  }

  // FOLLOW: re-locates the UAV on the path near the last progress (nearest sample in a window around it; farther than
  // precomputed/off_path_distance = lost the path -> done), then walks the segment ahead in pre_dir_ until a sample is blocked (outside
  // flight_zone_, or an occupied voxel within precomputed/follow_clearance; unknown is fine) or the path ends. The samples within
  // precomputed/follow_horizon become path_ (with FC-Planner headings); the segment is done when there is no UNKNOWN sample on it within
  // precomputed/max_known_gap. advance_only: never changes the following state besides the progress
  Explorer::FollowResult_t Explorer::planFollow(const std::vector<PathSample_t>& samples, const std::vector<WaypointLabel_t>& labels,
                                                 const std::vector<PrecomputedWaypoint_t>& wps, const octomap::point3d& start_coord, bool advance_only)
  {
    const int n = static_cast<int>(samples.size());
    const int d = pre_dir_;

    // the UAV's position along the path: nearest sample, searched a bit behind and up to a horizon ahead of the last progress (not globally,
    // so a path passing close to itself doesn't make it jump)
    const int p0     = nearestSample(samples, pre_progress_u_);
    const int behind = static_cast<int>(std::ceil(_pre_off_path_distance_ / _pre_sample_step_)) + 1;
    const int ahead  = static_cast<int>(std::ceil((_pre_follow_horizon_ + _pre_off_path_distance_) / _pre_sample_step_)) + 1;
    int       progress = -1;
    double    best     = std::numeric_limits<double>::infinity();
    for (int k = -behind; k <= ahead; k++)
    {
      const int j = p0 + k * d;
      if (j < 0 || j >= n)
      {
        continue;
      }
      const double dist = start_coord.distance(samples[j].position);
      if (dist < best)
      {
        best     = dist;
        progress = j;
      }
    }
    if (progress < 0 || best > _pre_off_path_distance_)
    {
      ROS_WARN("[MrsExplorer]: precomputed FOLLOW lost the path (%.2fm from it), finding next start", best);
      return FOLLOW_DONE;
    }
    pre_progress_u_ = samples[progress].u;

    // walk the segment ahead
    std::vector<octomap::point3d> path{start_coord};
    std::vector<double>           headings{samples[progress].yaw};
    std::vector<int>              walked{progress};
    bool                          has_unknown = false;
    double                        length      = 0.0;
    const char*                   end_reason  = "path end";
    {
      std::scoped_lock lock(mutex_octree_);
      if (!octree_)
      {
        return FOLLOW_DONE;
      }
      VoxelCache_t cache;
      const double scan = std::max(_pre_follow_horizon_, _pre_max_known_gap_);
      for (int j = progress + d; j >= 0 && j < n; j += d)
      {
        const PathSample_t& s = samples[j];
        if (labels[j] == LABEL_OUTSIDE || occupiedWithin(*octree_, s.position, _pre_follow_clearance_, cache))
        {
          end_reason = labels[j] == LABEL_OUTSIDE ? "outside" : "blocked";
          break;
        }
        length += s.position.distance(samples[j - d].position);
        if (length > scan)
        {
          end_reason = "scan end";
          break;
        }
        // unknown only beyond max_known_gap doesn't count: the known stretch up to it is better skipped via GOTO
        has_unknown = has_unknown || (labels[j] == LABEL_UNKNOWN && length <= _pre_max_known_gap_);
        if (length > _pre_follow_horizon_)
        {
          continue;
        }
        walked.push_back(j);
        if (s.position.distance(path.back()) <= _skip_path_point_distance_)
        {
          continue;
        }
        path.push_back(s.position);
        // heading of the waypoint being flown towards: the sample's own (segment end) along +index, its segment start's along -index
        headings.push_back(d > 0 ? s.yaw : wps[s.wp].yaw);
      }
    }

    if (!has_unknown || path.size() < 2 || path.back().distance(start_coord) <= _pre_min_path_length_)
    {
      if (!advance_only)
      {
        // a follow that ends right at its entry anchor didn't uncover anything
        const int entry = nearestSample(samples, pre_entry_u_);
        if (entry >= 0 && samples[progress].position.distance(samples[entry].position) <= _pre_min_path_length_)
        {
          EntryRecord_t& rec = entryRecord(samples, entry, d);
          rec.no_progress++;
          ROS_WARN("[MrsExplorer]: precomputed FOLLOW from sample %d (dir %+d) got nowhere (%s), no progress %d/%d", entry, d, end_reason,
                   rec.no_progress, _pre_max_no_progress_);
        }
        ROS_INFO("[MrsExplorer]: precomputed FOLLOW segment done at sample %d (%s, %s), finding next start", progress,
                 has_unknown ? "too short" : "no unknown left", end_reason);
      }
      return FOLLOW_DONE;
    }
    precomputed_has_targets_ = true;

    // the walked segment as cyan rays (published with path_ by pathAndTrajectory())
    for (size_t k = 0; k + 1 < walked.size(); k++)
    {
      const auto& v0 = samples[walked[k]].position;
      const auto& v1 = samples[walked[k + 1]].position;
      bv_path_->addRay(mrs_lib::geometry::Ray(Eigen::Vector3d(v0.x(), v0.y(), v0.z()), Eigen::Vector3d(v1.x(), v1.y(), v1.z())), 0.0, 1.0, 1.0, 1.0);
    }

    path.erase(path.begin());
    headings.erase(headings.begin());
    path_.points   = path;
    path_.headings = headings;
    path_.id       = next_path_id_++;
    goal_          = path.back();
    next_goal_     = path.back();
    ROS_INFO("[MrsExplorer]: precomputed FOLLOW%s from sample %d (waypoint ~%d, dir %+d), %.1fm, %lu points", advance_only ? " (advance)" : "",
             progress, samples[progress].wp, d, std::min(length, _pre_follow_horizon_), path.size());
    return FOLLOW_PATH;
  }

  // sample whose path parameter u is closest to the given one (samples are sorted by u), -1 if there are none
  int Explorer::nearestSample(const std::vector<PathSample_t>& samples, double u)
  {
    if (samples.empty())
    {
      return -1;
    }
    const auto it = std::lower_bound(samples.begin(), samples.end(), u, [](const PathSample_t& s, double v) { return s.u < v; });
    if (it == samples.begin())
    {
      return 0;
    }
    if (it == samples.end())
    {
      return static_cast<int>(samples.size()) - 1;
    }
    const int i = static_cast<int>(it - samples.begin());
    return (it->u - u < u - samples[i - 1].u) ? i : i - 1;
  }

  // the precomputed waypoints with extra samples inserted every precomputed/sample_step along each segment; each sample carries the yaw of the
  // waypoint its segment leads to (along +index), wp = the index of the waypoint it was resampled from (its segment's start for inserted ones)
  // and u = wp + fraction along the segment (the path parameter entries are kept by)
  std::vector<Explorer::PathSample_t> Explorer::resamplePath(const std::vector<PrecomputedWaypoint_t>& wps) const
  {
    std::vector<PathSample_t> samples;
    for (size_t i = 0; i < wps.size(); i++)
    {
      samples.push_back({wps[i].position, wps[i].yaw, static_cast<int>(i), static_cast<double>(i)});
      if (i + 1 == wps.size())
      {
        break;
      }
      const octomath::Vector3 d     = wps[i + 1].position - wps[i].position;
      const int               steps = std::max(1, static_cast<int>(std::ceil(d.norm() / _pre_sample_step_)));
      for (int k = 1; k < steps; k++)
      {
        samples.push_back({wps[i].position + d * (static_cast<float>(k) / steps), wps[i + 1].yaw, static_cast<int>(i), i + static_cast<double>(k) / steps});
      }
    }
    return samples;
  }

  // labels each sample with path_planner's reachability test (path_planner/hasClearance: every voxel within min_obstacle_clearance, stepped at
  // the octree resolution, must be known free; unknown counts as occupied), so a KNOWN sample is one path_planner can plan to:
  // OUTSIDE  - outside flight_zone_ (no roadmap nodes there), or on segment 0 (it leads from the precompute start inside the object's footprint)
  // OCCUPIED - occupied itself, or an occupied voxel within precomputed/clearance (too close to the structure to stand at)
  // UNKNOWN  - unknown itself, or an unknown voxel (and no occupied one) within precomputed/clearance
  // KNOWN    - the whole precomputed/clearance ball is known free
  std::vector<Explorer::WaypointLabel_t> Explorer::labelSamples(const std::vector<PathSample_t>& samples)
  {
    const size_t                 n = samples.size();
    std::vector<WaypointLabel_t> labels(n, LABEL_UNKNOWN);

    std::scoped_lock lock(mutex_octree_);
    if (!octree_)
    {
      return labels;
    }
    const OcTree_t& tree = *octree_;
    VoxelCache_t    cache; // shared by all balls (neighbouring samples' balls overlap almost entirely)

    for (size_t i = 0; i < n; i++)
    {
      const PathSample_t& s = samples[i];
      if (s.wp == 0 || !explorer_utils::intersect(flight_zone_, s.position))
      {
        labels[i] = LABEL_OUTSIDE;
        continue;
      }
      const uint8_t worst = clearanceState(tree, s.position, _pre_clearance_, cache);
      labels[i]           = worst == V_FREE ? LABEL_KNOWN : (worst == V_OCCUPIED ? LABEL_OCCUPIED : LABEL_UNKNOWN);
    }
    return labels;
  }

  // next sample from i in direction dir that is not OUTSIDE/OCCUPIED, -1 if none
  int Explorer::nextFollowed(const std::vector<WaypointLabel_t>& labels, int i, int dir)
  {
    for (int j = i + dir; j >= 0 && j < static_cast<int>(labels.size()); j += dir)
    {
      if (!isSkippedLabel(labels[j]))
      {
        return j;
      }
    }
    return -1;
  }

  // every KNOWN sample whose next followed sample in a direction is UNKNOWN is an entry, once per direction
  std::vector<Explorer::PathEntry_t> Explorer::findEntries(const std::vector<WaypointLabel_t>& labels)
  {
    std::vector<PathEntry_t> entries;
    for (int i = 0; i < static_cast<int>(labels.size()); i++)
    {
      if (labels[i] != LABEL_KNOWN)
      {
        continue;
      }
      for (const int dir : {+1, -1})
      {
        const int j = nextFollowed(labels, i, dir);
        if (j >= 0 && labels[j] == LABEL_UNKNOWN)
        {
          entries.emplace_back(i, dir);
        }
      }
    }
    return entries;
  }

  // record (no-progress count, backoff) of the entry at sample i in direction dir; records are kept by path parameter u and match a sample
  // within one index of it (two entries in the same direction are never adjacent), so they survive transform drift. Created empty on first
  // use; the reference is invalidated by the next call
  Explorer::EntryRecord_t& Explorer::entryRecord(const std::vector<PathSample_t>& samples, int i, int dir)
  {
    for (auto& rec : entry_records_)
    {
      if (rec.dir == dir && std::abs(nearestSample(samples, rec.u) - i) <= 1)
      {
        return rec;
      }
    }
    entry_records_.push_back({samples[i].u, dir, 0, ros::Time(0)});
    return entry_records_.back();
  }

  // plans start -> target (an entry anchor) through path_planner; the path only counts if it ends within precomputed/reach_tolerance of the
  // target (path_planner snaps the goal to the nearest roadmap node and still reports success when the goal itself isn't reachable from
  // there). Path points get the given yaw. False if the target isn't reached, or the path ends within precomputed/min_path_length of the
  // start (it would leave the UAV hovering)
  bool Explorer::planEntry(const octomap::point3d& start_coord, const octomath::Vector3& velocity, const octomap::point3d& target, double yaw,
                           std::vector<octomap::point3d>& path, std::vector<double>& headings)
  {
    std::vector<octomap::point3d> segment;
    if (!findSimplifiedPath(start_coord, target, velocity, segment) || segment.empty() || segment.back().distance(target) > _pre_reach_tolerance_)
    {
      return false;
    }

    // same deduplication as buildLocalPath()
    path.assign({start_coord});
    headings.assign({yaw});
    for (const auto& p : segment)
    {
      if (p.distance(path.back()) <= _skip_path_point_distance_)
      {
        continue;
      }
      path.push_back(p);
      headings.push_back(yaw);
    }
    return path.size() > 1 && path.back().distance(start_coord) > _pre_min_path_length_;
  }

}  // namespace explorer
