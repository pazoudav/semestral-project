#include <explorer/explorer.hpp>

#include <unordered_map>

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
  }  // namespace

  // precomputed coverage path following ("path frontiers"): instead of flying skeleton_estimator's path start to end (most of it runs through
  // space that is either unknown, so unplannable, or already well known, so useless), follows it only where it enters unknown space.
  // The path is resampled every precomputed/sample_step (resamplePath()) and every sample labelled with path_planner's own reachability test
  // (labelSamples(): KNOWN = its whole precomputed/clearance ball is known free, the same test path_planner applies to its roadmap nodes and
  // goal); an entry is a KNOWN sample (anchor) whose next followed sample in a direction (+1/-1 along the tour) is UNKNOWN, i.e. the last
  // point along the path path_planner can still reach before unknown space. The UAV flies there via path_planner; while it does,
  // timerReplanner() calls this every precomputed/replan_period with advance_only, which only re-plans to the committed entry once its anchor
  // has moved on with the growing map (and otherwise keeps the current trajectory), so the goal stays ahead of the UAV instead of it braking
  // at every anchor. Entries are kept by path parameter (PathSample_t::u = waypoint index + fraction along its segment), which stays put
  // when skeleton_estimator's transform drifts, unlike positions. A full replan tries the committed entry first, then the rest nearest-first
  // (Euclidean to the anchor, at most precomputed/max_entry_attempts). No TSP, no progress index: the map alone says what is done.
  // An entry whose anchor the UAV reaches precomputed/max_no_progress times without it advancing is skipped until the selected path changes;
  // one that fails to plan is skipped for precomputed/failure_backoff.
  // Fills path_ (with FC-Planner headings) and returns true on success; false -> keep the current trajectory (advance_only) or fall back to
  // frontier exploration (makePath())
  Explorer::PrecomputedResult_t Explorer::makePathPrecomputed(const octomap::point3d& start_coord, const octomath::Vector3& velocity, bool advance_only)
  {
    std::vector<PrecomputedWaypoint_t> wps;
    std::string                        frame;
    {
      std::scoped_lock lock(mutex_precomputed_);
      if (precomputed_reset_)
      {
        committed_entry_.reset();
        entry_records_.clear();
        precomputed_reset_ = false;
      }
      if (!precomputed_stable_ || precomputed_path_.size() < 2)
      {
        precomputed_has_targets_ = false;
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
      return PRE_NONE;
    }

    const std::vector<PathSample_t> samples = resamplePath(wps);
    const int                       n       = static_cast<int>(samples.size());
    const auto                      labels  = labelSamples(samples);

    // entries without progress are dropped until the selected path changes; entries that failed to plan only for precomputed/failure_backoff
    const ros::Time          now     = ros::Time::now();
    std::vector<PathEntry_t> entries = findEntries(labels);
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [&](const PathEntry_t& e) { return entryRecord(samples, e.first, e.second).no_progress >= _pre_max_no_progress_; }),
                  entries.end());
    // still has targets while an entry is only backing off, so exploration doesn't end over a transient failure
    precomputed_has_targets_ = !entries.empty();
    entries.erase(std::remove_if(entries.begin(), entries.end(),
                                 [&](const PathEntry_t& e) { return now < entryRecord(samples, e.first, e.second).retry_after; }),
                  entries.end());

    // advance the committed entry through the known stretch in its direction; the anchor becomes the last KNOWN sample before unknown space
    std::optional<PathEntry_t> committed;
    int                        committed_from = -1; // sample of the committed anchor as of the previous replan
    if (committed_entry_)
    {
      const auto [u, d] = *committed_entry_;
      committed_from    = nearestSample(samples, u);
      if (committed_from >= 0 && labels[committed_from] == LABEL_KNOWN)
      {
        int last_known = committed_from;
        for (int j = committed_from + d; j >= 0 && j < n && labels[j] != LABEL_UNKNOWN; j += d)
        {
          if (labels[j] == LABEL_KNOWN)
          {
            last_known = j;
          }
        }
        const PathEntry_t e{last_known, d};
        if (std::find(entries.begin(), entries.end(), e) != entries.end())
        {
          committed = e;
        }
      }
    }

    // periodic replan while flying to the committed anchor: only worth a new trajectory once that anchor has moved on noticeably
    if (advance_only)
    {
      if (!committed || samples[committed->first].position.distance(samples[committed_from].position) <= _pre_min_path_length_)
      {
        return PRE_NONE;
      }
      entries = {*committed};
    }

    if (entries.empty())
    {
      ROS_INFO_THROTTLE(1.0, "[MrsExplorer]: precomputed path has no %sentry into unknown space, frontier exploration",
                        precomputed_has_targets_ ? "available (backing off) " : "");
      committed_entry_.reset();
      return PRE_NONE;
    }

    // committed entry first, the rest nearest-first
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
      const PathEntry_t&            e      = entries[k];
      const PathSample_t&           anchor = samples[e.first];
      EntryRecord_t&                rec    = entryRecord(samples, e.first, e.second);
      std::vector<octomap::point3d> path;
      std::vector<double>           headings;

      // vertical entry: the lidar (360 deg, +-45 deg vertical FOV) never sees the path straight above/below it, so instead of flying up the
      // path line, zigzag at <= 45 deg: from the path line step out sideways to a look-up point (precomputed/lookup_offset away from the
      // object, at the anchor's height), from which the path line above/below is visible up to about lookup_offset further; once the anchor
      // has moved on, fly diagonally to it, step out again, and so on
      octomap::point3d target = anchor.position;
      bool             lookup = false;
      if (isVerticalEntry(samples, e))
      {
        const octomath::Vector3 off = start_coord - anchor.position;
        const double            horiz = std::hypot(off.x(), off.y());
        const double            climb = (samples[std::clamp(e.first + e.second, 0, n - 1)].position.z() >= anchor.position.z() ? 1.0 : -1.0) *
                             (anchor.position.z() - start_coord.z());
        if (start_coord.distance(anchor.position) <= 1.5 * _pre_lookup_offset_)
        {
          if (horiz < 0.5 * _pre_lookup_offset_)
          {
            lookup = findLookupPoint(anchor, target);  // on the path line: step out (stays the anchor if no look-up point is clear)
          } else if (climb < _pre_lookup_min_climb_)
          {
            // at the look-up point, but the path line above isn't in the map yet (map latency, or never observable): wait here
            if (advance_only)
            {
              return PRE_NONE;
            }
            rec.no_progress++;
            ROS_WARN("[MrsExplorer]: vertical precomputed entry at sample %d (dir %+d) hasn't moved on yet, holding, no progress %d/%d", e.first,
                     e.second, rec.no_progress, _pre_max_no_progress_);
            return PRE_HOLD;
          }
        }
      }

      // already at the anchor: the boundary hasn't moved since the UAV got here (e.g. never-observable space next to it) = no progress
      if (!lookup && start_coord.distance(anchor.position) <= _pre_min_path_length_)
      {
        if (advance_only)
        {
          return PRE_NONE;
        }
        rec.no_progress++;
        ROS_WARN("[MrsExplorer]: already at the anchor of precomputed entry at sample %d (dir %+d), no progress %d/%d", e.first, e.second,
                 rec.no_progress, _pre_max_no_progress_);
        continue;
      }
      // planning failure (roadmap not there yet, map changing): back off for a while instead of counting it against the entry
      if (!planEntry(start_coord, velocity, target, anchor.yaw, path, headings))
      {
        if (advance_only)
        {
          return PRE_NONE;
        }
        rec.retry_after = now + ros::Duration(_pre_failure_backoff_);
        ROS_WARN("[MrsExplorer]: precomputed entry at sample %d (dir %+d) could not be planned, retry in %.1fs", e.first, e.second, _pre_failure_backoff_);
        continue;
      }
      committed_entry_ = std::make_pair(anchor.u, e.second);

      // the unknown stretch of the precomputed tour ahead of the anchor (up to the next KNOWN sample) as cyan rays (published with path_ by pathAndTrajectory())
      for (int j = e.first; j + e.second >= 0 && j + e.second < n; j += e.second)
      {
        const auto& v0 = samples[j].position;
        const auto& v1 = samples[j + e.second].position;
        bv_path_->addRay(mrs_lib::geometry::Ray(Eigen::Vector3d(v0.x(), v0.y(), v0.z()), Eigen::Vector3d(v1.x(), v1.y(), v1.z())), 0.0, 1.0, 1.0, 1.0);
        if (labels[j + e.second] == LABEL_KNOWN)
        {
          break;
        }
      }

      // same commit as finalizePath(): drop the start point, goal_ = target (replan on approaching it), next_goal_ = path end
      path.erase(path.begin());
      headings.erase(headings.begin());
      path_.points   = path;
      path_.headings = headings;
      path_.id       = next_path_id_++;
      goal_          = target;
      next_goal_     = path.back();
      ROS_ERROR("[MrsExplorer]: %s precomputed path from sample %d (waypoint ~%d, dir %+d)%s, %lu points", advance_only ? "advancing along" : "following",
                e.first, anchor.wp, e.second, lookup ? " via look-up point" : "", path.size());
      return PRE_PATH;
    }

    committed_entry_.reset();
    ROS_WARN("[MrsExplorer]: no precomputed entry could be planned (%d tried), frontier exploration", attempts);
    return PRE_NONE;
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

  // plans start -> target (the anchor, or its look-up point) through path_planner; the path only counts if it ends within
  // precomputed/reach_tolerance of the target
  // (path_planner snaps the goal to the nearest roadmap node and still reports success when the goal itself isn't reachable from there).
  // Since the anchor is the last sample path_planner can reach before unknown space, nothing past it can be planned yet; the next replan
  // continues from the grown map. Path points get the given yaw. False if the target isn't reached, or the path ends within
  // precomputed/min_path_length of the start (it would leave the UAV hovering)
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

  // the path from the entry's anchor on in its direction, over about precomputed/lookup_offset, climbs or descends steeper than
  // precomputed/vertical_angle, i.e. into the lidar's blind cone above/below
  bool Explorer::isVerticalEntry(const std::vector<PathSample_t>& samples, const PathEntry_t& entry) const
  {
    const auto [i, dir]              = entry;
    const octomap::point3d& a        = samples[i].position;
    octomap::point3d        far      = a;
    for (int j = i + dir; j >= 0 && j < static_cast<int>(samples.size()); j += dir)
    {
      far = samples[j].position;
      if (far.distance(a) >= _pre_lookup_offset_)
      {
        break;
      }
    }
    const octomath::Vector3 v     = far - a;
    const double            horiz = std::hypot(v.x(), v.y());
    return v.norm() > 0.0 && std::abs(v.z()) > horiz * std::tan(_pre_vertical_angle_);
  }

  // look-up point of an anchor: precomputed/lookup_offset sideways from it at the same height, away from the object (opposite of the
  // FC-Planner yaw, which faces the surface), or rotated by +-45/+-90 deg from that if that one isn't clear; it must lie in flight_zone_
  // with its whole precomputed/clearance ball known free (so path_planner can reach it); false if no candidate is
  bool Explorer::findLookupPoint(const PathSample_t& anchor, octomap::point3d& result)
  {
    std::scoped_lock lock(mutex_octree_);
    if (!octree_)
    {
      return false;
    }
    VoxelCache_t cache;
    for (const double rot : {0.0, M_PI_4, -M_PI_4, M_PI_2, -M_PI_2})
    {
      const double           heading = anchor.yaw + M_PI + rot;
      const octomap::point3d p       = anchor.position + octomap::point3d(_pre_lookup_offset_ * std::cos(heading), _pre_lookup_offset_ * std::sin(heading), 0.0);
      if (explorer_utils::intersect(flight_zone_, p) && clearanceState(*octree_, p, _pre_clearance_, cache) == V_FREE)
      {
        result = p;
        return true;
      }
    }
    return false;
  }

}  // namespace explorer
