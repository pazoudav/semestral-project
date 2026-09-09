#include <path_planning/path_planner.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>

namespace path_planning
{

  PathPlanner::PathPlanner(octomap_planner_utils::AABB flight_zone, RoadmapParams params)
    : flight_zone_(flight_zone), params_(params), spatial_index_(params.connect_radius)
  {
  }

  std::vector<NodeId> PathPlanner::updateRoadmap(const octomap::OcTree& octree, const OctomapDiff& octomap_diff)
  {
    std::vector<NodeId> affected;

    if (octomap_diff.empty()) {
      return affected;
    }

    const octomap_planner_utils::AABB affected_region = boundingBoxOf(octomap_diff, params_.inflate_radius);
    const std::vector<NodeId>         dirty_nodes      = spatial_index_.queryRegion(affected_region);

    // 1. remove nodes that became invalid (now inside an obstacle) -- also drops incident edges
    removeInvalidNodes(octree, dirty_nodes, affected);

    // 2. re-validate/drop edges of the remaining dirty nodes that now cross occupied space
    revalidateEdges(octree, dirty_nodes, affected);

    // 3. sample new nodes into newly-freed space and connect them (PRM-style local repair)
    sampleNewNodes(octree, octomap_diff, affected);

    return affected;
  }

  std::unordered_map<NodeId, PathResult> PathPlanner::shortestPaths(NodeId start_id, const std::vector<NodeId>& goal_ids) const
  {
    std::unordered_map<NodeId, PathResult> results;

    if (nodes_.find(start_id) == nodes_.end()) {
      return results;
    }

    std::unordered_set<NodeId> remaining_goals;
    for (const NodeId id : goal_ids) {
      if (nodes_.find(id) != nodes_.end()) {
        remaining_goals.insert(id);
      }
    }
    if (remaining_goals.empty()) {
      return results;
    }

    using QueueEntry = std::pair<double, NodeId>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
    std::unordered_map<NodeId, double> dist;
    std::unordered_map<NodeId, NodeId> prev;

    dist[start_id] = 0.0;
    open.emplace(0.0, start_id);

    if (remaining_goals.erase(start_id) > 0) {
      results.emplace(start_id, PathResult{0.0, {start_id}});
    }

    while (!open.empty() && !remaining_goals.empty()) {
      const auto [d, id] = open.top();
      open.pop();

      const auto dist_it = dist.find(id);
      if (dist_it != dist.end() && d > dist_it->second) {
        continue; // stale entry: a cheaper path to `id` was already relaxed since this was queued
      }

      // Dijkstra finalizes nodes in non-decreasing distance order, so `d` here is `id`'s true
      // shortest distance from start -- reconstruct and record its path if it's a wanted goal.
      if (remaining_goals.erase(id) > 0) {
        std::vector<NodeId> path;
        for (NodeId at = id;; at = prev.at(at)) {
          path.push_back(at);
          if (at == start_id) {
            break;
          }
        }
        std::reverse(path.begin(), path.end());
        results.emplace(id, PathResult{d, std::move(path)});
      }

      const auto node_it = nodes_.find(id);
      if (node_it == nodes_.end()) {
        continue;
      }

      for (const RoadmapEdge& edge : node_it->second.neighbors) {
        const double alt          = d + edge.cost;
        const auto   nbr_dist_it  = dist.find(edge.neighbor_id);
        if (nbr_dist_it == dist.end() || alt < nbr_dist_it->second) {
          dist[edge.neighbor_id] = alt;
          prev[edge.neighbor_id] = id;
          open.emplace(alt, edge.neighbor_id);
        }
      }
    }

    return results;
  }

  std::optional<PathResult> PathPlanner::shortestPath(NodeId start_id, NodeId goal_id) const
  {
    auto results = shortestPaths(start_id, {goal_id});
    const auto it = results.find(goal_id);
    if (it == results.end()) {
      return std::nullopt;
    }
    return std::move(it->second);
  }

  std::optional<NodeId> PathPlanner::findNearestNode(const octomap::point3d& point) const
  {
    return spatial_index_.nearest(point);
  }

  std::vector<octomap::point3d> PathPlanner::simplifyPath(const octomap::OcTree& octree, const std::vector<octomap::point3d>& path) const
  {
    if (path.size() < 3) {
      return path;
    }

    octomap::KeyBoolMap clearance_cache; // occupancy cache shared across every check in this call, see hasClearanceCached

    std::vector<octomap::point3d> simplified;
    simplified.push_back(path.front());

    std::size_t anchor = 0;
    while (anchor < path.size() - 1) {
      // farthest waypoint still reachable from `anchor` with clearance; scanning from the end means the first hit is the farthest
      std::size_t farthest = anchor + 1;
      for (std::size_t j = path.size() - 1; j > anchor + 1; --j) {
        if (segmentHasClearance(octree, path[anchor], path[j], params_.min_obstacle_clearance, clearance_cache)) {
          farthest = j;
          break;
        }
      }
      simplified.push_back(path[farthest]);
      anchor = farthest;
    }

    return simplified;
  }

  void PathPlanner::removeInvalidNodes(const octomap::OcTree& octree, const std::vector<NodeId>& dirty_nodes, std::vector<NodeId>& affected)
  {
    for (const NodeId id : dirty_nodes) {
      const auto it = nodes_.find(id);
      if (it == nodes_.end()) {
        continue;
      }
      if (!hasClearance(octree, it->second.position, params_.min_obstacle_clearance)) {
        removeNode(id);
        affected.push_back(id);
      }
    }
  }

  void PathPlanner::revalidateEdges(const octomap::OcTree& octree, const std::vector<NodeId>& dirty_nodes, std::vector<NodeId>& affected)
  {
    for (const NodeId id : dirty_nodes) {
      const auto it = nodes_.find(id);
      if (it == nodes_.end()) {
        continue; // removed as invalid in step 1
      }

      std::vector<NodeId> to_drop;
      for (const RoadmapEdge& edge : it->second.neighbors) {
        const auto nbr_it = nodes_.find(edge.neighbor_id);
        if (nbr_it == nodes_.end() || !collisionFree(octree, it->second.position, nbr_it->second.position)) {
          to_drop.push_back(edge.neighbor_id);
        }
      }

      for (const NodeId nbr_id : to_drop) {
        removeEdge(id, nbr_id);
        affected.push_back(id);
        affected.push_back(nbr_id);
      }
    }
  }

  void PathPlanner::sampleNewNodes(const octomap::OcTree& octree, const OctomapDiff& octomap_diff, std::vector<NodeId>& affected)
  {
    std::vector<octomap::point3d> new_free_cells;
    for (const ChangedVoxel& v : octomap_diff) {
      if (v.state == VoxelState::FREE) {
        new_free_cells.push_back(v.position);
      }
    }

    for (const octomap::point3d& cell : new_free_cells) {
      if (!octomap_planner_utils::intersect(flight_zone_, cell)) {
        continue; // never sample new nodes outside the flight zone
      }

      if (octomap_planner_utils::getRand() > params_.target_density) {
        continue; // subsample new_free_cells at density=target_density
      }

      if (!farEnoughFromExisting(cell, params_.min_spacing)) {
        continue;
      }

      if (!hasClearance(octree, cell, params_.min_obstacle_clearance)) {
        continue; // too close to an occupied voxel (subsumes a plain occupancy check, since a zero-clearance point is included in the sweep)
      }

      const std::vector<NodeId> candidates = spatial_index_.radiusQuery(cell, params_.connect_radius);
      const NodeId              new_id     = createNode(cell);

      for (const NodeId cand_id : candidates) {
        const octomap::point3d& cand_pos = nodes_.at(cand_id).position;
        if (collisionFree(octree, cell, cand_pos)) {
          addEdge(new_id, cand_id, (cell - cand_pos).norm());
        }
      }

      affected.push_back(new_id);
    }
  }

  NodeId PathPlanner::createNode(const octomap::point3d& position)
  {
    const NodeId id = next_id_++;
    nodes_.emplace(id, RoadmapNode{id, position, {}});
    spatial_index_.insert(id, position);
    return id;
  }

  void PathPlanner::removeNode(NodeId id)
  {
    const auto it = nodes_.find(id);
    if (it == nodes_.end()) {
      return;
    }

    // dropping incident edges first avoids leaving dangling neighbor entries on the other end
    for (const RoadmapEdge& edge : it->second.neighbors) {
      const auto nbr_it = nodes_.find(edge.neighbor_id);
      if (nbr_it == nodes_.end()) {
        continue;
      }
      auto& nbr_neighbors = nbr_it->second.neighbors;
      nbr_neighbors.erase(std::remove_if(nbr_neighbors.begin(), nbr_neighbors.end(),
                                          [id](const RoadmapEdge& e) { return e.neighbor_id == id; }),
                           nbr_neighbors.end());
    }

    spatial_index_.remove(id);
    nodes_.erase(it);
  }

  void PathPlanner::addEdge(NodeId a, NodeId b, double cost)
  {
    nodes_.at(a).neighbors.push_back({b, cost});
    nodes_.at(b).neighbors.push_back({a, cost});
  }

  void PathPlanner::removeEdge(NodeId a, NodeId b)
  {
    const auto a_it = nodes_.find(a);
    if (a_it != nodes_.end()) {
      auto& neighbors = a_it->second.neighbors;
      neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
                                      [b](const RoadmapEdge& e) { return e.neighbor_id == b; }),
                       neighbors.end());
    }

    const auto b_it = nodes_.find(b);
    if (b_it != nodes_.end()) {
      auto& neighbors = b_it->second.neighbors;
      neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
                                      [a](const RoadmapEdge& e) { return e.neighbor_id == a; }),
                       neighbors.end());
    }
  }

  bool PathPlanner::isOccupied(const octomap::OcTree& octree, const octomap::point3d& p) const
  {
    const octomap::OcTreeNode* node = octree.search(p);
    return !node || octree.isNodeOccupied(node);
  }

  // approximate sphere sweep around p, stepped at the octree's own resolution: false as soon as
  // any occupied voxel is found within `clearance`
  bool PathPlanner::hasClearance(const octomap::OcTree& octree, const octomap::point3d& p, double clearance) const
  {
    const double step = octree.getResolution();
    for (double dx = -clearance; dx <= clearance; dx += step) {
      for (double dy = -clearance; dy <= clearance; dy += step) {
        for (double dz = -clearance; dz <= clearance; dz += step) {
          if (dx * dx + dy * dy + dz * dz > clearance * clearance) {
            continue;
          }
          if (isOccupied(octree, octomap::point3d(p.x() + dx, p.y() + dy, p.z() + dz))) {
            return false;
          }
        }
      }
    }
    return true;
  }

  // steps a ray from a to b and reports whether it passes through any known-occupied voxel;
  // unknown space is treated as passable, matching how newly-freed/unknown space is otherwise
  // handled by sampleNewNodes rather than by edge invalidation
  bool PathPlanner::collisionFree(const octomap::OcTree& octree, const octomap::point3d& a, const octomap::point3d& b) const
  {
    octomap::point3d direction = b - a;
    const double     distance  = direction.norm();
    if (distance < 1e-6) {
      return true;
    }
    direction /= distance;

    octomap::point3d hit;
    const bool has_hit = octree.castRay(a, direction, hit, /*ignoreUnknownCells=*/true, distance);
    return !has_hit;
  }

  bool PathPlanner::hasClearanceCached(const octomap::OcTree& octree, const octomap::point3d& p, double clearance, octomap::KeyBoolMap& cache) const
  {
    const double step = octree.getResolution();
    for (double dx = -clearance; dx <= clearance; dx += step) {
      for (double dy = -clearance; dy <= clearance; dy += step) {
        for (double dz = -clearance; dz <= clearance; dz += step) {
          if (dx * dx + dy * dy + dz * dz > clearance * clearance) {
            continue;
          }
          const octomap::point3d  q   = octomap::point3d(p.x() + dx, p.y() + dy, p.z() + dz);
          const octomap::OcTreeKey key = octree.coordToKey(q);

          const auto cached = cache.find(key);
          bool       occupied;
          if (cached != cache.end()) {
            occupied = cached->second;
          }
          else {
            occupied = isOccupied(octree, q);
            cache[key] = occupied;
          }

          if (occupied) {
            return false;
          }
        }
      }
    }
    return true;
  }

  // steps along a->b at octree resolution rather than casting a single zero-width ray, so the
  // whole segment (not just its center line) is required to keep `clearance` from occupied space
  bool PathPlanner::segmentHasClearance(const octomap::OcTree& octree, const octomap::point3d& a, const octomap::point3d& b, double clearance,
                                         octomap::KeyBoolMap& cache) const
  {
    octomap::point3d direction = b - a;
    const double     distance  = direction.norm();
    if (distance < 1e-6) {
      return hasClearanceCached(octree, a, clearance, cache);
    }
    direction /= distance;

    const double step = octree.getResolution();
    for (double t = 0.0; t < distance; t += step) {
      if (!hasClearanceCached(octree, a + direction * t, clearance, cache)) {
        return false;
      }
    }
    return hasClearanceCached(octree, b, clearance, cache);
  }

  bool PathPlanner::farEnoughFromExisting(const octomap::point3d& p, double min_spacing) const
  {
    return spatial_index_.radiusQuery(p, min_spacing).empty();
  }

  octomap_planner_utils::AABB PathPlanner::boundingBoxOf(const OctomapDiff& diff, double margin)
  {
    octomap::point3d min_p = diff.front().position;
    octomap::point3d max_p = diff.front().position;

    for (const ChangedVoxel& v : diff) {
      min_p.x() = std::min(min_p.x(), v.position.x());
      min_p.y() = std::min(min_p.y(), v.position.y());
      min_p.z() = std::min(min_p.z(), v.position.z());
      max_p.x() = std::max(max_p.x(), v.position.x());
      max_p.y() = std::max(max_p.y(), v.position.y());
      max_p.z() = std::max(max_p.z(), v.position.z());
    }

    const octomap::point3d margin_v(margin, margin, margin);
    return {min_p - margin_v, max_p + margin_v};
  }

  // -------------------------------- SpatialGrid --------------------------------

  PathPlanner::SpatialGrid::SpatialGrid(double cell_size)
    : cell_size_(cell_size)
  {
  }

  std::size_t PathPlanner::SpatialGrid::CellHash::operator()(const Cell& c) const
  {
    std::size_t h = std::hash<int>()(c.x);
    h ^= std::hash<int>()(c.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= std::hash<int>()(c.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }

  PathPlanner::SpatialGrid::Cell PathPlanner::SpatialGrid::cellOf(const octomap::point3d& p) const
  {
    return {static_cast<int>(std::floor(p.x() / cell_size_)),
            static_cast<int>(std::floor(p.y() / cell_size_)),
            static_cast<int>(std::floor(p.z() / cell_size_))};
  }

  void PathPlanner::SpatialGrid::insert(NodeId id, const octomap::point3d& position)
  {
    const Cell cell = cellOf(position);
    cells_[cell].push_back(id);
    cell_of_[id]    = cell;
    positions_[id]  = position;
  }

  void PathPlanner::SpatialGrid::remove(NodeId id)
  {
    const auto cell_it = cell_of_.find(id);
    if (cell_it == cell_of_.end()) {
      return;
    }

    const auto cells_it = cells_.find(cell_it->second);
    if (cells_it != cells_.end()) {
      auto& ids = cells_it->second;
      ids.erase(std::remove(ids.begin(), ids.end(), id), ids.end());
      if (ids.empty()) {
        cells_.erase(cells_it);
      }
    }

    cell_of_.erase(cell_it);
    positions_.erase(id);
  }

  std::vector<NodeId> PathPlanner::SpatialGrid::queryRegion(const octomap_planner_utils::AABB& region) const
  {
    const Cell min_c = cellOf(region.min);
    const Cell max_c = cellOf(region.max);

    std::vector<NodeId> result;
    for (int x = min_c.x; x <= max_c.x; ++x) {
      for (int y = min_c.y; y <= max_c.y; ++y) {
        for (int z = min_c.z; z <= max_c.z; ++z) {
          const auto it = cells_.find({x, y, z});
          if (it == cells_.end()) {
            continue;
          }
          result.insert(result.end(), it->second.begin(), it->second.end());
        }
      }
    }
    return result;
  }

  std::vector<NodeId> PathPlanner::SpatialGrid::radiusQuery(const octomap::point3d& center, double radius) const
  {
    const octomap::point3d            radius_v(radius, radius, radius);
    const octomap_planner_utils::AABB box{center - radius_v, center + radius_v};

    std::vector<NodeId> result;
    for (const NodeId id : queryRegion(box)) {
      if ((positions_.at(id) - center).norm() <= radius) {
        result.push_back(id);
      }
    }
    return result;
  }

  // expanding-ring search: scans grid cells in shells of increasing Chebyshev distance `r` from
  // center's own cell, tracking the closest node seen so far. Once every cell out to radius `r`
  // has been scanned, no unscanned cell can hold a point closer than r * cell_size_ to `center`
  // (the nearest a point in the next shell could be is that close, if `center` sits right at the
  // edge of its own cell) -- so the search stops as soon as that bound can no longer beat the
  // current best, instead of scanning every cell in the grid.
  std::optional<NodeId> PathPlanner::SpatialGrid::nearest(const octomap::point3d& center) const
  {
    if (positions_.empty()) {
      return std::nullopt;
    }

    const Cell center_cell = cellOf(center);

    std::optional<NodeId> best_id;
    double                 best_dist = std::numeric_limits<double>::infinity();

    std::size_t       visited_cells = 0;
    const std::size_t total_cells   = cells_.size();

    for (int r = 0; visited_cells < total_cells; ++r) {
      for (int x = center_cell.x - r; x <= center_cell.x + r; ++x) {
        for (int y = center_cell.y - r; y <= center_cell.y + r; ++y) {
          for (int z = center_cell.z - r; z <= center_cell.z + r; ++z) {
            if (std::max({std::abs(x - center_cell.x), std::abs(y - center_cell.y), std::abs(z - center_cell.z)}) != r) {
              continue; // only the shell at exactly Chebyshev distance r -- smaller r already covered the interior
            }

            const auto it = cells_.find({x, y, z});
            if (it == cells_.end()) {
              continue;
            }
            ++visited_cells;

            for (const NodeId id : it->second) {
              const double dist = (positions_.at(id) - center).norm();
              if (dist < best_dist) {
                best_dist = dist;
                best_id   = id;
              }
            }
          }
        }
      }

      if (best_id && best_dist <= static_cast<double>(r) * cell_size_) {
        break;
      }
    }

    return best_id;
  }

}
