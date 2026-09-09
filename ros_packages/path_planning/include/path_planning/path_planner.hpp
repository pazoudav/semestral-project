#pragma once

#include <octomap/octomap.h>
#include <octomap_planner_utils/utils.hpp>

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace path_planning
{

  using NodeId = std::uint64_t;

  // state a changed voxel transitioned into, relative to the previous roadmap update
  enum class VoxelState
  {
    FREE,
    OCCUPIED,
    UNKNOWN
  };

  // a single voxel reported by the caller as having changed state since the last UPDATE_ROADMAP call
  struct ChangedVoxel
  {
    octomap::point3d position;
    VoxelState       state;
  };

  // set of voxels that changed state since the previous roadmap update (newly occupied, newly
  // free, or newly unknown->free); the input to PathPlanner::updateRoadmap
  using OctomapDiff = std::vector<ChangedVoxel>;

  // one directed edge of a RoadmapNode: the neighbor it leads to and its traversal cost
  struct RoadmapEdge
  {
    NodeId neighbor_id;
    double cost;
  };

  // a single node of the sparse roadmap, together with its currently valid outgoing edges
  struct RoadmapNode
  {
    NodeId                    id;
    octomap::point3d          position;
    std::vector<RoadmapEdge>  neighbors;
  };

  // tuning knobs for UPDATE_ROADMAP's local repair pass
  struct RoadmapParams
  {
    double inflate_radius          = 1.0;  // margin added around the changed-voxel bounding box when collecting dirty nodes
    double target_density          = 0.3;  // fraction (0,1] of newly-freed cells subsampled as new-node candidates
    double min_spacing             = 0.75; // minimum allowed distance between two roadmap nodes
    double connect_radius          = 3.0;  // radius within which a new node attempts to connect to existing nodes
    double min_obstacle_clearance  = 0.5;  // a node within this distance of an occupied voxel is never added, and is removed if it already exists
  };

  // cheapest known path from a shortestPaths() call's start node to one particular node: its total
  // summed edge cost and the sequence of node ids from start to it, inclusive
  struct PathResult
  {
    double               cost;
    std::vector<NodeId>  path;
  };

  // Incremental sparse roadmap maintained over a live octomap::OcTree. UPDATE_ROADMAP repairs the
  // roadmap in place from the set of voxels that changed since the previous call instead of
  // rebuilding it from scratch: invalid nodes/edges are dropped, then newly-freed space is
  // resampled PRM-style to patch the gaps left behind.
  class PathPlanner
  {
    public:
      // flight_zone bounds where new nodes may be sampled (see sampleNewNodes) -- same AABB
      // frontier_detection::Detector builds from zone_x/zone_y/zone_z/zone/global/floor params
      explicit PathPlanner(octomap_planner_utils::AABB flight_zone, RoadmapParams params = RoadmapParams());

      // Applies one incremental repair pass against `octree`, given the voxels that changed state
      // since the last call. Returns the ids of every node touched (removed, edge-modified, or
      // newly created) by this pass.
      std::vector<NodeId> updateRoadmap(const octomap::OcTree& octree, const OctomapDiff& octomap_diff);

      // Dijkstra's algorithm over the roadmap graph, from start_id to every node listed in
      // goal_ids (stops once all of them have been finalized rather than exploring the whole
      // graph). Returns, for each reachable goal, its cost and node-id path from start_id
      // inclusive; a goal_ids entry that is not in the roadmap or not reachable from start_id is
      // simply absent from the result. Empty if start_id is not currently in the roadmap.
      std::unordered_map<NodeId, PathResult> shortestPaths(NodeId start_id, const std::vector<NodeId>& goal_ids) const;

      // convenience wrapper around shortestPaths() for a single goal_id; nullopt if goal_id is not
      // reachable from start_id (see shortestPaths() for the exact absence conditions)
      std::optional<PathResult> shortestPath(NodeId start_id, NodeId goal_id) const;

      // nearest-neighbor lookup among the current roadmap nodes, via the SpatialGrid's
      // expanding-ring search; nullopt if the roadmap has no nodes
      std::optional<NodeId> findNearestNode(const octomap::point3d& point) const;

      // greedy line-of-sight shortcutting: collapses `path` into as few straight segments as
      // possible by repeatedly extending each anchor to the farthest later waypoint that keeps
      // min_obstacle_clearance the whole way there (see segmentHasClearance), then continuing from
      // there. Occupancy queries are cached for the duration of one call (see hasClearanceCached).
      std::vector<octomap::point3d> simplifyPath(const octomap::OcTree& octree, const std::vector<octomap::point3d>& path) const;

      const std::unordered_map<NodeId, RoadmapNode>& nodes() const
      {
        return nodes_;
      }

      std::size_t size() const
      {
        return nodes_.size();
      }

    private:
      // Minimal spatial index over node positions, playing the role of the pseudocode's KDTree:
      // a uniform grid keyed by cell coordinates (cell size == connect_radius) supporting O(1)
      // insert/remove and region/radius queries. A grid was chosen over a real KD-tree because the
      // roadmap needs cheap node *removal*, which a classic KD-tree does not support without an
      // occasional full rebuild.
      class SpatialGrid
      {
        public:
          explicit SpatialGrid(double cell_size);

          void insert(NodeId id, const octomap::point3d& position);
          void remove(NodeId id);

          // ids of every indexed node whose position falls inside `region`
          std::vector<NodeId> queryRegion(const octomap_planner_utils::AABB& region) const;
          // ids of every indexed node within `radius` of `center`
          std::vector<NodeId> radiusQuery(const octomap::point3d& center, double radius) const;
          // id of the indexed node closest to `center`, found by an expanding ring search over
          // grid cells rather than a linear scan; nullopt if the grid is empty
          std::optional<NodeId> nearest(const octomap::point3d& center) const;

        private:
          struct Cell
          {
            int x, y, z;
            bool operator==(const Cell& o) const
            {
              return x == o.x && y == o.y && z == o.z;
            }
          };

          struct CellHash
          {
            std::size_t operator()(const Cell& c) const;
          };

          Cell cellOf(const octomap::point3d& p) const;

          double                                                    cell_size_;
          std::unordered_map<Cell, std::vector<NodeId>, CellHash>   cells_;
          std::unordered_map<NodeId, Cell>                          cell_of_;
          std::unordered_map<NodeId, octomap::point3d>              positions_;
      };

      octomap_planner_utils::AABB              flight_zone_;
      RoadmapParams                            params_;
      std::unordered_map<NodeId, RoadmapNode>  nodes_;
      SpatialGrid                              spatial_index_;
      NodeId                                   next_id_ = 1;

      // step 1: drops nodes that no longer have min_obstacle_clearance from an occupied voxel (and their incident edges)
      void removeInvalidNodes(const octomap::OcTree& octree, const std::vector<NodeId>& dirty_nodes, std::vector<NodeId>& affected);
      // step 2: drops edges of dirty nodes that now cross occupied space
      void revalidateEdges(const octomap::OcTree& octree, const std::vector<NodeId>& dirty_nodes, std::vector<NodeId>& affected);
      // step 3: PRM-style local repair -- samples new nodes into newly-freed space and connects them
      void sampleNewNodes(const octomap::OcTree& octree, const OctomapDiff& octomap_diff, std::vector<NodeId>& affected);

      NodeId createNode(const octomap::point3d& position);
      void   removeNode(NodeId id);
      void   addEdge(NodeId a, NodeId b, double cost);
      void   removeEdge(NodeId a, NodeId b);

      bool isOccupied(const octomap::OcTree& octree, const octomap::point3d& p) const;
      bool collisionFree(const octomap::OcTree& octree, const octomap::point3d& a, const octomap::point3d& b) const;
      bool farEnoughFromExisting(const octomap::point3d& p, double min_spacing) const;
      // false if any occupied voxel lies within `clearance` of p (approximate sphere check, stepped at octree resolution)
      bool hasClearance(const octomap::OcTree& octree, const octomap::point3d& p, double clearance) const;
      // same sphere check as hasClearance, but each queried voxel's occupancy is looked up in
      // `cache` first and stored there after, so repeat queries within one simplifyPath call (its
      // sphere sweeps around different points routinely overlap) hit the octree at most once
      bool hasClearanceCached(const octomap::OcTree& octree, const octomap::point3d& p, double clearance, octomap::KeyBoolMap& cache) const;
      // like collisionFree, but requires min_obstacle_clearance the entire way from a to b rather
      // than a bare zero-width raycast -- steps along the segment at octree resolution, checking
      // hasClearanceCached at each step
      bool segmentHasClearance(const octomap::OcTree& octree, const octomap::point3d& a, const octomap::point3d& b, double clearance,
                                octomap::KeyBoolMap& cache) const;

      static octomap_planner_utils::AABB boundingBoxOf(const OctomapDiff& diff, double margin);
  };

}
