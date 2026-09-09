# path_planning

## 1. Purpose

`path_planning` is a ROS1 nodelet (`path_planning::PathPlanningNodelet`) intended to eventually replace `prm_solver`'s roadmap/search with a different path-planning approach. It is deliberately built with the same outward-facing ROS interface as `prm_solver` (identical private in-topic handle names and a byte-for-byte identical `FindSimplifiedPath.srv`), so it could later be swapped in for `prm_solver` without changing any caller. Unlike `prm_solver`, whose map input is already global and replaces its tree wholesale, this package's octomap input is the *local*-zone map, which it fuses into a persistent global `octree_` member the same way `frontier_detection::Detector` does. Each incoming octomap is also turned into a diff of changed voxels — restricted to a local zone around the UAV's current position (`octomap_planner_utils::localZoneFromPosition`, sized by `zone/local/width`/`zone/local/height`, clamped to `flight_zone_`) rather than the whole incoming tree — and fed straight into `PathPlanner::updateRoadmap`, so the sparse roadmap is kept live off the octomap callback; `~find_simplified_path_in` snaps its requested start/goal onto the nearest existing roadmap nodes, runs `PathPlanner::shortestPath` between them, and shortcuts the resulting node-position path via `PathPlanner::simplifyPath` (greedy shortcutting that requires `min_obstacle_clearance` along each candidate segment, not just a bare line-of-sight raycast, against the current octree) before returning it. The frontier subscription is wired up but not yet acted on, and the update timer remains an empty stub (`timerUpdate` only checks readiness flags). It is not yet started from `tmux/session.yml` or included by `mrs_octomap_mapping_planning`'s `mapplan.launch`.

The package also contains `path_planning::PathPlanner` (`include/path_planning/path_planner.hpp`, `src/path_planner.cpp`), a standalone, self-contained class implementing incremental sparse-roadmap maintenance, Dijkstra-based shortest-path search, and greedy clearance-preserving path simplification over a live `octomap::OcTree`. It is compiled into the same `MrsPathPlanning` library as the nodelet and is called from two places: `callbackOctomap` (`updateRoadmap`, on every octomap with a non-empty diff) and `callbackFindSimplifiedPath` (`findNearestNode`/`shortestPath`/`simplifyPath`).

## 2. ROS interface

### Subscribed topics (private names, remapped in `launch/path_planning.launch`)

| Private name | Remapped to | Type | Purpose |
|---|---|---|---|
| `~octomap_in` | `octomap_server/octomap_local_binary` | `octomap_msgs/Octomap` | Local-zone octomap; each message is decoded and fused into the persistent `octree_` (`callbackOctomap`) — seeded from the first message, merged via `octomap_planner_utils::mergeInto` thereafter. The roadmap diff fed to `PathPlanner::updateRoadmap` is further restricted to a local zone around the UAV's current position (`octomap_planner_utils::localZoneFromPosition`, using `zone/local/width`/`zone/local/height`), left empty for that callback if the UAV's position isn't currently available. Sets `map_ready_`. |
| `~tracker_cmd_in` | `control_manager/tracker_cmd` | `mrs_msgs/TrackerCommand` | UAV position/command feed, used only via `SubscribeHandler` freshness for `getPosition()`; no dedicated callback logic. |
| `~diagnostics_in` | `control_manager/diagnostics` | `mrs_msgs/ControlManagerDiagnostics` | Control-manager diagnostics; `controlManagerDiagCallback` is a no-op, subscribed only so `getPosition()` can check message freshness. |
| `~frontiers_in` | `frontier_detector/frontiers` | `frontier_detection/FrontierArray` | Frontier list; `callbackFrontiers` currently only logs that frontiers were received (TODO: nothing is inserted into any planning data structure yet, since none exists). |

### Services

| Private name | Remapped to | Type | Purpose |
|---|---|---|---|
| `~find_simplified_path_in` | `~find_simplified_path` | `path_planning/FindSimplifiedPath` (this package's own `.srv`, field-identical to `prm_solver`'s) | Request: `geometry_msgs/Point start`, `geometry_msgs/Point goal`, `geometry_msgs/Vector3 velocity` (`velocity` is unused). Response: `geometry_msgs/Point[] path`, `bool success`. `callbackFindSimplifiedPath` snaps `start`/`goal` onto their nearest roadmap nodes (`PathPlanner::findNearestNode`, an expanding-ring search over `SpatialGrid`), runs `PathPlanner::shortestPath` between them, shortcuts the resulting node-position path via `PathPlanner::simplifyPath` against the current `octree_` (falls back to the raw path if `octree_` is still null), and fills `path` with the simplified waypoints; `success` is `path.size() > 1`. Returns `success = false` (without calling `shortestPath`) if no roadmap node exists yet near `start`/`goal`, or if no path is found between them. |

### Visualization

`bv_roadmap_` (`mrs_lib::BatchVisualizer`, topic `visualize_roadmap`) draws the current roadmap's nodes and edges; each octomap callback that touches the roadmap builds a `RoadmapVisSnapshot` (`enqueueVisualization`) and hands it to a dedicated `visualizationWorker` thread, which does the marker construction/publish off the octomap-callback thread — mirroring `prm_solver`'s roadmap visualization.

## 3. Important functions/classes

- **`PathPlanningNodelet`** (`src/path_planning_nodelet.cpp`) — the nodelet; everything lives inline in this single class/file (no split-out planner class yet, unlike `prm_solver`'s `PRM`).
  - `onInit()` — loads zone/timer parameters, sets up subscribers/service, builds `flight_zone_`, starts the update timer and `tree_cleanup_thread_`.
  - `~PathPlanningNodelet()` — sets `tree_cleanup_shutdown_`, notifies `tree_cleanup_cv_`, and joins `tree_cleanup_thread_`.
  - `callbackOctomap` — decodes the incoming local-zone octomap message; resolves the UAV's current position (`octomap_planner_utils::getPosition`, transformed into the message's frame) and, if available, builds a local-zone `AABB` around it (`octomap_planner_utils::localZoneFromPosition`, sized by `zone/local/width`/`zone/local/height`, clamped to `flight_zone_`) and builds an `OctomapDiff` only from the decoded tree's leaves inside that box (`begin_leafs_bbx`/`end_leafs_bbx`) — if the UAV's position isn't available, the diff is left empty for that callback (throttled warning logged). Under `mutex_octree_`, seeds `octree_` from the decoded tree if `octree_` is still null, otherwise expands the decoded tree and merges it into `octree_` via `octomap_planner_utils::mergeInto`. If the diff is non-empty, feeds it to `path_planner_->updateRoadmap()` under `mutex_roadmap_`. Hands the decoded local tree off to `retireTree()` for destruction off the callback thread, enqueues a visualization update, and sets `map_ready_`.
  - `timerUpdate` (rate: `timer_rates/update` in `config/path_planning.yaml`) — currently an empty stub: only checks `is_initialized_`/`map_ready_` (TODO: drive further path-planning maintenance from here once needed).
  - `callbackFrontiers` — logs that a `FrontierArray` was received; does not yet insert anything into any data structure (TODO comment).
  - `callbackFindSimplifiedPath` — the `find_simplified_path_in` service handler; converts `req.start`/`req.goal` to `octomap::point3d`, snaps each onto its nearest roadmap node via `path_planner_->findNearestNode()`, runs `path_planner_->shortestPath()` to get a raw node-position path, then shortcuts it via `path_planner_->simplifyPath(*octree_, raw_path)` (falling back to the raw path if `octree_` is still null) before converting it into `res.path` (one `geometry_msgs::Point` per waypoint). Locks both `mutex_octree_` and `mutex_roadmap_` (via `std::scoped_lock`) for the duration, since `simplifyPath` needs read access to the octree alongside the roadmap lookups. `res.success` is `res.path.size() > 1`; returns `success = false` early if either point has no nearby roadmap node, or if `shortestPath` finds nothing.
  - `controlManagerDiagCallback` — no-op, present only so the diagnostics `SubscribeHandler` can track freshness.
  - `timeoutOctomap` / `timeoutTrackerCmd` — `SubscribeHandler` timeout callbacks; log a throttled warning once a topic that previously had data goes stale.
  - `getPosition` — resolves the UAV's current reference position via `octomap_planner_utils::getPosition`, using the octree's frame.
  - `enqueueVisualization` / `visualizationWorker` — copy the current roadmap (nodes + one draw per undirected edge) into a `RoadmapVisSnapshot` and hand it to a dedicated visualization thread, which builds/publishes `bv_roadmap_` markers off the octomap-callback thread.
  - `retireTree` / `treeCleanupWorker` — same background-cleanup pattern as `frontier_detection::Detector`: retired local octrees are handed to a dedicated `tree_cleanup_thread_` so that freeing a large tree's nodes never happens on the octomap-callback thread.
- **`PathPlanner`** (`include/path_planning/path_planner.hpp`, `src/path_planner.cpp`) — standalone, self-contained class implementing incremental sparse-roadmap maintenance, Dijkstra-based shortest-path search, and greedy clearance-preserving path simplification over a live `octomap::OcTree`. Compiled into the same `MrsPathPlanning` library as the nodelet and called from `PathPlanningNodelet::callbackOctomap` (`updateRoadmap`) and `callbackFindSimplifiedPath` (`findNearestNode`/`shortestPath`/`simplifyPath`). Reuses `octomap_planner_utils::AABB` and `octomap_planner_utils::getRand()` from the shared utils library; deliberately avoids pulling in PCL/FLANN for a real KD-tree.
  - Constructed as `PathPlanner(octomap_planner_utils::AABB flight_zone, RoadmapParams params = RoadmapParams())`; `flight_zone` bounds where `sampleNewNodes` may place new nodes.
  - `updateRoadmap(const octomap::OcTree& octree, const OctomapDiff& octomap_diff)` — the only mutating entry point; returns the `std::vector<NodeId>` of every node touched (removed, edge-modified, or newly created) by the pass. Runs three steps in order:
    1. `removeInvalidNodes` — computes the affected region as the diff's bounding box inflated by `RoadmapParams::inflate_radius`, collects the "dirty" nodes inside it via the spatial index, and drops any of them that no longer have `min_obstacle_clearance` from an occupied voxel (`hasClearance`) — dropping their incident edges too.
    2. `revalidateEdges` — for each surviving dirty node, drops any edge whose straight-line path to its neighbor no longer passes `collisionFree` (an `octree.castRay` check that treats unknown cells as passable, matching how newly-freed/unknown space is instead handled by step 3).
    3. `sampleNewNodes` — subsamples the diff's newly-`FREE` cells inside `flight_zone_` at probability `target_density` (via `octomap_planner_utils::getRand()`), skips candidates too close to an existing node (`min_spacing`) or without `min_obstacle_clearance` (`hasClearance`), creates a node for the rest, and connects each to existing nodes within `connect_radius` that pass the same `collisionFree` check.
  - `shortestPaths(NodeId start_id, const std::vector<NodeId>& goal_ids)` — Dijkstra from `start_id`, stopping once every listed goal has been finalized; returns a `PathResult{cost, path}` per reachable goal, omitting unreachable/unknown ones. Empty if `start_id` isn't in the roadmap.
  - `shortestPath(NodeId start_id, NodeId goal_id)` — convenience wrapper around `shortestPaths` for a single goal; `std::nullopt` if unreachable.
  - `findNearestNode(const octomap::point3d& point)` — nearest-neighbor lookup among the current roadmap nodes; delegates to the `SpatialGrid`'s `nearest()`, an expanding-ring search over grid cells rather than a linear scan over `nodes()`. `std::nullopt` if the roadmap is empty. Called (under `mutex_roadmap_`) by `PathPlanningNodelet::callbackFindSimplifiedPath` to snap `req.start`/`req.goal` onto roadmap nodes.
  - `simplifyPath(const octomap::OcTree& octree, const std::vector<octomap::point3d>& path)` — greedy shortcutting: starting from an anchor (initially `path.front()`), scans backward from the end of `path` for the farthest waypoint whose segment to the anchor keeps `min_obstacle_clearance` the whole way (`segmentHasClearance`, not a bare zero-width raycast), appends it to the result, and continues from there, collapsing `path` into as few straight, clearance-respecting segments as possible. Builds one `octomap::KeyBoolMap` occupancy cache local to the call and threads it through every `segmentHasClearance`/`hasClearanceCached` lookup, so repeat voxel queries within the same simplification are answered from the cache instead of re-querying the octree; the cache does not persist across separate calls. Returns `path` unchanged if it has fewer than 3 points. Called by `PathPlanningNodelet::callbackFindSimplifiedPath` on the raw `shortestPath` result before it is returned to the caller.
  - `hasClearanceCached(const octomap::OcTree& octree, const octomap::point3d& p, double clearance, octomap::KeyBoolMap& cache)` (private) — the same sphere-sweep clearance check as `hasClearance`, but each queried voxel's occupancy is looked up in `cache` first and stored there if absent, so a voxel already queried earlier in the same `simplifyPath` call is never re-queried against the octree.
  - `segmentHasClearance(const octomap::OcTree& octree, const octomap::point3d& a, const octomap::point3d& b, double clearance, octomap::KeyBoolMap& cache)` (private) — like `collisionFree`, but steps along the segment from `a` to `b` at octree resolution (rather than a single raycast) and requires `hasClearanceCached` to hold at every step, so the whole segment — not just its center line — must keep `clearance` from occupied space. Used only by `simplifyPath`; `collisionFree` itself is unchanged and still used by `revalidateEdges`/`sampleNewNodes`.
  - `nodes()` / `size()` — const accessors over the internal `std::unordered_map<NodeId, RoadmapNode>`.
  - Types: `NodeId` (`std::uint64_t`), `VoxelState` (`FREE`/`OCCUPIED`/`UNKNOWN`), `ChangedVoxel{position, state}`, `OctomapDiff = std::vector<ChangedVoxel>`, `RoadmapEdge{neighbor_id, cost}`, `RoadmapNode{id, position, neighbors}`, `RoadmapParams{inflate_radius, target_density, min_spacing, connect_radius, min_obstacle_clearance}`, `PathResult{cost, path}`.
  - Private nested `SpatialGrid` class — a uniform-grid spatial index over node positions (cell size == `connect_radius`) supporting `insert`/`remove`/`queryRegion(AABB)`/`radiusQuery(point, radius)`/`nearest(point)`. `nearest` scans grid cells in shells of increasing Chebyshev distance from the query point's cell, stopping once the closest node found so far is within the distance the current shell radius guarantees no farther cell could beat (falling back to scanning every occupied cell as a termination guarantee). Used in place of a real KD-tree specifically because the roadmap needs cheap node *removal*, which a classic KD-tree doesn't support without an occasional full rebuild. A clean-room implementation, not derived from `prm_solver`'s roadmap code.
- **`include/path_planning/path_planning.hpp`** — placeholder header for any further planning classes beyond `PathPlanner`; currently unused/empty. `PathPlanningNodelet` itself is still entirely self-contained in `path_planning_nodelet.cpp`.

## 4. File structure

```
path_planning/
├── CMakeLists.txt                        # catkin build config; builds MrsPathPlanning from path_planning_nodelet.cpp + path_planner.cpp
├── package.xml                            # package manifest/dependencies
├── nodelets.xml                           # pluginlib export: path_planning/PathPlanning -> path_planning::PathPlanningNodelet
├── config/
│   └── path_planning.yaml                 # zone sizes, update timer rate, RoadmapParams tuning, visualization scales
├── launch/
│   └── path_planning.launch               # standalone/nodelet-manager launch, topic/service remaps
├── srv/
│   └── FindSimplifiedPath.srv             # start/goal/velocity -> path/success
├── include/path_planning/
│   ├── path_planning.hpp                  # empty placeholder header for further planning classes, currently unused
│   └── path_planner.hpp                   # PathPlanner: incremental sparse-roadmap maintenance + Dijkstra search over a live octomap
└── src/
    ├── path_planning_nodelet.cpp          # PathPlanningNodelet: the current nodelet implementation
    └── path_planner.cpp                   # PathPlanner implementation; called from the nodelet's callbackOctomap/callbackFindSimplifiedPath
```

## 5. Dependencies

From `package.xml` / `CMakeLists.txt`:

- `roscpp`, `rospy`, `nodelet`, `pluginlib` (via `nodelets.xml`) — nodelet plumbing.
- `mrs_lib` — `SubscribeHandler`, `ParamLoader`, `Transformer`, `mutex` helpers, and `BatchVisualizer` (`bv_roadmap_`, publishing the live roadmap on `visualize_roadmap`).
- `mrs_msgs` — `TrackerCommand`, `ControlManagerDiagnostics`.
- `octomap_msgs`, `octomap_ros`, and the system `octomap`/`OCTOMAP` library — map representation and (de)serialization.
- `message_generation`/`message_runtime`, `std_msgs`, `geometry_msgs` — for the package's own `FindSimplifiedPath.srv`.
- `frontier_detection` — **message-only dependency**: consumes `frontier_detection/FrontierArray` on `~frontiers_in`; does not link against or call into `frontier_detection`'s code.
- `octomap_planner_utils` — shared geometry/octomap helpers. `PathPlanningNodelet` uses `mergeInto`, `getPosition`, and `localZoneFromPosition` (in `callbackOctomap`, to restrict the roadmap diff to a local zone around the UAV's current position); `PathPlanner` separately reuses `AABB` (for its bounding-box/region queries and `flight_zone_`), `getRand()` (for `sampleNewNodes`'s subsampling), and `intersect()` (to keep new nodes inside `flight_zone_`) — no new package dependency was added for `PathPlanner`, deliberately avoiding PCL/FLANN for a KD-tree.
- `cmake_modules` — pulled in via catkin; Eigen3 (used by `visualizationWorker` for `bv_roadmap_`'s points/rays) is not an explicit dependency of its own and comes in transitively via `mrs_lib`.

`CATKIN_ENABLE_TESTING` is explicitly disabled; there is no test suite for this package.
