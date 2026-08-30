# prm_solver

## 1. Purpose

`prm_solver` maintains a probabilistic roadmap (PRM) over the live octomap around the UAV and answers flyable-path queries against it. It runs as a standalone nodelet (`prm_solver::PRMNodelet`) that keeps its roadmap in sync with the incoming map and with candidate viewpoints from `frontier_detection` on its own, independent of any other package's control flow, and exposes a single blocking service that returns a simplified, collision-checked path between two points. `mrs_octomap_planner`'s `Explorer::makePath()` is the only known caller of that service, using it to turn TSP-ordered viewpoint tours into flyable sub-paths.

## 2. ROS interface

### Subscribed topics (private names, remapped in `launch/prm_solver.launch`)

| Private name | Remapped to | Type | Purpose |
|---|---|---|---|
| `~octomap_in` | `octomap_server/octomap_global_full` | `octomap_msgs/Octomap` | Incoming map; converted to an `octomap::OcTree` and stored for the update timer (`callbackOctomap`). |
| `~tracker_cmd_in` | `control_manager/tracker_cmd` | `mrs_msgs/TrackerCommand` | UAV position/command feed, used only via `SubscribeHandler` freshness (`getPosition()`), no dedicated callback logic. |
| `~diagnostics_in` | `control_manager/diagnostics` | `mrs_msgs/ControlManagerDiagnostics` | Control-manager diagnostics; callback is a no-op, subscribed only so `getPosition()` can check message freshness. |
| `~frontiers_in` | `frontier_detection/frontiers` | `frontier_detection/FrontierArray` | Frontier list; for every frontier in every message (not just newly-added ones), the first viewpoint's position is inserted as a roadmap node via `PRM::addNode` (`callbackFrontiers`). |

### Services

| Private name | Remapped to | Type | Purpose |
|---|---|---|---|
| `~find_simplified_path_in` | `~find_simplified_path` | `prm_solver/FindSimplifiedPath` (this package's own `.srv`) | The package's only externally-callable entry point. Request: `geometry_msgs/Point start`, `geometry_msgs/Point goal`, `geometry_msgs/Vector3 velocity`. Response: `geometry_msgs/Point[] path`, `bool success` (`success` is false when the resulting path has fewer than 2 points). Wraps `PRM::findSimplifiedPath`. |

### Visualization output

- `visualize_prm` — an `mrs_lib::BatchVisualizer` instance (topic prefix `visualize_prm`), owned entirely by `PRMNodelet` (`PRM` has no knowledge of it at all). Publishes the current PRM roadmap zone (as a translucent cuboid), all roadmap nodes (as points), and all node-to-node edges (as red rays). Built and published entirely from a dedicated background thread (`PRMNodelet::visualizationWorker`), decoupled from `timerUpdate` so the (comparatively expensive) marker construction never blocks roadmap maintenance.

`addNode`/`updateZone` themselves are **not** exposed as services — they only run from the nodelet's own subscription callbacks and its own update timer, never called by `Explorer` or any other external node.

## 3. Important functions/classes

- **`PRMNodelet`** (`src/prm_nodelet.cpp`) — the nodelet. Owns the `PRM` instance, all subscriptions, the update timer, the `find_simplified_path` service, and (unlike `PRM`) the `mrs_lib::BatchVisualizer` (`bv_prm_`) itself. Drives the `PRM` roadmap entirely from its own inputs, and keeps all `bv_prm_` marker construction + publish off the timer thread via a dedicated background thread (`vis_thread_`, started at the end of `onInit()`, joined in the `~PRMNodelet()` destructor) that is the *only* code touching `bv_prm_`/`bv_map_frame_set_`, so no locking is needed around them.
  - `onInit()` — loads parameters, sets up subscribers/service/visualizer, constructs the `PRM` instance, starts `vis_thread_`.
  - `~PRMNodelet()` — sets `vis_shutdown_`, notifies `vis_cv_`, and joins `vis_thread_`.
  - `callbackOctomap` — converts the incoming octomap message to an `OcTree` and stores it (mutexed) for the update timer; no longer touches `bv_prm_` (that responsibility moved into the worker).
  - `timerUpdate` (rate: `timer_rates/update` in `config/prm_solver.yaml`) — snapshots the current octree, computes the local zone around the UAV, calls `PRM::updateZone`, then hands off to `enqueueVisualization()` instead of publishing/clearing `bv_prm_` inline.
  - `enqueueVisualization` — under the existing `mutex_prm_` lock, copies each roadmap node's position/neighbor positions out of `prm_->nodes_` plus `prm_->currentZone()` into a lightweight `VisualizationSnapshot` (one `NodeVis` per node), then hands it to the worker thread by setting `pending_vis_snapshot_` and notifying `vis_cv_`.
  - `visualizationWorker` — runs on `vis_thread_` for the nodelet's entire lifetime: blocks on `vis_cv_` for the next `pending_vis_snapshot_`, then does all the `bv_prm_->addCuboid`/`addPoint`/`addRay` marker construction plus `publish()`/`clearBuffers()` for that snapshot (including the one-time `setParentFrame` call). Exits when `~PRMNodelet()` sets `vis_shutdown_` and notifies the condition variable.
  - `callbackFrontiers` — inserts a roadmap node at the first viewpoint of each frontier in every incoming `FrontierArray` message via `PRM::addNode`.
  - `callbackFindSimplifiedPath` — the `find_simplified_path_in` service handler; wraps `PRM::findSimplifiedPath`.
  - `getPosition` / `msgToMap` — small helpers for UAV pose lookup and octomap-message decoding.
- **`PRM`** (`include/prm_solver/prm.hpp` + `src/prm.cpp`) — the roadmap/search class, independent of ROS nodelet machinery aside from logging (`ROS_INFO`/`ROS_WARN`). It has no knowledge of visualization whatsoever — its constructor takes only the roadmap tunables (no `BatchVisualizer` parameter), and `updateZone` does no marker drawing/publishing; that responsibility belongs entirely to `PRMNodelet` now.
  - `nodes_` — the roadmap's node vector (`std::vector<std::shared_ptr<node_t>>`), public rather than private specifically so external callers can build their own visualization snapshot from it, the same pattern as `FrontierManager::fis_c_` in `frontier_detection`.
  - `updateZone(tree, zone, map_update)` — ages/invalidates/removes roadmap nodes inside `zone` and resamples new ones to keep node density up to a volume-based budget; nodes with fewer neighbors are aged more slowly to preserve long-range connectivity. Called on every new map (`map_update=true`) and on the nodelet's own timer (`map_update=false`); also stores `zone` as `zone_` for `currentZone()`.
  - `currentZone()` — public getter returning the zone last passed to `updateZone()` (`zone_`), added purely so `PRMNodelet::enqueueVisualization` can rebuild the visualization state without `PRM` depending on any visualization type.
  - `addNode(position)` — inserts a new node at `position` (skipped if too close to an existing node) and wires it to nearby nodes that have a clear line of free space between them (up to `max_neighbors`).
  - `removeInvalidNodes()` — compacts the node vector, dropping nodes marked invalid and pruning dangling neighbor references.
  - `findCloseNodes(point, r)` — returns roadmap nodes within radius `r` of `point`, sorted nearest-first.
  - `findNodePath(start, goal)` — A* search over the roadmap graph between two existing nodes, Euclidean-distance heuristic.
  - `findPath(start, goal, velocity)` — finds the closest roadmap nodes to `start`/`goal` (candidate start nodes are biased by `velocity` direction when nonzero) and runs `findNodePath` between them; returns an empty path if the goal isn't in free space or no roadmap nodes are reachable near either endpoint.
  - `findSimplifiedPath(start, goal, velocity)` — `findPath()` followed by a forward+reverse `simplifyFreeSpacePath()` pass to shortcut the route; this is the only method called from outside `PRM`.
  - `simplifyFreeSpacePath(path)` — shortcuts a path by merging waypoints whenever the straight segment between them stays in free space for the drone's size (`free_space_diameter_`).
  - `simplifyRaycastPath(path)` — **dead/unused**: shortcuts a path using raw octomap raycasts (line-of-sight only, ignoring drone size); not called anywhere in the package.
  - `distance(start, goal)` — **dead/unused**: total length of a simplified `findPath()` route; not called anywhere.
  - `extraDistance(start, goal)` — **dead/unused**: computes distance/height-change/turn-angle/velocity-change statistics along a simplified `findPath()` route; not called anywhere.

## 4. File structure

```
prm_solver/
├── CMakeLists.txt                        # catkin build config; builds MrsPrmSolver from prm.cpp + prm_nodelet.cpp
├── package.xml                           # package manifest/dependencies
├── nodelets.xml                          # pluginlib export: prm_solver/PRM -> prm_solver::PRMNodelet
├── config/
│   └── prm_solver.yaml                   # zone sizes, PRM tuning params, viz scale, update timer rate
├── launch/
│   └── prm_solver.launch                 # standalone/nodelet-manager launch, topic/service remaps
├── srv/
│   └── FindSimplifiedPath.srv            # start/goal/velocity -> path/success; the sole external service
├── include/prm_solver/
│   └── prm.hpp                           # PRM class + node_t/path_t/a_start_node_t/customLess A* support types
└── src/
    ├── prm.cpp                           # PRM implementation: roadmap maintenance + A* search + path simplification
    └── prm_nodelet.cpp                   # PRMNodelet: ROS glue (subscriptions, timer, service, visualization)
```

## 5. Dependencies

From `package.xml` / `CMakeLists.txt`:

- `roscpp`, `rospy`, `nodelet`, `pluginlib` (via `nodelets.xml`) — nodelet plumbing.
- `mrs_lib` — `SubscribeHandler`, `BatchVisualizer`, `ParamLoader`, `Transformer`, `mutex` helpers.
- `mrs_msgs` — `TrackerCommand`, `ControlManagerDiagnostics`.
- `octomap_msgs`, `octomap_ros`, and the system `octomap`/`OCTOMAP` library — map representation and (de)serialization.
- `message_generation`/`message_runtime`, `std_msgs`, `geometry_msgs` — for the package's own `FindSimplifiedPath.srv`.
- `frontier_detection` — **message-only dependency**: consumes `frontier_detection/FrontierArray` (and the nested `Frontier`/`Viewpoint` types) on `~frontiers_in`; does not link against or call into `frontier_detection`'s code.
- `octomap_planner_utils` — shared geometry/octomap helpers (`AABB`, `intersect`, `isFreeSpace`, `getSampleFromAABB`, `volume`, `localZoneFromPosition`, `getPosition`, `path_info_t`, `INVALID_DISTANCE`, etc.), used throughout `PRM` and `PRMNodelet`.
- `cmake_modules`, `Eigen3` — linear algebra (`Eigen::MatrixXd cost_matrix_`, visualization geometry); note `cost_matrix_` is allocated but otherwise unused in the current code.
- `pcl` (`point_types`, `register_point_struct`, `kdtree_flann`) — included in `prm.hpp` but the only PCL-based code (a `PointXYZIDX` point type + KD-tree scaffold) is commented out; not currently exercised.
