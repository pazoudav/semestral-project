# frontier_detection

## Purpose

`frontier_detection` is a standalone ROS1 nodelet that finds the boundary between known-free and unknown space in a live Octomap and turns it into flyable observation points. As the UAV explores, it repeatedly extracts clusters of "frontier" cells (unknown cells adjacent to free space) within a local zone around the drone, discards clusters that are no longer real frontiers, splits overly large/elongated ones, and samples candidate viewpoints for each remaining cluster, scored by how many frontier cells they can see. The result is published as a `frontier_detection/FrontierArray` and consumed independently by `mrs_octomap_planner` (to trigger a TSP re-solve), `prm_solver` (roadmap node insertion), and `tsp_solver` (TSP ordering) — this package has no knowledge of any of them and performs no removal-by-id feedback; a frontier disappears from the feed only once `FrontierManager::removeFrontiers()` itself determines it is no longer a frontier.

## ROS interface

### Subscribed topics (private, remapped in `launch/detector.launch`)

| Handle | Remapped to | Type | Purpose |
|---|---|---|---|
| `~octomap_local_in` | `octomap_server/octomap_local_binary` | `octomap_msgs/Octomap` | Local-zone octomap; the sole map input and driver of the whole pipeline (`callbackOctomapLocal`). Each incoming message is decoded and merged into the persistent `octree_` (seeding it on the first message, merging via `octomap_planner_utils::mergeInto` thereafter), which is then used both for the UAV start-position occupancy check and as the tree passed into `FrontierManager::processNewMap` for the actual frontier BFS/extraction. |
| `~tracker_cmd_in` | `control_manager/tracker_cmd` | `mrs_msgs/TrackerCommand` | Used (with the diagnostics below) to determine the UAV's current reference position. |
| `~diagnostics_in` | `control_manager/diagnostics` | `mrs_msgs/ControlManagerDiagnostics` | Only used for freshness/validity checks by the `SubscribeHandler`; the callback body is empty. |

### Published topics

| Handle | Remapped to | Type | Purpose |
|---|---|---|---|
| `~frontiers_out` | `frontier_detection/frontiers` | `frontier_detection/FrontierArray` | One entry per valid frontier cluster, each carrying only its single best (highest-coverage) viewpoint. |
| (internal) `visualize_frontiers` | — | via `mrs_lib::BatchVisualizer` | Frontier cell cubes, cluster centers, viewpoint cubes, and the current search-zone wireframe, for RViz. Built and published entirely from a dedicated background thread (`Detector::visualizationWorker`), decoupled from the octomap-processing pipeline so marker construction never blocks frontier detection. |

### Custom messages

- **`Viewpoint.msg`** — a single candidate observation point: `geometry_msgs/Point position`, `int32 coverage` (number of frontier cells visible from it).
- **`Frontier.msg`** — one frontier cluster: `uint32 id`, `Viewpoint[] viewpoints` (only the best one is ever populated by the publisher), `uint32 size` (cell count).
- **`FrontierArray.msg`** — `Header header` + `Frontier[] frontiers`, the full feed published on `frontiers_out`.

## Important functions/classes

- **`Detector`** (`frontier_detector.cpp`) — the nodelet itself. Loads config, wires up subscribers/publisher, and on every new local-zone octomap: merges it into a persistent, ever-growing working tree (`octree_`), resolves the UAV's current position, checks the start position is free in `octree_`, computes a local search zone around it (`octomap_planner_utils::localZoneFromPosition`), drives `FrontierManager::processNewMap` against `octree_`, publishes the `FrontierArray`, and hands a visualization snapshot off to a dedicated background thread. It owns the `mrs_lib::BatchVisualizer` (`bv_frontiers_`) itself — `FrontierManager` has no knowledge of it at all — via a `vis_thread_` (started at the end of `onInit()`, joined in the `~Detector()` destructor) that is the *only* code touching `bv_frontiers_`/`bv_map_frame_set_`, so no locking is needed around them. It also runs a second background thread, `tree_cleanup_thread_`, that owns destroying retired octrees off the callback thread.
  - `callbackOctomapLocal` — the sole per-map callback (on `~octomap_local_in`) and the driver of the whole pipeline; there is no longer a `~octomap_in`/`callbackOctomap`/`octree_local_`. Decodes the incoming local-zone message inline via `octomap_msgs::msgToMap` (dynamic-cast into a short-lived local `OcTreeUniquePtr_t`, bailing out if it comes back null); under `mutex_octree_`, seeds the persistent `octree_` member from it directly if `octree_` is still null (first message), otherwise merges its leaves into `octree_` via `octomap_planner_utils::mergeInto(local_tree, octree_)` — overwriting the corresponding voxels in `octree_` with the fresher local values while leaving everything outside the local map's extent untouched. The now-merged decoded local tree is then handed to `retireTree()` for destruction off this thread (it is never itself kept around as a member). `octree_` — a persistent, ever-growing map assembled purely from these local-zone merges — is then used both for the UAV start-position occupancy check and as the tree passed into `FrontierManager::processNewMap` (which runs on every call, not just some). Ends by calling `publishFrontiers()` then `enqueueVisualization()` — neither call itself does any (potentially slow) `BatchVisualizer` marker work.
  - `publishFrontiers` — builds the outgoing `FrontierArray`, keeping only the single best viewpoint per frontier.
  - `enqueueVisualization` — under the existing `mutex_frontiers_` lock, copies `FrontierManager::fis_c_`'s centers/colors/viewpoints/cells (plus `minCoverage()`/`currentZone()`) into a lightweight `VisualizationSnapshot`, then hands it to the worker thread by setting `pending_vis_snapshot_` and notifying `vis_cv_`.
  - `visualizationWorker` — runs on `vis_thread_` for the nodelet's entire lifetime: blocks on `vis_cv_` for the next `pending_vis_snapshot_`, then does all the `bv_frontiers_->addPoint`/`addCuboid`/`publish`/`clearBuffers`/`setParentFrame` work for that snapshot. Exits when `~Detector()` sets `vis_shutdown_` and notifies the condition variable.
  - `retireTree` — hands a retired octree (the temporary decoded local tree, once merged into `octree_`) off to `tree_cleanup_thread_` instead of letting it be destroyed on the caller's (octomap-callback) thread.
  - `treeCleanupWorker` — runs on `tree_cleanup_thread_` for the nodelet's entire lifetime: blocks on `tree_cleanup_cv_` for retired octrees, then actually destroys them (freeing every node — expensive for a large map) off the octomap-callback thread. Exits when `~Detector()` sets `tree_cleanup_shutdown_` and notifies the condition variable.
  - `getPosition` — resolves the UAV reference position from tracker command + diagnostics via `octomap_planner_utils::getPosition`.
- **`FrontierManager`** (`frontier_manager.hpp`/`.cpp`) — owns the set of frontier clusters (`FIS`) inside the active zone. Has no knowledge of visualization whatsoever — its constructor takes only the sampling/scoring tunables (no `BatchVisualizer` parameter), and `processNewMap` never builds any markers; instead it exposes `minCoverage()`/`currentZone()` getters so a caller (`Detector::enqueueVisualization`) can reconstruct what `processNewMap` would have drawn. Its `tree_` member is a non-owning raw `octomap::OcTree*`, valid only for the duration of the `processNewMap()` call that sets it — the octree's actual lifetime is owned solely by `Detector::octree_`; the one internal call into `octomap_planner_utils::isFreeSpace` (which takes a `shared_ptr`) wraps `tree_` in a temporary no-op-deleter `shared_ptr` just for that call. It also holds two small scratch buffers, both sized/reset in `setZone` alongside `global_closed_`, kept around purely to avoid reallocating on every call: `frontier_stamp_`/`current_frontier_epoch_` — an epoch-stamped closed-set used by `frontierSearch()`'s inner per-cluster BFS instead of allocating and zeroing a fresh zone-sized `vector<bool>` for every discovered cluster — and `ray_scratch_`, a single `octomap::KeyRay` reused across all of `viewpointCoverage()`'s raycasts instead of default-constructing a fresh one (which reserves a 100k-element buffer) per frontier cell.
  - `processNewMap(octomap::OcTree* tree, ...)` — full update pass: borrows the raw tree pointer for the call's duration, removes stale frontiers, BFS-extracts newly discovered frontier cell clusters starting from the UAV's cell, adds/splits them, and (re)samples viewpoints for clusters intersecting the current zone.
  - `removeFrontiers` — drops frontiers that fail `isStillFrontier`, growing the search zone to re-cover the space they vacated, and marks surviving frontiers' cells as already-explored so the BFS doesn't rediscover them.
  - `isStillFrontier` — re-validates a previously found cluster: false if it shrank below `min_frontier_size_` or if fewer than `size_decrease_ratio_` of its cells are still genuine frontier cells.
  - `isFrontierCell` — true if a cell is unknown and has at least one free 6-connected neighbor.
  - `keyInZone` — true if a key lies within `[min_key_, max_key_]`; a direct key-bounds check used in `frontierSearch()`'s neighbor loops in place of converting the key to a coordinate and calling `intersect(zone_, ...)`.
  - `addFrontier` — adds a cluster as a new `FIS`; if it's too large, splits it in two via SVD (along the dominant axis) and recurses.
  - `makeViewpoints` — samples candidate viewpoints around a `FIS`, keeping only ones in free space with enough surrounding clearance and sufficient coverage, sorted best-first.
  - `viewpointCoverage` — counts how many of a frontier's cells are within max range/angle of a viewpoint and reachable by an unoccluded octomap raycast; casts each ray into the reused `ray_scratch_` buffer rather than a freshly-constructed `KeyRay`.
  - `minCoverage` / `currentZone` — public getters exposing `min_coverage_` and `zone_` (the latter as last set by `processNewMap`/`removeFrontiers`), added purely so `Detector::enqueueVisualization` can rebuild the visualization state without `FrontierManager` depending on any visualization type.
- **`FIS`** (`fis.hpp`/`.cpp`) — Frontier Information Structure: one clustered set of frontier cells plus its centroid, bounding box, and sampled viewpoints.
  - `sampleViewpoint` — randomly samples a point at a given horizontal radius/vertical offset around the cluster's center.
- **Package-local `frontier_detection/utils.hpp`/`.cpp`** — small standalone helpers (`frontier_t` = `std::vector<octomap::point3d>`, `mean` (takes its `frontier_t` by const reference), `getMinBound`, `getMaxBound`), distinct from and not to be confused with the shared `octomap_planner_utils` library.

## File structure

```
frontier_detection/
├── CMakeLists.txt                  # builds MrsFrontierDetection nodelet library + generates the msg/ types
├── package.xml                     # catkin package manifest / dependencies
├── nodelets.xml                    # pluginlib export: frontier_detection/Detector -> nodelet::Nodelet
├── launch/
│   └── detector.launch             # standalone launch for the Detector nodelet (topic remaps, config load)
├── config/
│   └── detector.yaml               # zone sizes, frontier/viewpoint sampling & scoring thresholds, viz scales
├── msg/
│   ├── Viewpoint.msg                # a candidate observation point + its coverage score
│   ├── Frontier.msg                 # one frontier cluster: id, size, best viewpoint(s)
│   └── FrontierArray.msg            # header + Frontier[] - the published feed
├── include/frontier_detection/
│   ├── fis.hpp                      # FIS class + viewpoint_t: one frontier cluster and its sampled viewpoints
│   ├── frontier_manager.hpp         # FrontierManager: BFS frontier extraction/removal/viewpoint sampling over a zone
│   └── utils.hpp                    # package-local helpers: frontier_t typedef, mean/getMinBound/getMaxBound
└── src/
    ├── frontier_detector.cpp        # Detector nodelet: ROS glue (subs/pub, params, per-map pipeline driver)
    ├── frontier_manager.cpp         # FrontierManager implementation
    ├── fis.cpp                      # FIS implementation
    └── utils.cpp                    # implementation of the package-local helpers above
```

Note: this package has exactly one `utils.hpp`/`utils.cpp` pair (`frontier_detection` namespace, holding just `frontier_t`/`mean`/`getMinBound`/`getMaxBound`). It is separate from — and not a duplicate of — the shared `octomap_planner_utils::utils.hpp`/`.cpp` library that this package also depends on and fully qualifies via `octomap_planner_utils::`.

## Dependencies

From `package.xml` / `CMakeLists.txt`:

- **`octomap_planner_utils`** — this project's shared geometry/octomap helper library (`AABB`, `intersect`, `makeUnion`, `isFreeSpace`, `getNeighbourKey`/`NEIGHBOUR_OFFSETS`, `getColor`, `getPosition`, `localZoneFromPosition`, `mergeInto`, etc).
- **`mrs_lib`** — `SubscribeHandler`, `BatchVisualizer`, `ParamLoader`, `Transformer`, mutex helpers.
- **`mrs_msgs`** — `TrackerCommand`, `ControlManagerDiagnostics`.
- **`nodelet`** / **`roscpp`** / **`rospy`** — nodelet + ROS core.
- **`octomap_msgs`** / **`octomap_ros`** — Octomap message (de)serialization; plus `find_package(octomap)` for the native `octomap`/`OCTOMAP_LIBRARIES` C++ API.
- **`dynamicEDT3D`** (`find_package` in CMakeLists) — linked but not directly used by this package's own sources.
- **`message_generation`/`message_runtime`**, **`std_msgs`**, **`geometry_msgs`** — for the custom `Viewpoint`/`Frontier`/`FrontierArray` messages.
- **`visualization_msgs`** — for `BatchVisualizer`-driven RViz markers.
- **`Eigen3`** — SVD-based frontier splitting in `FrontierManager::addFrontier`.
- **`cmake_modules`** — CMake find-module support pulled in via catkin.

`CATKIN_ENABLE_TESTING` is explicitly disabled; there is no test suite for this package.
