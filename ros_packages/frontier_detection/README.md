# frontier_detection

## Purpose

`frontier_detection` is a standalone ROS1 nodelet that finds the boundary between known-free and unknown space in a live Octomap and turns it into flyable observation points. As the UAV explores, it repeatedly extracts clusters of "frontier" cells (unknown cells adjacent to free space) within a local zone around the drone, discards clusters that are no longer real frontiers, splits overly large/elongated ones, and samples candidate viewpoints for each remaining cluster, scored by how many frontier cells they can see. The result is published as a `frontier_detection/FrontierArray` and consumed independently by `mrs_octomap_planner` (to trigger a TSP re-solve), `prm_solver` (roadmap node insertion), and `tsp_solver` (TSP ordering) — this package has no knowledge of any of them and performs no removal-by-id feedback; a frontier disappears from the feed only once `FrontierManager::removeFrontiers()` itself determines it is no longer a frontier.

## ROS interface

### Subscribed topics (private, remapped in `launch/detector.launch`)

| Handle | Remapped to | Type | Purpose |
|---|---|---|---|
| `~octomap_in` | `octomap_server/octomap_global_full` | `octomap_msgs/Octomap` | Incremental/full octomap; drives the whole processing pipeline (`callbackOctomap`). |
| `~tracker_cmd_in` | `control_manager/tracker_cmd` | `mrs_msgs/TrackerCommand` | Used (with the diagnostics below) to determine the UAV's current reference position. |
| `~diagnostics_in` | `control_manager/diagnostics` | `mrs_msgs/ControlManagerDiagnostics` | Only used for freshness/validity checks by the `SubscribeHandler`; the callback body is empty. |

### Published topics

| Handle | Remapped to | Type | Purpose |
|---|---|---|---|
| `~frontiers_out` | `frontier_detection/frontiers` | `frontier_detection/FrontierArray` | One entry per valid frontier cluster, each carrying only its single best (highest-coverage) viewpoint. |
| (internal) `visualize_frontiers` | — | via `mrs_lib::BatchVisualizer` | Frontier cell cubes, cluster centers, viewpoint cubes, and the current search-zone wireframe, for RViz. |

### Custom messages

- **`Viewpoint.msg`** — a single candidate observation point: `geometry_msgs/Point position`, `int32 coverage` (number of frontier cells visible from it).
- **`Frontier.msg`** — one frontier cluster: `uint32 id`, `Viewpoint[] viewpoints` (only the best one is ever populated by the publisher), `uint32 size` (cell count).
- **`FrontierArray.msg`** — `Header header` + `Frontier[] frontiers`, the full feed published on `frontiers_out`.

## Important functions/classes

- **`Detector`** (`frontier_detector.cpp`) — the nodelet itself. Loads config, wires up subscribers/publisher, and on every new octomap: decodes it, resolves the UAV's current position, computes a local search zone around it (`octomap_planner_utils::localZoneFromPosition`), drives `FrontierManager::processNewMap`, then publishes visualization and the `FrontierArray`.
  - `callbackOctomap` — main per-map update; the effective driver of the whole pipeline.
  - `publishFrontiers` — builds the outgoing `FrontierArray`, keeping only the single best viewpoint per frontier.
  - `getPosition` — resolves the UAV reference position from tracker command + diagnostics via `octomap_planner_utils::getPosition`.
  - `msgToMap` — decodes an `octomap_msgs/Octomap` (binary or full) into an `octomap::OcTree`.
- **`FrontierManager`** (`frontier_manager.hpp`/`.cpp`) — owns the set of frontier clusters (`FIS`) inside the active zone.
  - `processNewMap` — full update pass: removes stale frontiers, BFS-extracts newly discovered frontier cell clusters starting from the UAV's cell, adds/splits them, and (re)samples viewpoints for clusters intersecting the current zone.
  - `removeFrontiers` — drops frontiers that fail `isStillFrontier`, growing the search zone to re-cover the space they vacated, and marks surviving frontiers' cells as already-explored so the BFS doesn't rediscover them.
  - `isStillFrontier` — re-validates a previously found cluster: false if it shrank below `min_frontier_size_` or if fewer than `size_decrease_ratio_` of its cells are still genuine frontier cells.
  - `isFrontierCell` — true if a cell is unknown and has at least one free 6-connected neighbor.
  - `addFrontier` — adds a cluster as a new `FIS`; if it's too large, splits it in two via SVD (along the dominant axis) and recurses.
  - `makeViewpoints` — samples candidate viewpoints around a `FIS`, keeping only ones in free space with enough surrounding clearance and sufficient coverage, sorted best-first.
  - `viewpointCoverage` — counts how many of a frontier's cells are within max range/angle of a viewpoint and reachable by an unoccluded octomap raycast.
- **`FIS`** (`fis.hpp`/`.cpp`) — Frontier Information Structure: one clustered set of frontier cells plus its centroid, bounding box, and sampled viewpoints.
  - `sampleViewpoint` — randomly samples a point at a given horizontal radius/vertical offset around the cluster's center.
- **Package-local `frontier_detection/utils.hpp`/`.cpp`** — small standalone helpers (`frontier_t` = `std::vector<octomap::point3d>`, `mean`, `getMinBound`, `getMaxBound`), distinct from and not to be confused with the shared `octomap_planner_utils` library.

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

- **`octomap_planner_utils`** — this project's shared geometry/octomap helper library (`AABB`, `intersect`, `makeUnion`, `isFreeSpace`, `getNeighbourKey`/`NEIGHBOUR_OFFSETS`, `getColor`, `getPosition`, `localZoneFromPosition`, etc).
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
