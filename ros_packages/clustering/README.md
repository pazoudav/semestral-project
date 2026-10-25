# clustering

## Purpose

`clustering` groups the live frontier set into clusters around the UAV, so `tsp_solver` orders
one representative frontier per cluster instead of every frontier. It runs as a standalone
nodelet (`Clustering`) between `frontier_detector` and `tsp_solver`: it consumes
`frontier_detector/frontiers`, feeds each frontier's first viewpoint (plus its id) into a
persistent `Tree` rooted at the UAV's current position, and republishes the root's direct
children (one existing frontier per cluster, unchanged, ids preserved) as a
`frontier_detector/FrontierArray` on `clustering/clusters`.

`tsp_solver.launch` remaps its `~frontiers_in` to `clustering/clusters`, so the TSP tour is
built over clusters; `explorer` and `path_planner` still subscribe to the raw
`frontier_detector/frontiers` feed. Because cluster heads are original frontiers with their
original ids, `explorer`'s per-frontier viewpoint cache (used by `refineLocalTour()`) still
matches the tour's ids.

## ROS interface

### Topics subscribed

| Topic (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~frontiers_in` | `frontier_detector/frontiers` | `frontier_detector/FrontierArray` | The frontier set to cluster; each message triggers one `Tree::update` + publish (`callbackFrontiers`). |
| `~tracker_cmd_in` | `control_manager/tracker_cmd` | `mrs_msgs/TrackerCommand` | UAV position for the tree root, via `explorer_utils::getPosition`; times out (warns) after 3 s. |
| `~diagnostics_in` | `control_manager/diagnostics` | `mrs_msgs/ControlManagerDiagnostics` | Freshness check only (empty callback), used by `explorer_utils::getPosition`. |

### Topics published

| Topic (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~clusters_out` | `~/clusters` (→ `clustering/clusters`) | `frontier_detector/FrontierArray` | One frontier per root child of the tree (the last received copy of that frontier), header copied from the input message. Consumed by `tsp_solver`. |
| `~visualize_tree` | — | `mrs_lib::BatchVisualizer` markers | Every node below the root's children as a small point plus parent→child edges, one color per branch. |
| `~visualize_clusters` | — | `mrs_lib::BatchVisualizer` markers | The root's children (the clusters) as large points, same per-branch color. |
| `~timing/callbackFrontiers/total` | — | `std_msgs/Float64` | Wall-clock time of `callbackFrontiers` in ms, via `explorer_utils::TimingPublisher`. |

Both visualizers are shown in the RViz layout (`tmux/rviz.rviz`).

### Parameters (`config/clustering.yaml`)

| Parameter | Meaning |
|---|---|
| `tree/growth_factor` | Every radius of a node is scaled by `growth_factor^scale`, where `scale` is the node's integer step count. |
| `tree/expand_radius` | A root child increases its `scale` while the UAV is farther than this (scaled). |
| `tree/deflate_radius` | …and decreases it while the UAV is closer than this. Keep `<= expand_radius / growth_factor`, otherwise grow/shrink alternate every frame. |
| `tree/break_radius` | Children farther than this from their parent are handed back to the grandparent. |
| `tree/build_radius` | Siblings closer than this are merged under one of them. |
| `tree/root_radius` | Additions/removals are only processed within `2 * root_radius` of the UAV. |
| `viz/scale/{points,root_points,lines}` | Marker sizes for the two visualizers. |

## Important functions/classes

- **`clustering::Clustering`** (`clustering_nodelet.cpp`) — the nodelet. `onInit` loads `Params`, builds the `Tree`, and wires up subscribers, the clusters publisher, both `BatchVisualizer`s, a `mrs_lib::Transformer` and `timing_`. `callbackFrontiers` moves `tree_->root.pos` to the UAV, passes `(viewpoints[0].position, id)` of every frontier that has a viewpoint to `Tree::update`, drops cached frontiers whose id the tree no longer contains, publishes the root's children as clusters, and calls `visualizeTree`.
- **`clustering::Tree`** (`tree.hpp`/`tree.cpp`) — owns every `Node` (`nodes_`, keyed by frontier id) and the root. `update(points)` adds unseen ids as root children, removes nodes whose id was missing from this update **only if they lie within `2 * root_radius` of the root** (so callers may pass either the full set or just the nearby part), then for each root child adapts its radii (`double_radii`/`halve`) and runs `release_far_children` + `absorb_neighbours`. Non-copyable: nodes point at its `params` and `root`.
- **`clustering::Node`** (`node.hpp`/`node.cpp`) — a tree point with `parent`/`children`, a subtree `weight`, and an integer `scale`. Radii are derived as base radius × `growth[scale]`, so far-away branches get larger radii and merge more coarsely. `detach()` hands a removed node's children to its parent; `merge(group)` picks the group member nearest the weight-averaged position as head and re-parents the rest under it (skipping members the head would immediately release again).
- **`clustering::Params` / `Vec3`** (`node.hpp`) — tree radii plus a precomputed `growth` table (`update_growth()`, up to `MAX_SCALE = 32`), and a minimal 3D vector type with the arithmetic the tree needs.

## File structure

```
clustering/
├── CMakeLists.txt              # catkin build config; builds the Clustering nodelet library
├── package.xml                 # package manifest / dependencies
├── nodelets.xml                # pluginlib export: clustering/Clustering -> clustering::Clustering
├── README.md                   # this file
├── config/
│   └── clustering.yaml         # tree radii and visualization scales
├── launch/
│   └── clustering.launch       # standalone/nodelet-manager launch, topic remaps
├── include/clustering/
│   ├── node.hpp                # Vec3, Params, Node
│   └── tree.hpp                # Tree
└── src/
    ├── clustering_nodelet.cpp  # Clustering nodelet: ROS glue + visualization
    ├── node.cpp                # Node implementation (radii, release, merge)
    └── tree.cpp                # Tree::update
```

## Dependencies

From `package.xml`/`CMakeLists.txt`:
- **catkin deps**: `cmake_modules`, `mrs_lib`, `nodelet`, `pluginlib`, `roscpp`, `rospy`, `std_msgs`, `geometry_msgs`, `mrs_msgs`.
- **In-workspace package deps**: `frontier_detector` (`FrontierArray`/`Frontier` message types), `explorer_utils` (`getPosition` and `TimingPublisher`).
- The tree itself (`node.*`, `tree.*`) is plain C++17 with no ROS dependency.
