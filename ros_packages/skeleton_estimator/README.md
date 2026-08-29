# skeleton_estimator

## Purpose

`skeleton_estimator` is a standalone ROS1 nodelet package that extracts a ROSA
(Rotational Symmetry Axis) curve-skeleton from the live octomap point cloud of a
scanned structure (e.g. a wind turbine). The skeleton is used elsewhere in this
workspace (`mrs_octomap_planner`'s `Explorer`) as a guide for sampling coverage
viewpoints. The algorithm is a **from-scratch port**, not a vendored copy, of
`predrecon::ROSA_main::pointCloudCallback` from the external
[FC-Planner](https://github.com/HKUST-Aerial-Robotics/FC-Planner) repository
(`~/user_ros_workspace/src/FC-Planner/src/rosa`, outside this workspace's `src/`),
trimmed down to exactly the code that callback's call graph reaches — FC-Planner's
own branch/segment-decomposition stages were already dead code (commented out
upstream, never called from `pointCloudCallback`) and were dropped, along with the
fields/params only they touched. The package depends only on `roscpp`/`nodelet`/
`mrs_lib`/PCL/Eigen — it does not pull in FC-Planner or its `traj_utils` stack.

## ROS interface

All topic names below are the private (`~`) names as declared in the nodelet; see
[launch/skeleton_estimator.launch](launch/skeleton_estimator.launch) for the actual
remaps used at runtime.

### Subscribed

| Topic (private name) | Remapped to | Type | Purpose |
|---|---|---|---|
| `~octomap_points_in` | `octomap_global_vis/octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` | Live octomap point cloud; triggers the ROSA pipeline. |
| `~source_skeleton_in` | `/source_skeleton` | `visualization_msgs/Marker` (LINE_LIST) | Prerecorded "source" skeleton, replayed manually (see `tmux/session.yml`, outside this package), used as the RANSAC alignment reference. |
| `~target_skeleton_in` | `/rosa_vis/rr_lines` | `visualization_msgs/Marker` (LINE_LIST) | The live skeleton this node itself just published on `~skeleton_lines_out`; triggers RANSAC alignment against the source skeleton. |
| `~source_viewpoints_in` | `/source_viewpoints` | `visualization_msgs/MarkerArray` | Prerecorded candidate viewpoints associated with the source skeleton. |

### Published

| Topic (private name) | Remapped to | Type | Purpose |
|---|---|---|---|
| `~input_cloud_out` | `/rosa_vis/input_cloud` | `sensor_msgs/PointCloud2` | Debug: the filtered cloud fed into the ROSA pipeline. |
| `~skeleton_points_out` | `/rosa_vis/rr_pts` | `sensor_msgs/PointCloud2` | The ROSA result's referenced vertices, as points. |
| `~skeleton_lines_out` | `/rosa_vis/rr_lines` | `visualization_msgs/Marker` (LINE_LIST) | The ROSA skeleton edges — the main output. This is the absolute topic `mrs_octomap_planner`'s `Explorer::targetSkeletonCallback` listens to as `target_skeleton_in`, and this node's own `~target_skeleton_in` also loops back through it. |
| `~skeleton_vertex_ids_out` | `/rosa_vis/vertex_ID` | `visualization_msgs/MarkerArray` (TEXT_VIEW_FACING) | Debug: per-vertex ID text markers, reimplemented locally instead of depending on FC-Planner's `traj_utils::PlanningVisualization`. |
| `~transformed_skeleton_out` | `transformed_skeleton` | `visualization_msgs/Marker` (LINE_LIST) | The source skeleton's line points, transformed by the RANSAC alignment `T` into the target/live frame. |
| `~ransac_skeleton_out` | `ransaced_skeleton` | `visualization_msgs/Marker` | Advertised but not currently published to from any callback in this package. |
| `~candidate_viewpoints_out` | `candidate_viewpoints` | `sensor_msgs/PointCloud2` | The source's candidate viewpoints, transformed by `T` into the live frame — feeds `Explorer`'s skeleton-guided viewpoint sampling. |

## Important functions/classes

- **`SkeletonEstimator`** (`src/skeleton_estimator_nodelet.cpp`) — the nodelet. Owns
  frame-skip/re-entrancy gating, drives `Rosa` per incoming octomap cloud, and
  separately handles RANSAC-based alignment of a prerecorded source skeleton against
  the live ROSA output:
  - `onInit()` — loads `RosaParams`/`world_frame_id` from config, wires up all subscribers/publishers.
  - `callbackOctomapPoints()` — port of `ROSA_main::pointCloudCallback`: filters the incoming cloud, runs `Rosa::run()`, publishes the resulting skeleton.
  - `publishInputCloud()` / `publishSkeleton()` — the two RViz-debug publishers reimplemented locally (originally via FC-Planner's `traj_utils::PlanningVisualization`).
  - `sourceSkeletonCallback()` / `sourceViewpointsCallback()` / `targetSkeletonCallback()` — cache the prerecorded source skeleton/viewpoints and, on each new live skeleton, RANSAC-align the two and republish the transformed viewpoints/skeleton.
  - `loadSkeleton()` — resamples a LINE_LIST Marker's segments into a point cloud for RANSAC correspondence.
  - `runRANSAC()` / `estimateTransform()` — RANSAC search (with a short history buffer for temporal smoothing) for the best-fit XY similarity transform (scale + rotation + translation) between source and target skeleton point sets.

- **`Rosa`** (`include/skeleton_estimator/rosa.hpp` + `src/rosa.cpp`) — pure,
  non-ROS class implementing the ROSA algorithm (port of `predrecon::ROSA_main`),
  driven entirely through `setParams()` + `run()`:
  - `run()` — runs the full pipeline on a caller-filtered cloud, returns `false` if the input is too small (`est_num < 200`).
  - `pcloudReadOff()` — adopts the input cloud (and optionally synthesizes ground points).
  - `normalize()` — centers/scales the cloud, estimates normals, voxel-downsamples to the target point count.
  - `pcloudAdjMatrixMahalanobis()` — builds the Mahalanobis-weighted point adjacency (`P_.neighs`).
  - `rosaDrosa()` (dROSA) — iteratively estimates and smooths per-point symmetry normals, then projects points onto their symmetry plane to get initial ROSA positions.
  - `rosaDcrosa()` (dcROSA) — contracts ROSA positions further via neighbor-averaging and PCA-based shrinking.
  - `rosaLineextract()` — clusters ROSA positions into discrete skeleton vertices and collapses triangles in their adjacency graph into a tree/graph structure.
  - `rosaRecenter()` — recenters and prunes low-support skeleton vertices.
  - `restoreScale()` / `storeRealGraph()` — map the skeleton back to original coordinates and extract the edge-referenced vertex subset that gets published.
  - `graph()` — accessor for the resulting `SkeletonGraph` (vertices/edges/real_vertices).

- **`ExtraDel`** (`include/skeleton_estimator/extra_del.hpp` + `src/extra_del.cpp`) —
  small Eigen matrix row/column extraction-and-deletion-by-index helper class, ported
  unchanged from FC-Planner's `predrecon::Extra_Del` (trimmed to the methods `Rosa`
  actually calls): `rowsExtM`, `rowsDelM`, `colsDelM`.

- **`DataWrapper`** (`include/skeleton_estimator/data_wrapper.hpp`) — thin wrapper
  over a flattened xyz point buffer (column-major: all x's, then y's, then z's), used
  by `Rosa::pcloudIsoncut`/`distanceQuery` for fast per-point access during
  cutting-plane queries.

## File structure

```
skeleton_estimator/
├── CMakeLists.txt                            # catkin build config; builds SkeletonEstimator from the 3 src/ files
├── package.xml                                # package manifest / dependencies
├── nodelets.xml                                # pluginlib nodelet class export (skeleton_estimator/SkeletonEstimator)
├── config/
│   └── skeleton_estimator.yaml                # ROSA algorithm parameters (radius, num_drosa, k_KNN, alpha, ...)
├── launch/
│   └── skeleton_estimator.launch               # standalone/managed nodelet launch, topic remaps, config loading
├── include/skeleton_estimator/
│   ├── rosa.hpp                                # Rosa class + RosaParams/SkeletonGraph, the ported ROSA_main pipeline
│   ├── extra_del.hpp                            # ExtraDel: Eigen row/column extract/delete helpers
│   └── data_wrapper.hpp                         # DataWrapper: flattened xyz buffer accessor
└── src/
    ├── skeleton_estimator_nodelet.cpp            # SkeletonEstimator nodelet (ROS glue, RANSAC alignment)
    ├── rosa.cpp                                  # Rosa pipeline implementation
    └── extra_del.cpp                              # ExtraDel implementation
```

## Dependencies

From `package.xml` / `CMakeLists.txt`:

- `roscpp`, `rospy`, `nodelet`, `pluginlib` — core ROS1 nodelet plumbing.
- `mrs_lib` — `ParamLoader`/`SubscribeHandler` used for config loading and topic subscriptions.
- `std_msgs`, `geometry_msgs`, `sensor_msgs`, `visualization_msgs` — message types.
- `pcl_conversions`, `pcl_ros`, and `PCL` (via `find_package(PCL REQUIRED)`) — point cloud I/O, filtering (`PassThrough`, `VoxelGrid`, `RandomSample`), normal estimation, and KD-tree search.
- `Eigen3` — all ROSA linear algebra (SVD, dense matrix ops).
- `cmake_modules` — CMake find-module support for the above.

Despite being a port of FC-Planner's ROSA extraction algorithm, this package does
**not** depend on FC-Planner or its `traj_utils` package (used upstream only for two
debug publishers, which are reimplemented locally here instead). It is not declared
anywhere in this workspace's `ros_packages/`, and none of FC-Planner's sources are
referenced or vendored.
