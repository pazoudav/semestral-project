# skeleton_estimator

## Purpose

`skeleton_estimator` is a standalone ROS1 nodelet package that extracts a ROSA
(Rotational Symmetry Axis) curve-skeleton from the live octomap point cloud of a
scanned structure (e.g. a wind turbine), aligns a prerecorded source skeleton to it,
and maps an offline-precomputed coverage path into the live world. The precomputed
path (`skeleton_estimator/precomputed_path`) is consumed by `explorer`
(`~precomputed_path_in`), which follows it into unknown space once it is stable
(`makePathPrecomputed()`); the transformed candidate viewpoints feed `tsp_solver` as its
*guiding* set (KD-tree; each tour is restricted to frontiers within `guide_radius` of them until `explorer` switches guiding off via `set_guiding`). The algorithm is a **from-scratch port**, not a vendored copy, of
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
| `~octomap_in` | `octomap_server/octomap_global_binary` | `octomap_msgs/Octomap` | Global octomap for the RANSAC free-space penalty; only subscribed when `alignment/free_space/enabled`. |
| `~source_skeleton_in` | `/source_skeleton` | `visualization_msgs/Marker` (LINE_LIST) | Prerecorded "source" skeleton, replayed manually (see `tmux/session.yml`, outside this package), used as the RANSAC alignment reference. |
| `~target_skeleton_in` | `~/extracted_skeleton` | `visualization_msgs/Marker` (LINE_LIST) | The live skeleton this node itself just published on `~skeleton_lines_out`; triggers RANSAC alignment against the source skeleton. |
| `~source_viewpoints_in` | `/source_viewpoints` | `visualization_msgs/MarkerArray` | Prerecorded candidate viewpoints associated with the source skeleton. Each `init_vps_pos` marker's position is the viewpoint and its `pose.orientation` encodes the surface normal (`normal = orientation * UnitX`, pointing from the surface toward the viewpoint; the viewpoint looks along `-normal`). Data is produced by `scripts/sample_surface.py` + `scripts/ply_to_marker_array.py`, sampled on the surface (`--offset 0`). |

### Published

| Topic (private name) | Remapped to | Type | Purpose |
|---|---|---|---|
| `~input_cloud_out` | `~/input_cloud` | `sensor_msgs/PointCloud2` | Debug: the filtered cloud fed into the ROSA pipeline. |
| `~skeleton_points_out` | `~/skeleton_points` | `sensor_msgs/PointCloud2` | The ROSA result's referenced vertices, as points. |
| `~skeleton_lines_out` | `~/extracted_skeleton` | `visualization_msgs/Marker` (LINE_LIST) | The ROSA skeleton edges — the main output. This node's own `~target_skeleton_in` loops back through it. |
| `~skeleton_vertex_ids_out` | `~/skeleton_vertex_ids` | `visualization_msgs/MarkerArray` (TEXT_VIEW_FACING) | Debug: per-vertex ID text markers, reimplemented locally instead of depending on FC-Planner's `traj_utils::PlanningVisualization`. |
| `~transformed_skeleton_out` | `~/transformed_skeleton` | `visualization_msgs/Marker` (LINE_LIST) | The source skeleton's line points, transformed by the RANSAC alignment `T` into the target/live frame. |
| `~candidate_viewpoints_out` | `~/candidate_viewpoints` | `sensor_msgs/PointCloud2` (PointNormal: x y z normal_x normal_y normal_z curvature) | The source's candidate viewpoints, transformed by `T` into the live frame (normals rotated and renormalized), then shifted `alignment/viewpoint_offset` meters along their normals; only the shifted points are published — feeds `tsp_solver`'s `~candidate_viewpoints_in` (remapped to `skeleton_estimator/candidate_viewpoints`), which builds a KD-tree over them as the guiding set restricting tour node selection (`guide_radius`) until `explorer` disables guiding. |
| `~precomputed_path_out` | `~/precomputed_path` | `skeleton_estimator/PrecomputedPath` (latched) | Per-alignment RANSAC estimate (`scale`, `yaw`, `translation`, `height`), stability fields (`stable`, `history_size`, `max_delta_{height,yaw,translation}`), informational `mean_residual` (`sqrt(cost/N)`, truncated RMS NN distance in world meters) and `observed_ratio`, and the selected path (`path_id`, `path_height`, `path_scale`, `applied_scale`, `waypoints[]` of `skeleton_estimator/Waypoint`: position, yaw, `is_viewpoint`). Check `stable` before using. Path fields are empty (`path_id` -1) when no path set is loaded. |
| `~precomputed_path_vis_out` | `~/precomputed_path_vis` | `visualization_msgs/MarkerArray` (latched) | Path line strip (red = unstable, green = stable), a yaw arrow per viewpoint, and a status text. |
| `~timing/targetSkeletonCallback/{load_target,candidates,ransac,viewpoints,skeleton,precomputed_path,total}` | same | `std_msgs/Float64` | Wall-clock time (ms) of each phase of `targetSkeletonCallback`, published via `explorer_utils::TimingPublisher`. |
| `~timing/callbackOctomap/total` | same | `std_msgs/Float64` | Wall-clock time (ms) of `callbackOctomap`. |

### Parameters (precomputed paths)

| Param | Purpose |
|---|---|
| `precomputed/paths_dir` | `data/<world_type>/paths` directory; set from the `paths_dir` launch arg (`${SKELETON_DATA_DIR}/paths` in `tmux/session.yml`). Empty/missing disables path output; the estimate is still published. |
| `precomputed/apply_residual_scale` | Also scale the selected path by estimate scale / path scale. |
| `precomputed/selection/margin` | [m] Hysteresis on height-bin selection: the current path is kept until another bin is closer to the estimated height by more than this; 0 = plain nearest bin. |
| `precomputed/stability/{window,eps_height,eps_yaw_deg,eps_trans}` | Stable when the last `window` estimates are all within the eps of the newest; re-evaluated every callback (no latch). |

### Parameters (transform smoothing)

| Param | Purpose |
|---|---|
| `alignment/blend/enabled` | Toggle the temporal filter on the RANSAC transform (`smoothTransform()`). |
| `alignment/blend/near_factor` | An estimate is "near" when within this factor x `precomputed/stability/eps_{height,yaw_deg,trans}` of the filtered transform. |
| `alignment/blend/rate` | Fraction of the way the filtered transform moves towards a near estimate per callback. |
| `alignment/blend/switch_after` | Consecutive far callbacks after which the new estimate is adopted outright. |

### Parameters (free-space penalty)

| Param | Purpose |
|---|---|
| `alignment/free_space/enabled` | Toggle the penalty (and the `~octomap_in` subscription). |
| `alignment/free_space/weight` | Penalty scale: `weight * score_truncation^2 * \|target\| * free_fraction`. |
| `alignment/free_space/margin` | [m] A source point counts as free only if its voxel is known free and no occupied voxel lies within this distance. |

### Parameters (yaw symmetry)

| Param | Purpose |
|---|---|
| `symmetry_order` | Optional int from the world's `world_info.yaml` (default 1 = none, clamped to >= 1): the structure's k-fold yaw symmetry about its vertical axis. For k > 1, equivalent poses (yaw vs yaw + 360/k) are folded to the variant closest to the current yaw. |

## Important functions/classes

- **`SkeletonEstimator`** (`src/skeleton_estimator_nodelet.cpp`) — the nodelet. Owns
  frame-skip/re-entrancy gating, drives `Rosa` per incoming octomap cloud, and
  separately handles RANSAC-based alignment of a prerecorded source skeleton against
  the live ROSA output:
  - `onInit()` — loads `RosaParams`/`world_frame_id`/`min_z`/`alignment/{min,max}_height_ratio`/`alignment/viewpoint_offset` (stand-off distance in meters, independent of RANSAC scale) from config and `zone_x`/`zone_y`/`zone_z` (plus optional `symmetry_order`, default 1) from the world's `world_info.yaml` (`world_info_file_config` launch arg; `rosa/norm_scale <= 0` falls back to `max(zone_x, zone_y, zone_z)`), wires up all subscribers/publishers.
  - `callbackOctomapPoints()` — port of `ROSA_main::pointCloudCallback`: drops incoming points below `min_z` (z-passthrough), runs `Rosa::run()`, publishes the resulting skeleton.
  - `publishInputCloud()` / `publishSkeleton()` — the two RViz-debug publishers reimplemented locally (originally via FC-Planner's `traj_utils::PlanningVisualization`).
  - `sourceSkeletonCallback()` / `sourceViewpointsCallback()` / `targetSkeletonCallback()` — cache the prerecorded source skeleton/viewpoints (viewpoints stored as `pcl::PointNormal`, normal taken from the marker orientation; deriving RANSAC scale bounds so the aligned skeleton's top lies within `[min,max]_height_ratio × zone_z`, and caching the skeleton's XY centroid as `symmetry_center_`) and, on each new live skeleton, RANSAC-align the two and republish the transformed viewpoints (via `pcl::transformPointCloudWithNormals`, normals renormalized, then offset along the normal)/skeleton.
  - `publishPrecomputedPath()` — called at the end of `targetSkeletonCallback()` (when RANSAC produced a transform): decomposes the transform into an estimate, calls `checkStability()`, selects/transforms a path via `PrecomputedPaths`, publishes `~precomputed_path_out`, then `publishPrecomputedPathVis()`. Uses `source_top_` (cached in `sourceSkeletonCallback()`) for the height/`observed_ratio`.
  - `loadSkeleton()` — resamples a LINE_LIST Marker's segments into a point cloud for RANSAC correspondence.
  - `runRANSAC()` / `estimateTransform()` — RANSAC search (each call's best joins a 10-entry history that is re-scored against the current target, sorted, worst dropped, best returned) for the best-fit XY similarity transform (scale + rotation + translation) between source and target skeleton point sets.
  - `smoothTransform()` (private) — called in `targetSkeletonCallback()` right after `runRANSAC()` (when the RANSAC history is non-empty); its output `T` feeds all downstream outputs (candidate viewpoints, transformed skeleton, precomputed path and stability check). Filters scale, yaw and XY translation: a near estimate moves the filtered transform `alignment/blend/rate` of the way towards it (yaw wrapped); a far one is held off, and adopted outright after `alignment/blend/switch_after` consecutive far callbacks. Filter state is reset in `sourceSkeletonCallback()`. Pass-through when `alignment/blend/enabled` is false.
  - `foldSymmetry(T, ref_yaw)` / `symmetryReferenceYaw()` (private) — with `symmetry_order_` k > 1, `foldSymmetry()` replaces `T` by `T * S_i` (S_i = yaw rotation by 2*pi*i/k about the vertical axis through `symmetry_center_`, the source skeleton's XY centroid cached in `sourceSkeletonCallback()`) so its yaw is the closest equivalent to `ref_yaw`; scale and z are preserved. `symmetryReferenceYaw()` returns the filtered yaw, else the best stored history hypothesis's yaw, else 0. `runRANSAC()` folds its best transform before storing it in the history (inverse recomputed), and `targetSkeletonCallback()` folds `T` before `smoothTransform()`, so symmetric flips don't disturb the blend/stability checks. No-op for k = 1.
  - `callbackOctomap()` — deserializes the global octomap into an immutable `FreeSpaceMap_t` snapshot (`octomap::OcTree` + KD-tree of occupied leaf centers) and swaps it in under `mutex_free_map_`.
  - `freeSpacePenalty()` (private) — fraction of transformed source skeleton points lying in known-free voxels with no occupied voxel within `free_margin_`, scaled by `weight * score_truncation^2 * |target|`. `runRANSAC()` adds it only when a hypothesis's alignment score alone beats the current best, and the history re-score includes it.
  - `scoreAlignment()` (private) — scores a hypothesis for both RANSAC and history re-scoring: sum over target skeleton points of `min((scale * d_src)^2, score_truncation^2)` (NN distance to the transformed source, in world meters; `alignment/score_truncation`, 4.0 m in the shipped config).

- **`Rosa`** (`include/skeleton_estimator/rosa.hpp` + `src/rosa.cpp`) — pure,
  non-ROS class implementing the ROSA algorithm (port of `predrecon::ROSA_main`),
  driven entirely through `setParams()` + `run()`:
  - `run()` — runs the full pipeline on a caller-filtered cloud, returns `false` if the input is too small (`est_num < 200`, used only as this bail-out gate) or `norm_scale <= 0`. Deterministic for identical input and identical temporal-prior state (no time-seeded RNG; ground `RandomSample` uses a fixed seed, `createOrthonormalFrame()` is a cross-product construction).
  - `applyTemporalPrior()` (private, called at the end of `run()` after `graph_` is assigned) — EMA-stabilizes the published skeleton against the previous frame: each edge-referenced vertex is matched to the previous run's published vertices (`prev_vertices_`) by mutual nearest neighbor within `rosa/temporal/match_radius` (m, default 2.0); matched vertices move to `prev + blend * (new - prev)` (`rosa/temporal/blend`, default 0.3), unmatched keep their raw position. Topology is untouched; `graph_.real_vertices` is rebuilt and stored as the next prior. Disabled via `rosa/temporal/enabled` (default true). Because of `prev_vertices_`, `run()` is **not** stateless across calls.
  - `pcloudReadOff()` — adopts the input cloud (and optionally synthesizes ground points).
  - `normalize()` — centers the cloud and scales it by the fixed `norm_scale` (`rosa/norm_scale`, metres; not the cloud's bounding box), so `radius`/`sample_r`/leaf/`delta_` keep a constant metric size as the map grows. The voxel leaf is `pt_downsample_size * norm_scale` rounded to a multiple of `input_resolution`, and the centroid is snapped to a multiple of the leaf so the voxel grid is world-anchored (unchanged map regions downsample to the same points). Sorts it lexicographically by (z, y, x) (before normal estimation and again after the final voxel downsample, so output is independent of input point order), estimates normals, voxel-downsamples with the fixed leaf. `RosaParams::input_resolution` (`rosa/input_resolution`, default 0.2 m) is the octomap resolution.
  - `orientNormalsConsistently()` (private, called at the end of `normalEstimation()`) — orients normals via MST propagation over a symmetric k-NN graph (k = `estimation_number`, edge weight `1 - |ni·nj|`); each component is seeded at its point farthest from the centroid and oriented towards the centroid.
  - `pcloudAdjMatrixMahalanobis()` — builds the Mahalanobis-weighted point adjacency (`P_.neighs`).
  - `rosaDrosa()` (dROSA) — iteratively estimates and smooths per-point symmetry normals, then projects points onto their symmetry plane to get initial ROSA positions.
  - `rosaDcrosa()` (dcROSA) — contracts ROSA positions further via neighbor-averaging and, only if `rosa/dcrosa_shrink` (default false; the ported shrink step was a mathematical no-op), PCA-based shrinking onto the local principal line with a soft weight ramping 0->1 between `rosa/shrink_conf_low` (0.4) and `rosa/shrink_conf_high` (0.6).
  - `rosaLineextract()` — clusters ROSA positions into discrete skeleton vertices and collapses triangles in their adjacency graph into a tree/graph structure.
  - `rosaRecenter()` — recenters and prunes low-support skeleton vertices.
  - `restoreScale()` / `storeRealGraph()` — map the skeleton back to original coordinates and extract the edge-referenced vertex subset that gets published.
  - `graph()` — accessor for the resulting `SkeletonGraph` (vertices/edges/real_vertices).

- **`PrecomputedPaths`** (`include/skeleton_estimator/precomputed_paths.hpp` + `src/precomputed_paths.cpp`) —
  plain C++ (no ROS) holder of offline-precomputed coverage paths (produced by
  `scripts/precompute_paths/`): `load()` reads `data/<world_type>/paths/index.yaml` and each
  `h<H>/{order,viewpoints}.txt`; `select(est, current, margin)` picks the path whose height bin is
  closest to the estimated height, but keeps `current` unless another bin is closer by more than
  `margin` (hysteresis; the nodelet passes its last pick `selected_path_`, reset in
  `sourceSkeletonCallback()`, and `precomputed/selection/margin`); `transform()` maps it into the live world (yaw, scale, translation);
  `appliedScale()` gives the scale used; `warnings()` lists problems from the last `load()`.

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
├── CMakeLists.txt                            # catkin build config; builds SkeletonEstimator from the 4 src/ files, generates msgs
├── package.xml                                # package manifest / dependencies
├── nodelets.xml                                # pluginlib nodelet class export (skeleton_estimator/SkeletonEstimator)
├── msg/
│   ├── Waypoint.msg                            # position, yaw, is_viewpoint
│   └── PrecomputedPath.msg                     # estimate, stability, selected path waypoints
├── data/<world_type>/                          # (turbine, pylon) prerecorded skeleton/viewpoints .outmsg + paths/ (index.yaml with h_base and per-height entries, h<H>/{order,viewpoints}.txt plus FC-Planner logs)
├── config/
│   └── skeleton_estimator.yaml                # ROSA algorithm parameters (radius, num_drosa, k_KNN, alpha, ...)
├── launch/
│   └── skeleton_estimator.launch               # standalone/managed nodelet launch, topic remaps, config + world_info.yaml loading
├── include/skeleton_estimator/
│   ├── rosa.hpp                                # Rosa class + RosaParams/SkeletonGraph, the ported ROSA_main pipeline
│   ├── extra_del.hpp                            # ExtraDel: Eigen row/column extract/delete helpers
│   ├── data_wrapper.hpp                         # DataWrapper: flattened xyz buffer accessor
│   └── precomputed_paths.hpp                    # PrecomputedPaths: load/select/transform offline paths
└── src/
    ├── skeleton_estimator_nodelet.cpp            # SkeletonEstimator nodelet (ROS glue, RANSAC alignment, precomputed path output)
    ├── precomputed_paths.cpp                     # PrecomputedPaths implementation
    ├── rosa.cpp                                  # Rosa pipeline implementation
    └── extra_del.cpp                              # ExtraDel implementation
```

## Dependencies

From `package.xml` / `CMakeLists.txt`:

- `roscpp`, `rospy`, `nodelet`, `pluginlib` — core ROS1 nodelet plumbing.
- `explorer_utils` — `TimingPublisher` (`explorer_utils/timing.hpp`), owned by the nodelet as `timing_`.
- `mrs_lib` — `ParamLoader`/`SubscribeHandler` used for config loading and topic subscriptions.
- `octomap_msgs`, `octomap` — global octomap message and `OcTree` for the free-space penalty.
- `std_msgs`, `geometry_msgs`, `sensor_msgs`, `visualization_msgs` — message types.
- `message_generation` / `message_runtime` — build/run support for this package's own `Waypoint`/`PrecomputedPath` messages.
- `yaml-cpp` — `PrecomputedPaths` reads `index.yaml`.
- `pcl_conversions`, `pcl_ros`, and `PCL` (via `find_package(PCL REQUIRED)`) — point cloud I/O, filtering (`PassThrough`, `VoxelGrid`, `RandomSample`), normal estimation, and KD-tree search.
- `Eigen3` — all ROSA linear algebra (SVD, dense matrix ops).
- `cmake_modules` — CMake find-module support for the above.

Despite being a port of FC-Planner's ROSA extraction algorithm, this package does
**not** depend on FC-Planner or its `traj_utils` package (used upstream only for two
debug publishers, which are reimplemented locally here instead). It is not declared
anywhere in this workspace's `ros_packages/`, and none of FC-Planner's sources are
referenced or vendored.
