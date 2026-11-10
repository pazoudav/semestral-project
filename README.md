# Energy-aware UAV coverage path planning in scanning of outdoor structures

Autonomous UAV exploration/inspection of outdoor structures (wind turbines, pylons, bridges) on the
[CTU-MRS UAV stack](http://github.com/ctu-mrs/mrs_uav_core) (ROS1, Gazebo). The UAV explores frontiers
of a live octomap and, once it recognises the structure (skeleton + RANSAC alignment against a known
model), switches to following an offline-precomputed coverage path transformed into the live world.

Demo video: https://youtu.be/lA2REzf3Iso

## Running

This repo lives at `src/semestral-project` inside a catkin workspace that already contains the MRS
packages (`mrs_uav_core`, `mrs_lib`, `mrs_msgs`, gazebo simulation, ...).

```bash
./run.sh [-c|-b] <world_name>
```

- `-c` cleans the catkin workspace and builds, `-b` builds without cleaning, no flag just launches.
- `<world_name>` is a directory under `worlds/` containing `<world_name>.world`, `<world_name>.ply`
  (optionally `.dae`) and `world_info.yaml` (`world_type`, `spawn_position`, `zone_x/y/z`,
  optional `symmetry_order`). Only `worlds/example/` is checked in; generate `world_info.yaml` for new
  worlds with `scripts/worldbounds/worldbounds.py` (or `scripts/migrate_worlds.sh`).

`run.sh` starts the tmuxinator session `tmux/session.yml`: roscore, Gazebo, drone spawn, MRS hw_api/core,
`mapplan.launch` (point-cloud filters + octomap server), the project nodelets, takeoff and RViz.
Most behaviour is tuned without rebuilding via each package's `config/*.yaml`.

## Packages (`ros_packages/`)

Each package has its own detailed `README.md` (ROS interface tables, functions, parameters).

| Package | Role |
|---|---|
| [explorer](ros_packages/explorer/README.md) | Core decision maker: replanning state machine, precomputed-path GOTO/FOLLOW, frontier-tour path assembly, local viewpoint refinement, trajectory dispatch to MRS control. |
| [frontier_detector](ros_packages/frontier_detector/README.md) | Extracts frontier clusters and samples viewpoints in a local zone around the UAV; owns the `Frontier`/`FrontierArray`/`Viewpoint` msgs. |
| [clustering](ros_packages/clustering/README.md) | Distance-scaled cluster tree over frontier viewpoints; forwards one frontier per cluster. |
| [tsp_solver](ros_packages/tsp_solver/README.md) | Global viewpoint tour via the bundled LKH binary, edge costs from real roadmap path lengths; optional skeleton-guided node selection. |
| [path_planner](ros_packages/path_planner/README.md) | Persistent global octree + incremental sparse roadmap; serves collision-free simplified paths. |
| [skeleton_estimator](ros_packages/skeleton_estimator/README.md) | ROSA skeleton extraction, RANSAC alignment to a known source skeleton, object estimate and stability, precomputed path selection/transform. |
| [explorer_utils](ros_packages/explorer_utils/README.md) | Shared library (geometry, octomap helpers, UAV position lookup, `TimingPublisher`). No node. |
| [mrs_octomap_mapping_planning](ros_packages/mrs_octomap_mapping_planning/README.md) | Integration launch only (`mapplan.launch`): PCL filters, octomap server, octomap visualizers, nodelet manager. |

## Data flow between packages

All project nodelets are launched from `tmux/session.yml` with `standalone:=true`, i.e. each runs in
its own process. Every message below therefore goes through ROS serialization (TCPROS) — there is no
zero-copy intra-process passing. All names are relative to the `/<UAV_NAME>/` namespace.

```
PUSH (topics, asynchronous)
  octomap_server      ─ octomap_local_binary ─────►  frontier_detector, path_planner, explorer
  octomap_server      ─ octomap_global_binary ────►  octomap_global_vis, skeleton_estimator
  octomap_global_vis  ─ point_cloud_centers ──────►  skeleton_estimator
  tmux rostopic pub   ─ /source_skeleton,_viewpoints ► skeleton_estimator
  frontier_detector   ─ frontiers ────────────────►  explorer, clustering, (path_planner: unused)
  clustering          ─ clusters ─────────────────►  tsp_solver
  skeleton_estimator  ─ candidate_viewpoints ─────►  tsp_solver
  skeleton_estimator  ─ precomputed_path ─────────►  explorer

PULL (services, blocking; arrow = caller ──► server)
  explorer            ──► tsp_solver       set_start, solve, set_guiding
  explorer            ──► path_planner     find_simplified_path
  tsp_solver          ──► path_planner     find_simplified_path   (edge lengths)
  explorer            ──► explorer         replan_request         (self-call)
  explorer            ──► MRS              trajectory_generation/get_path, control_manager/trajectory_reference
```

### Topics

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `octomap_server/octomap_local_binary` | `octomap_msgs/Octomap` | octomap_server | `explorer` (collision checks, precomputed sample labelling), `frontier_detector`, `path_planner` — each fuses the local map into its own global octree |
| `octomap_server/octomap_global_binary` | `octomap_msgs/Octomap` | octomap_server | `skeleton_estimator` (only if `alignment/free_space/enabled`), `octomap_global_vis` |
| `octomap_global_vis/octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` | octomap rviz visualizer | `skeleton_estimator` (ROSA input) |
| `frontier_detector/frontiers` | `frontier_detector/FrontierArray` | `frontier_detector` | `explorer` (viewpoint cache for local refinement, frontier count, `tsp_ready_`), `clustering`, `path_planner` (callback only logs) |
| `clustering/clusters` | `frontier_detector/FrontierArray` | `clustering` | `tsp_solver` (keeps its distance graph in sync; uses `viewpoints[0]`) |
| `skeleton_estimator/candidate_viewpoints` | `sensor_msgs/PointCloud2` | `skeleton_estimator` | `tsp_solver` (KD-tree guiding set: tour restricted to frontiers within `guide_radius`) |
| `skeleton_estimator/precomputed_path` (latched) | `skeleton_estimator/PrecomputedPath` | `skeleton_estimator` | `explorer` (GOTO/FOLLOW once stable) |
| `skeleton_estimator/extracted_skeleton` | `visualization_msgs/Marker` | `skeleton_estimator` | `skeleton_estimator` itself (loop-back as RANSAC target), RViz |
| `/source_skeleton`, `/source_viewpoints` (latched) | `Marker` / `MarkerArray` | `rostopic pub` panes in tmux `skeleton` window, from `skeleton_estimator/data/<world_type>/*.outmsg` | `skeleton_estimator` |
| `control_manager/tracker_cmd`, `control_manager/diagnostics` | `mrs_msgs` | MRS control_manager | `explorer`, `frontier_detector`, `path_planner`, `clustering` (UAV position via `explorer_utils::getPosition`) |
| `tsp_solver/guiding_enabled` (latched) | `std_msgs/Bool` | `tsp_solver` | `scripts/repeat_runs.py` (logging) |
| `explorer/exploration_finished` (latched) | `std_msgs/Bool` | `explorer` | external scripts |
| `<node>/timing/<function>/<phase>` | `std_msgs/Float64` | every nodelet | PlotJuggler |

### Services (blocking request/response)

| Service | Type | Server | Clients |
|---|---|---|---|
| `path_planner/find_simplified_path` | `path_planner/FindSimplifiedPath` | `path_planner` | `explorer` (per tour chunk / GOTO leg), `tsp_solver` (batched edge lengths for new nodes and on every `setStart`) |
| `tsp_solver/set_start` | `tsp_solver/SetStart` | `tsp_solver` | `explorer` (`makePath()`, before each solve) |
| `tsp_solver/solve` | `tsp_solver/Solve` | `tsp_solver` | `explorer` (returns tour points + frontier ids) |
| `tsp_solver/set_guiding` | `std_srvs/SetBool` | `tsp_solver` | `explorer` (`disableTspGuiding()`, `false` once the precomputed path is done) |
| `explorer/replan_request` | `std_srvs/Trigger` | `explorer` | `explorer` itself (`requestReplan()` self-call, serializes replans onto the service thread) |
| `trajectory_generation/get_path` | `mrs_msgs/GetPathSrv` | MRS trajectory_generation | `explorer` |
| `control_manager/trajectory_reference` | `mrs_msgs/TrajectoryReferenceSrv` | MRS control_manager | `explorer` |

### One replan, end to end

1. `explorer`'s `timerReplanner()` decides to replan (idle, goal within latency, MPC prediction collides,
   or a FOLLOW re-extension) and calls its own `replan_request` service.
2. `makePath()` first tries the precomputed path: if `skeleton_estimator`'s path is stable, it labels path
   samples against its octree and either flies to an entry point (GOTO, via `path_planner/find_simplified_path`)
   or builds the trajectory directly from the path samples (FOLLOW, no `path_planner`).
3. Otherwise it falls back to frontiers: `tsp_solver/set_start` + `tsp_solver/solve` (LKH over the clustered,
   optionally skeleton-guided viewpoints; edge costs already queried from `path_planner`), refines the nearest
   clusters' viewpoints locally, then calls `path_planner/find_simplified_path` per tour chunk.
4. The assembled path goes to `trajectory_generation/get_path`, and the resulting trajectory to
   `control_manager/trajectory_reference`.

Note the asymmetry: maps and frontiers are **pushed** asynchronously, while every planning decision is
**pulled** synchronously by `explorer` through blocking services, so a slow `tsp_solver` (LKH runs up to
`lkh_time_limit * lkh_runs` s) or `path_planner` query directly delays the replan.

## Visualization

RViz layout `tmux/rviz.rviz`: frontiers, viewpoints and the global tour (explorer tab), the roadmap
(`path_planner/visualize_roadmap`), cluster tree (`clustering`), extracted/transformed skeleton and the
precomputed path (red = unstable, green = stable) from `skeleton_estimator`.
