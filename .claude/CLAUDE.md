# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Energy-aware UAV coverage path planning for scanning outdoor structures (e.g. wind turbines), built on the CTU-MRS UAV stack (ROS1/catkin, Gazebo simulation). This directory (`semestral-project`) is `src/semestral-project` inside a larger catkin workspace rooted at `/home/david/user_ros_workspace/` — it is not a standalone buildable checkout; the workspace's other `src/` packages (`mrs_uav_core`, `mrs_lib`, `mrs_msgs`, gazebo simulation, etc.) must already be present as sibling packages for a build to succeed.

Demo video: https://youtu.be/lA2REzf3Iso

## Build & run

```bash
./run.sh [-c] <world_name>       # -c cleans the catkin workspace first; world_name is a Gazebo world in ~/user_ros_workspace/worlds/
```

`run.sh` runs `catkin build` for the local packages (`tsp_solver` is pulled in transitively as a build dependency of `mrs_octomap_planner`, so it isn't listed explicitly), computes world bounds/spawn position via an external `worldbounds` binary, exports them as env vars (`SPAWN_POSITION`, `ZONE_X`, `ZONE_Y`, `ZONE_Z`), then hands off to `tmux/start.sh`, which launches a `tmuxinator` session (`tmux/session.yml`) with windows for roscore, Gazebo, drone spawn, the MRS UAV core/control stack, the mapping+planning pipeline (`mapplan.launch`), takeoff, and RViz.

To build just this project's packages manually (from the catkin workspace root):
```bash
catkin build octomap_planner_utils frontier_detection prm_solver tsp_solver skeleton_estimator mrs_octomap_planner mrs_octomap_mapping_planning
```
Do not build the project proactively — testing is done manually, by running the simulation.

There is no test suite (`CATKIN_ENABLE_TESTING` is explicitly disabled in every package's `CMakeLists.txt`) and no linter is configured.

`mrs_octomap_mapping_planning` also depends on external MRS repos (`mrs_octomap_server`, `mrs_octomap_tools`, `mrs_subt_planning_lib`) fetched via `gitman` (see its `.gitman.yml`) rather than vendored under this repo's `src/`.

### Runtime tuning

Most planner behavior is tunable without rebuilding via each package's `config/*.yaml`: [mrs_octomap_planner/config/explorer.yaml](ros_packages/mrs_octomap_planner/config/explorer.yaml) (planning distances, frontier/viewpoint sampling, timer rates), [frontier_detection/config/detector.yaml](ros_packages/frontier_detection/config/detector.yaml), [prm_solver/config/prm_solver.yaml](ros_packages/prm_solver/config/prm_solver.yaml) (zone sizes, PRM parameters, update timer rate), [tsp_solver/config/tsp_solver.yaml](ros_packages/tsp_solver/config/tsp_solver.yaml) (TSP duration), and [skeleton_estimator/config/skeleton_estimator.yaml](ros_packages/skeleton_estimator/config/skeleton_estimator.yaml) (ROSA algorithm parameters).

## Architecture

Seven ROS packages under `ros_packages/`, each with its own detailed `README.md` (purpose, full ROS interface tables, important functions/classes, file structure, dependencies) — **read the relevant package README before editing it**; this section only covers the cross-package big picture. `mrs_octomap_planner`, `frontier_detection`, `prm_solver`, `tsp_solver`, `skeleton_estimator`, and `mrs_octomap_mapping_planning` are `nodelet`-based catkin packages (built into a shared lib loaded by a nodelet manager, not standalone executables); `octomap_planner_utils` is a plain library package with no nodelet.

- **[octomap_planner_utils](ros_packages/octomap_planner_utils/README.md)** — shared geometry/octomap helper library (AABB math, octomap neighbor/free-space checks, random sampling, UAV position lookup). No ROS interface of its own; linked by `mrs_octomap_planner` and `frontier_detection`, both of which fully qualify every call with `octomap_planner_utils::`.

- **[mrs_octomap_planner](ros_packages/mrs_octomap_planner/README.md)** — the core of the project; everything else is scaffolding around it. It's the `Explorer` nodelet, which does **not** itself solve TSP ordering or PRM path search — it drives the standalone `tsp_solver` and `prm_solver` nodelets over blocking ROS services from `makePath()`, and does not detect frontiers itself (that's `frontier_detection`). Its declared `State_t` state machine is now largely vestigial — real control flow is driven by `map_ready_`/`tsp_ready_` flags and two timers, not by `state_`.

- **[frontier_detection](ros_packages/frontier_detection/README.md)** — standalone nodelet with its own message package (`msg/{Frontier,FrontierArray,Viewpoint}.msg`); extracts/splits frontier clusters (`FrontierManager`/`FIS`) within a local zone around the UAV, samples candidate viewpoints, and publishes `frontier_detection/FrontierArray`. `mrs_octomap_planner`, `prm_solver`, and `tsp_solver` each independently subscribe to this feed for their own purposes — there is no removal-by-id feedback channel; a frontier drops off the feed only when `FrontierManager::removeFrontiers()` itself invalidates it.

- **[prm_solver](ros_packages/prm_solver/README.md)** — standalone nodelet (`PRMNodelet`/`PRM`) maintaining a probabilistic roadmap over the live octomap, entirely from its own subscriptions/timer (map updates, its own zone timer, and one roadmap node per incoming frontier viewpoint) — `Explorer` never drives roadmap maintenance directly. Exposes one service, `~find_simplified_path_in`, which `Explorer::makePath()` calls per tour sub-segment.

- **[tsp_solver](ros_packages/tsp_solver/README.md)** — standalone nodelet (`Solver`/`TSPsolver`) ordering viewpoints into a global tour using a fixed **Euclidean** distance metric, independent of `prm_solver`'s roadmap. Solves via the bundled **LKH** binary (`ros_packages/tsp_solver/LKH/`, shelled out to via `std::system()`), with a greedy+3-opt fallback retained but not wired into the primary solve path. Exposes `~set_start_in`/`~solve_in`, both called back-to-back by `Explorer::makePath()`.

- **[skeleton_estimator](ros_packages/skeleton_estimator/README.md)** — standalone nodelet extracting a ROSA curve-skeleton from the live octomap point cloud; a from-scratch, trimmed-down port of `ROSA_main::pointCloudCallback` from the external FC-Planner repo (not part of this workspace and not a dependency). It is entirely self-contained: it publishes the skeleton on `/rosa_vis/rr_lines` and then, in its own `targetSkeletonCallback`, immediately consumes that same topic to RANSAC-align it against a manually-replayed "source" skeleton/viewpoint set, publishing the transformed result on `candidate_viewpoints_out`/`transformed_skeleton_out`. `mrs_octomap_planner`'s `Explorer` has no skeleton-related code at all (no subscription to any of these topics); `tsp_solver` subscribes to `candidate_viewpoints_out` but its callback currently returns immediately, so nothing actually consumes this output at runtime.

- **[mrs_octomap_mapping_planning](ros_packages/mrs_octomap_mapping_planning/README.md)** — no source of its own; just the top-level integration launch file [mapplan.launch](ros_packages/mrs_octomap_mapping_planning/launch/mapplan.launch) wiring together the PointCloud filters, Octomap Server, this project's `mrs_octomap_planner`, Octomap RViz visualizers, and a nodelet manager. The `frontier_detection`/`prm_solver`/`tsp_solver` includes are present but **commented out** here — all three (plus `skeleton_estimator`) are instead started from `tmux/session.yml`.

### Data/control flow at runtime

1. Octomap server publishes an incremental octomap → `Explorer::callbackOctomap` ingests it (for its own emergency-collision checks); `prm_solver` and `frontier_detection` independently subscribe to the same octomap topic for their own purposes.
2. `frontier_detection::Detector` extracts frontiers out-of-process and publishes `frontier_detection/frontiers`. `Explorer::callbackFrontiers` consumes it only to flag a TSP re-solve is due; `prm_solver` independently inserts a roadmap node per viewpoint; `tsp_solver` independently keeps its own distance graph in sync.
3. `prm_solver` updates its roadmap against the current map entirely on its own (on every new octomap, and on its own timer) — not driven by `Explorer`.
4. Periodically, `Explorer::makePath()` calls `tsp_solver`'s `~set_start_out`/`~solve_out` services for a fresh global viewpoint tour (LKH), then steps through that tour in chunks, calling `prm_solver`'s `~find_simplified_path_out` service per chunk to assemble a flyable, simplified `path_`.
5. `timerPath` drives the drone toward the next viewpoint along the assembled path, validates the trajectory is still viable as the map updates, and publishes trajectory references via `mrs_msgs`/`mrs_uav_core` services.
6. Visualization: frontiers, viewpoints, and the global TSP path are published via `mrs_lib::BatchVisualizer` and viewed in the RViz "explorer" tab; the PRM roadmap is visualized separately by `prm_solver` on its own `visualize_prm` `BatchVisualizer`.
7. Separately, `octomap_global_vis` publishes the octomap as a `PointCloud2`; `skeleton_estimator` consumes it, runs ROSA extraction, publishes on `/rosa_vis/rr_lines`, and then — entirely within its own nodelet — RANSAC-aligns that result against a manually-replayed source skeleton (still fed via `tmux/session.yml`) to derive `candidate_viewpoints_out`. This branch does not touch `mrs_octomap_planner` at all; it is independent of and does not feed back into steps 1–6.

### Simulation stack (tmux/)

[tmux/session.yml](tmux/session.yml) defines the full simulation session: `roscore` → Gazebo world spawn → drone spawn → `mrs_uav_px4_api` (hw_api) → `mrs_uav_core` → `mapplan.launch` → autostart/takeoff → RViz. Per-node config overrides live in [tmux/config/](tmux/config/) (octomap server/filters, network, world, hw_api). `tmux/rviz.rviz` is the RViz layout used for visualization. The `skeleton` window additionally starts `skeleton_estimator` and, via `rostopic pub`/`cat` panes, manually replays a prerecorded "source" skeleton and viewpoint set (`~/user_ros_workspace/skeletons/turbine-*.outmsg`) into `skeleton_estimator`'s own `source_skeleton_in`/`source_viewpoints_in` — there is still no live producer for those two topics.

## Notes for future changes

- `mfile.txt` and `temp.txt` at the repo root are scratch/leftover files, not part of the build — don't treat them as documentation.
- Prefer editing a package's `config/*.yaml` over hardcoding tunable parameters in C++ when adding new configurable behavior.
- After a structural change to a single package under `ros_packages/` (files/topics/services/functions/dependencies added, removed, or renamed), use the `package-readme` agent to update that package's `README.md` rather than editing it by hand — it's the source of truth this file's Architecture section summarizes.
- Several files across these packages are known dead/unreferenced code (e.g. `mrs_octomap_planner/src/pmm/`, `prm_solver`'s `simplifyRaycastPath`/`distance`/`extraDistance`, `tsp_solver`'s greedy/3-opt fallback path) — check the package's own README before assuming a function is live and wired up.
