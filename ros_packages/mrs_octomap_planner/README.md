# mrs_octomap_planner

## 1. Purpose

`mrs_octomap_planner` provides the `Explorer` nodelet, the core of this project's energy-aware UAV coverage planning pipeline. It is the component that actually flies the UAV: it tracks the live octomap and the UAV's current/predicted state, watches for imminent collisions along the predicted trajectory, and periodically assembles a flyable path out of the outputs of the other planning packages. It does not itself solve the travelling-salesman ordering of viewpoints or search the roadmap for collision-free sub-paths — those are delegated at runtime to the standalone `tsp_solver` and `prm_solver` nodelets over blocking ROS services — nor does it detect frontiers/viewpoints itself (that's `frontier_detection`). `Explorer`'s own job is to glue these together into a continuously replanned, dynamically-generated trajectory that is handed to the MRS trajectory generator/tracker, while guarding against collisions detected in newer maps.

## 2. ROS interface

All topic/service names below are the private (`~`) names used in `src/explorer.cpp`; the right-hand side is how [`launch/explorer.launch`](launch/explorer.launch) remaps them at runtime.

### Subscribed topics

| Private name | Remapped to | Type | Notes |
|---|---|---|---|
| `~tracker_cmd_in` | `control_manager/tracker_cmd` | `mrs_msgs/TrackerCommand` | source of the MPC full-state prediction used for collision checks |
| `~octomap_in` | `octomap_server/octomap_global_full` | `octomap_msgs/Octomap` | converted into the internal `octree_` (`callbackOctomap`) |
| `~diagnostics_in` | `control_manager/diagnostics` | `mrs_msgs/ControlManagerDiagnostics` | freshness-gates `getFullStatePrediction()`/`getPosition()` (both thin delegators into `octomap_planner_utils`); the callback itself (`controlManagerDiagCallback`) is currently a no-op |
| `~frontiers_in` | `frontier_detector/frontiers` | `frontier_detection/FrontierArray` | only used to flag that a TSP re-solve is due (`callbackFrontiers`); the message payload itself is not consumed here |

### Published topics

| Private name | Remapped to | Type | Notes |
|---|---|---|---|
| `~reference_out` | `control_manager/reference` | `mrs_msgs/ReferenceStamped` | advertised in `onInit()` and remapped in `explorer.launch`, but `pub_reference_.publish()` is never called anywhere in the file — dead output |
| `visualize_path` (via `mrs_lib::BatchVisualizer bv_path_`) | — | `visualization_msgs/Marker`/`MarkerArray` (BatchVisualizer) | debug rays for the assembled local path (magenta, in `pathAndTrajectory`) and the global TSP tour (green, in `visualizeGlobalPath`, called from `makePath`) |

### Services called (client)

| Private name | Remapped to | Type | Called from |
|---|---|---|---|
| `~trajectory_generation_out` | `trajectory_generation/get_path` | `mrs_msgs/GetPathSrv` | `makeTrajectory()` — turns the `Path_t` passed to it (`path_`, assembled by `makePath()`) into a `mrs_msgs/TrajectoryReference`, returned to the caller (no publishing) |
| `~trajectory_reference_out` | `control_manager/trajectory_reference` | `mrs_msgs/TrajectoryReferenceSrv` | `publishTrajectory()` — publishes the trajectory produced by `makeTrajectory()` (`fly_now=true`, `input_id = path_.id`) to the control manager |
| `~set_start_out` | `tsp_solver/set_start` | `tsp_solver/SetStart` | `solveTsp()` (called from `makePath()`) — sets the TSP start position before solving |
| `~solve_out` | `tsp_solver/solve` | `tsp_solver/Solve` | `solveTsp()` (called from `makePath()`) — requests the ordered global viewpoint tour from `tsp_solver` |
| `~find_simplified_path_out` | `prm_solver/find_simplified_path` | `prm_solver/FindSimplifiedPath` | `buildLocalPath()` (called from `makePath()`) — requests a flyable, simplified sub-path per tour segment from `prm_solver` |
| `~replan_request_out` | `~replan_request` | `std_srvs/Trigger` | `requestReplan()` — called from `timerReplanner()` whenever `state_ == STATE_IDLE`, or once `STATE_TRAJ_EXEC` and `nearGoal()`/`checkTrajectoryCollision()` fires; rate-limited by `planning/replan_request_cooldown` |

### Services advertised (server)

| Private name | Remapped to | Type | Handler |
|---|---|---|---|
| `~replan_request_in` | `~replan_request` | `std_srvs/Trigger` | `callbackReplanRequest()` — calls `pathAndTrajectory()` (which calls `makePath()`, then builds and publishes a trajectory) and returns its success/failure as `response.success` (`response.message` is always `"acknowledged"` regardless of outcome) |

`~replan_request_in` and `~replan_request_out` are both remapped in `explorer.launch` to the same private topic `~replan_request`, so this is a synchronous self-call within the `Explorer` node: `requestReplan()` (itself called from `timerReplanner()`) calls its own `~replan_request` service; `callbackReplanRequest()` handles it by running the actual replan (`pathAndTrajectory()` → `makePath()` → `makeTrajectory()` → `publishTrajectory()`) and reporting whether it succeeded. `requestReplan()` uses that result to move `state_` to `STATE_TRAJ_EXEC` on success or back to `STATE_IDLE` on failure. Unlike in earlier versions of this file, this round-trip is no longer inert scaffolding — it is the mechanism that actually drives every replan.

## 3. State machine

`Explorer` declares a `State_t` enum (`STATE_IDLE, STATE_PATH_PLANNING, STATE_TRAJ_EXEC, STATE_COLLISION`) and a `changeState()` helper that logs and sets the `state_` member. `state_` is driven entirely from `timerReplanner()`/`requestReplan()`:

- `STATE_IDLE` — the initial value, set once in `onInit()`. `timerReplanner()` re-enters it whenever `requestReplan()`'s self-call to `~replan_request` fails, and `checkStuck()` forces it back to `STATE_IDLE` if the UAV hasn't moved more than `planning/stuck_distance` in `planning/stuck_duration` seconds. While `state_ == STATE_IDLE`, `timerReplanner()` calls `requestReplan()` on every tick (subject to the cooldown), i.e. it keeps retrying until a replan succeeds.
- `STATE_PATH_PLANNING` — set by `requestReplan()` immediately before it makes the blocking `~replan_request` service call; used only as a transient marker while that call is in flight.
- `STATE_TRAJ_EXEC` — set by `requestReplan()` once the `~replan_request` call returns `success = true` (i.e. `pathAndTrajectory()`/`makePath()` produced and published a trajectory). While in this state, `timerReplanner()` runs its full per-tick logic: `nearGoal()`, `checkTrajectoryCollision()`, and `checkStuck()`.
- `STATE_COLLISION` — declared in the enum but never assigned anywhere in the current code; dead state. (Collision detection instead routes through `checkTrajectoryCollision()` triggering a normal `requestReplan()`, not a dedicated collision state.)

`timerReplanner()` short-circuits with `if (state_ != STATE_TRAJ_EXEC) return;` after the `STATE_IDLE` branch, so its `nearGoal()`/`checkTrajectoryCollision()`/`checkStuck()` checks only run once a trajectory is actually being executed. Note that the module-level `_state_names_` string array used for logging in `changeState()` still has 7 entries (`"IDLE", "STATE_PATH_PLANNING", "STATE_TRAJ_EXEC", "STATE_COLLISION", "TSP_UPDATED", "WAITING", "FLYING"`) left over from an older 7-value enum, even though `State_t` now only has 4 values (indices 0-3); the trailing three entries are unused but harmless since `state_` never takes a value outside 0-3. The comment directly above `changeState()`'s definition ("only ever called with STATE_FRONTIERS_UPDATED ... state_ is otherwise not read/branched on anywhere") is stale and no longer describes the code below it — `changeState()` is now called from both `requestReplan()` and `timerReplanner()`, and `state_` is actively branched on.

## 4. Important functions

(all in `mrs_octomap_planner::Explorer`, `src/explorer.cpp`, unless noted)

- **`onInit()`** — nodelet entry point; loads parameters from `explorer.yaml`, sets up subscribers/publishers/service clients, the `~replan_request_in` service server, starts `timer_main_`/`timer_replanner_`, and sets `state_` to `STATE_IDLE`. `timer_path_` is declared but its `createTimer()` call and the `timerPath()` method itself are both commented out — that timer no longer exists; its old responsibilities were folded into `pathAndTrajectory()`, which is now invoked from `callbackReplanRequest()` instead of its own timer. `timer_rates/path` is still loaded into `_rate_path_timer_` here, but with `timer_path_` never created that parameter has no effect at runtime.
- **`timerMain()`** — periodic main-loop timer (`timer_rates/main`); still a no-op placeholder once the map is ready.
- **`timerReplanner()`** — periodic timer (`timer_rates/fast` in `explorer.yaml`, default 20 Hz) and the actual driver of the whole replanning loop. Guards on `is_initialized_`/`map_ready_`/`tsp_ready_`. If `state_ == STATE_IDLE`, calls `requestReplan()` and returns (so it keeps retrying every tick, subject to the cooldown, until a replan succeeds). Otherwise it returns immediately unless `state_ == STATE_TRAJ_EXEC`; while executing a trajectory it fetches `getCurrentStateAndPrediction()` and then, in order: if `nearGoal()` is true, calls `requestReplan()` and returns; else if `checkTrajectoryCollision()` is true, calls `requestReplan()` and returns; else if `checkStuck()` is true, forces `changeState(STATE_IDLE)` (which causes `requestReplan()` to be retried on the next tick).
- **`requestReplan()`** — no-ops if called again within `planning/replan_request_cooldown` seconds of its last actual call (guards against being hammered by `timerReplanner()`'s fast tick while idle). Otherwise sets `state_ = STATE_PATH_PLANNING` and makes the blocking `~replan_request_out` service call (a self-call to this node's own `~replan_request_in`, handled by `callbackReplanRequest()`); on success sets `stuck_check_reset_ = true` and `state_ = STATE_TRAJ_EXEC`, on failure sets `state_ = STATE_IDLE`.
- **`pathAndTrajectory()`** — runs one full replan-and-publish cycle: calls `makePath()` (returns `false` if it didn't produce a new path); draws the resulting `path_` as magenta debug rays on `bv_path_`, publishes and clears that buffer; calls `makeTrajectory(path_)` (returns `false` on `std::nullopt`); calls `publishTrajectory()` with the result (returns `false` on failure); logs the total replan time and returns `true`. Called only from `callbackReplanRequest()`. This is the successor to the old, now-removed `timerPath()` — the logic is the same, just invoked synchronously from the `~replan_request_in` handler instead of its own timer.
- **`callbackOctomap()`** — converts the incoming octomap message into `octree_`, latches the map frame onto `bv_path_` once, and sets `map_ready_`; does *not* extract frontiers (that happens out-of-process in `frontier_detection`).
- **`callbackFrontiers()`** — sets `tsp_ready_` to flag that a TSP re-solve is due; the message payload itself is not otherwise consumed. PRM roadmap updates from frontiers happen internally in `prm_solver`, not here.
- **`callbackReplanRequest()`** — handler for `~replan_request_in`; calls `pathAndTrajectory()` and returns its result in `response.success` (`response.message` is unconditionally `"acknowledged"`). This is the sole entry point that actually triggers a replan/trajectory publish; it is only ever invoked via the self-call described above (from `requestReplan()`).
- **`makePath()`** — the core replanning routine, a thin orchestrator over several helpers, called unconditionally (no internal collision/distance gating of its own): `getCurrentStateAndPrediction()` (abort if unavailable) → sets `goal_`/`current_viewpoint_` to `start_coord` → `solveTsp()` (abort if it fails) → `visualizeGlobalPath()` → `buildLocalPath()` (abort if it fails) → `finalizePath()`, whose return value is `makePath()`'s own return value. Unlike in earlier versions of this file, `makePath()` no longer calls `checkImminentCollision()` or `nearGoal()` itself — both of those gating checks now live in `timerReplanner()`, which only invokes the whole chain (via `requestReplan()`) once they've fired.
- **`getCurrentStateAndPrediction(start_coord, prediction, velocity)`** — fetches the current UAV position via `getPosition()` and the MPC full-state prediction via `getFullStatePrediction()`, deriving `start_coord`/`prediction`/`velocity`; returns `false` with a throttled warning if either is unavailable. Called from both `makePath()` and `timerReplanner()`.
- **`checkImminentCollision(start_coord, prediction)`** — walks the MPC prediction's position samples against a snapshot of `octree_`; if any sample is not free space (per `_flight_free_distance_`), sets `path_.points = {start_coord, last_free_point_}`, bumps `path_.id`, sets `goal_` to the brake point, sets `braking_ = true`, and returns `true` (an emergency brake path). Otherwise updates `last_free_point_` and returns `false`. **Dead code**: still defined but no longer called from anywhere in the file (it used to be called from `makePath()`; that call site was removed). `_free_space_dia_` (`prm/free_space_diameter`) is only read inside this function, so that parameter is now loaded but has no effect at runtime either.
- **`nearGoal(start_coord, prediction)`** — on the very first call (`!first_path_planend_`) initializes `goal_`/`next_goal_` to `start_coord`; otherwise returns `true` only once the UAV is within `_replanning_distance_` of both `goal_` and `next_goal_`, i.e. close enough to its current waypoints to warrant a fresh tour. Called only from `timerReplanner()` now (no longer from `makePath()`).
- **`checkStuck(start_coord)`** — new helper backing the stuck-detection added to `timerReplanner()`. Tracks `last_moved_position_`/`last_moved_time_`; whenever the UAV has moved more than `planning/stuck_distance` since the tracked position (or `stuck_check_reset_` is set, e.g. right after a successful `requestReplan()`), it re-arms the tracker and returns `false`; otherwise it returns `true` once `planning/stuck_duration` seconds have elapsed without that much movement, signalling `timerReplanner()` to force `state_` back to `STATE_IDLE`.
- **`solveTsp(velocity, glob_path)`** — calls `tsp_solver`'s `~set_start_out` (with `current_viewpoint_`) then `~solve_out` (with `velocity`) services; fills `glob_path` from the response and returns `false` if either call/response fails or the returned tour has `size() <= 1`.
- **`visualizeGlobalPath(glob_path)`** — draws the freshly solved global TSP tour as green debug rays on `bv_path_` (buffer is not published/cleared here — that happens together with the local path rays in `pathAndTrajectory()`).
- **`buildLocalPath(start_coord, glob_path, velocity, path, sub_global_path)`** — walks forward through `glob_path`, skipping points closer than `_skip_path_point_distance_` to the last accepted `sub_global_path` point, and for each accepted sub-segment calls `prm_solver`'s `~find_simplified_path_out` service to get a simplified sub-path, appending points more than `_skip_path_point_distance_` from the current `path` tail; stops once `sub_global_path` has grown past 3 points and an (unnormalized) running `path_distance` exceeds 4.0. Returns `false` if any `~find_simplified_path_out` call/response fails. Note the `path_distance` accumulation (`path_distance += path_distance + ...`) doubles the running total on each iteration rather than summing segment lengths, so the loop's actual distance-based exit condition is not a literal 4 m of path length; it is still a real code path, not dead, just numerically odd.
- **`finalizePath(path, sub_global_path, t0)`** — drops the leading point of `path` (the UAV's current, already-occupied position), commits the rest into `path_.points`, bumps `path_.id`; if the result is non-empty, updates `goal_`/`next_goal_`, sets `first_path_planend_ = true`, logs the replan time, and returns `true`; otherwise warns "no suitable frontiers" and returns `false`.
- **`checkTrajectoryCollision(start_coord, prediction)`** — checks the given MPC prediction's position samples against a snapshot of `octree_` (per `_flight_free_distance_`) and returns `true` if any sample is occupied (stops at the first occupied sample found). Called live from `timerReplanner()` on every tick while `state_ == STATE_TRAJ_EXEC`; its parameters are now passed in by the caller rather than fetched internally. The source comment directly above its definition ("dead code, never called") is stale and describes an older version of the function — it is a live, load-bearing check today. This is a separate check from `checkImminentCollision()` (see above, now itself dead), which additionally mutated `path_`/`goal_`/`braking_`/`last_free_point_`; `checkTrajectoryCollision()` only reports true/false and mutates no planning state itself.
- **`makeTrajectory(const Path_t& path)`** — converts the given `path` (points, `path.id` unused here) into an `mrs_msgs::TrajectoryReference` via the `~trajectory_generation_out` service and returns it (or `std::nullopt` on a failed/unsuccessful service call). `pathAndTrajectory()` calls it with `path_` after `makePath()` returns.
- **`publishTrajectory(const mrs_msgs::TrajectoryReference& trajectory, int id)`** — publishes a trajectory previously produced by `makeTrajectory()` via the `~trajectory_reference_out` service, setting `fly_now=true` and `input_id=id`; returns `false` on a failed/unsuccessful service call. `pathAndTrajectory()` calls it with the result of `makeTrajectory()` and `path_.id`.
- **`getFullStatePrediction()`** — returns the tracker's full-state MPC prediction (or `nullopt` if diagnostics/tracker_cmd are stale (>2s) or no frame transform is available), delegating to `octomap_planner_utils::getFullStatePrediction`.
- **`getPosition()`** — returns the current UAV position in the octree frame, delegating to `octomap_planner_utils::getPosition`.
- **`msgToMap()`** — deserializes an `octomap_msgs/Octomap` (binary or full) into an `OcTree_t`.
- **`changeState()`** — logs and sets `state_`; see State machine section above. Called from `requestReplan()` (`STATE_PATH_PLANNING`, `STATE_TRAJ_EXEC`, `STATE_IDLE`) and `timerReplanner()` (`STATE_IDLE`, via `checkStuck()`).
- **`controlManagerDiagCallback()`** — diagnostics callback; currently a no-op besides the init guard.
- **`timeoutTrackerCmd()` / `timeoutOctomap()`** — `SubscribeHandler` timeout callbacks; log a throttled warning when the respective topic goes stale.

## 5. File structure

```text
mrs_octomap_planner/
├── CMakeLists.txt                        # only src/explorer.cpp is compiled, into MrsOctomapPlanner_Explorer
├── package.xml
├── nodelets.xml                          # exports mrs_octomap_planner/Explorer; an older OctomapPlanner class entry is commented out
├── LICENSE
├── README.md
├── launch/
│   └── explorer.launch                   # launches the Explorer nodelet, loads explorer.yaml, sets up all remaps (incl. ~replan_request_in/~replan_request_out -> ~replan_request)
├── config/
│   ├── explorer.yaml                     # actually loaded by explorer.launch — planning/prm/viz/timer params (incl. timer_rates/fast for timer_replanner_); no longer has a frontiers: block (frontier extraction lives entirely in frontier_detection/config/detector.yaml)
│   ├── octomap_planner.yaml              # leftover: not referenced by explorer.launch or any code in this package (params for the old, no-longer-built OctomapPlanner/A*/SubT-planner nodelet)
│   └── debug_verbosity.yaml              # log4j config, only loaded if LOGGER_DEBUG env is set
├── src/
│   ├── explorer.cpp                      # COMPILED — the only file in the active build; the Explorer nodelet
│   └── pmm/                              # NOT COMPILED — unused vendored library, not referenced by CMakeLists.txt or explorer.cpp
│       ├── common.cpp
│       ├── pmm_mg_trajectory3d.cpp
│       ├── pmm_trajectory.cpp
│       └── pmm_trajectory3d.cpp
└── include/
    └── pmm/                              # NOT COMPILED — headers for the same unused vendored library
        ├── common.hpp
        ├── pmm_mg_trajectory3d.hpp
        ├── pmm_trajectory.hpp
        └── pmm_trajectory3d.hpp
```

The `pmm/` sources/headers are a vendored "piecewise minimum-time trajectory" library (per their file-header authorship comments: Robert Penicka, Krystof Teissing, Matej Novosad, CTU FEL) — they are not included in `CMakeLists.txt`'s `add_library(MrsOctomapPlanner_Explorer ...)` and are not `#include`d from `explorer.cpp`, so they are leftover/dead code, analogous to other unreferenced files noted for this package historically (an earlier version of this package also carried unreferenced `astar_planner.hpp`/`planner_template.hpp`/`skleton.cpp` files; those have since been removed from the tree entirely, but `pmm/` remains).

## 6. Dependencies

From `package.xml`/`CMakeLists.txt` (build-time, catkin). The package defines no `msg`/`srv`/`action` files of its own (the former `srv/Path.srv` and its `message_generation`/`message_runtime` dependencies were removed along with the dead `~get_path_in` service):

- `mrs_lib`, `mrs_msgs`, `mrs_modules_msgs` — MRS UAV stack core: `BatchVisualizer`, `ParamLoader`, `SubscribeHandler`, `Transformer`, `ServiceClientHandler`, and message types (`TrackerCommand`, `ControlManagerDiagnostics`, `TrajectoryReference*`, `GetPathSrv`, etc.)
- `std_srvs` — `std_srvs::Trigger`, used for both the `~replan_request_in` service server (`ss_replan_request_`/`callbackReplanRequest`) and the `~replan_request_out` service client (`sc_replan_request_`, called from `requestReplan()`, itself called from `timerReplanner()`)
- `mrs_subt_planning_lib`, `mrs_octomap_tools` — external MRS repos; `mrs_octomap_tools/octomap_methods.h` is included by `explorer.cpp`
- `octomap_msgs`, `octomap_ros`, plus `find_package(octomap REQUIRED)` and `find_package(dynamicEDT3D REQUIRED)` — octomap representation/conversion
- `nodelet`, `roscpp`, `rospy`, `pluginlib` (via `PLUGINLIB_EXPORT_CLASS`) — nodelet plumbing
- `nav_msgs`, `std_msgs`, `geometry_msgs`, `visualization_msgs`, `sensor_msgs` — standard ROS message types
- `Eigen3` (`find_package(Eigen3 REQUIRED)`) — used via `mrs_lib::geometry::Ray`/`BatchVisualizer` calls in `explorer.cpp`
- `octomap_planner_utils` — this project's own shared geometry/octomap helper library; `explorer.cpp` fully qualifies every call with the `octomap_planner_utils::` prefix (`isFreeSpace`, `getPosition`, `getFullStatePrediction`, etc.) — `Explorer::getPosition()`/`Explorer::getFullStatePrediction()` are now both thin delegators to their `octomap_planner_utils` counterparts
- `frontier_detection` — declared as a build dependency for its `FrontierArray` message type, consumed on `~frontiers_in`

Runtime-only (not build dependencies, but required at runtime for `Explorer` to function — separate standalone nodelets started elsewhere, e.g. from `tmux/session.yml`):

- `tsp_solver` — provides the `~set_start_out`/`~solve_out` services `solveTsp()` calls (also a build dependency, for the `SetStart`/`Solve` service message types)
- `prm_solver` — provides the `~find_simplified_path_out` service `buildLocalPath()` calls (also a build dependency, for the `FindSimplifiedPath` service message type)
- `frontier_detection` — the actual `Detector` nodelet must be running to publish `frontier_detection/frontiers`, which `Explorer` only consumes as a trigger signal (the same topic is independently consumed by `tsp_solver` and `prm_solver` for their own internal state)

`~replan_request_in`/`~replan_request_out` are both remapped to the same `~replan_request` topic in `explorer.launch`, so the `std_srvs::Trigger` request/response round-trips entirely within this node today — it is not currently a dependency edge to or from any other package.
