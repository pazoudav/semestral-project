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
| `~diagnostics_in` | `control_manager/diagnostics` | `mrs_msgs/ControlManagerDiagnostics` | freshness-gates `getFullStatePrediction()`/`getPosition()`; the callback itself (`controlManagerDiagCallback`) is currently a no-op |
| `~frontiers_in` | `frontier_detection/frontiers` | `frontier_detection/FrontierArray` | only used to flag that a TSP re-solve is due (`callbackFrontiers`); the message payload itself is not consumed here |

### Published topics

| Private name | Remapped to | Type | Notes |
|---|---|---|---|
| `~reference_out` | *(not remapped in `explorer.launch`)* | `mrs_msgs/ReferenceStamped` | advertised in `onInit()` but never actually published to — dead output |
| `~big_octomap_out` | `big_octomap` | `visualization_msgs/MarkerArray` | advertised in `onInit()` but never actually published to — dead output |
| `visualize_path` (via `mrs_lib::BatchVisualizer bv_path_`) | — | `visualization_msgs/Marker`/`MarkerArray` (BatchVisualizer) | debug rays for the assembled local path (magenta, in `timerPath`) and the global TSP tour (green, in `makePath`) |

### Services called (client)

| Private name | Remapped to | Type | Called from |
|---|---|---|---|
| `~trajectory_generation_out` | `trajectory_generation/get_path` | `mrs_msgs/GetPathSrv` | `makeTrajectory()` — turns `path_` into a trajectory |
| `~trajectory_reference_out` | `control_manager/trajectory_reference` | `mrs_msgs/TrajectoryReferenceSrv` | `makeTrajectory()` — publishes the trajectory (`fly_now=true`) to the control manager |
| `~set_start_out` | `tsp_solver/set_start` | `tsp_solver/SetStart` | `makePath()` — sets the TSP start position before solving |
| `~solve_out` | `tsp_solver/solve` | `tsp_solver/Solve` | `makePath()` — requests the ordered global viewpoint tour from `tsp_solver` |
| `~find_simplified_path_out` | `prm_solver/find_simplified_path` | `prm_solver/FindSimplifiedPath` | `makePath()` — requests a flyable, simplified sub-path per tour segment from `prm_solver` |

### Services advertised (server)

| Private name | Remapped to | Type | Notes |
|---|---|---|---|
| `~get_path_in` | `~get_path` | `mrs_octomap_planner/Path` (see [`srv/Path.srv`](srv/Path.srv)) | `callbackGetPath` — currently unimplemented stub, always returns `false` |
| `~explore_in` | `~explore` | `std_srvs/Trigger` | `callbackExplore` — currently unimplemented stub, always returns `true` with no side effects |

## 3. State machine

`Explorer` declares a `State_t` enum (`STATE_IDLE → STATE_MAP_UPDATED → STATE_FRONTIERS_UPDATED → STATE_PRM_UPDATED → STATE_TSP_UPDATED → STATE_WAITING → STATE_FLYING`) and a `changeState()` helper that logs and sets the `state_` member. In the current code, however, only two of these states are ever actually reached:

- `STATE_IDLE` — the initial value, set once in `onInit()`.
- `STATE_FRONTIERS_UPDATED` — set every time `callbackFrontiers()` receives a new `frontier_detection/FrontierArray` message.

`state_` is otherwise never read or branched on anywhere in the file — none of `timerMain()`, `timerPath()`, or `makePath()` check it. The actual control flow is instead driven directly by the `map_ready_`/`tsp_ready_` atomic flags (set in `callbackOctomap()`/`callbackFrontiers()` respectively) and by `timerPath()` unconditionally calling `makePath()`/`makeTrajectory()` on every tick once those flags are set. In practice the state machine is now a vestigial diagnostic label rather than something that gates behavior — consistent with PRM/TSP solving having moved out-of-process into the `prm_solver`/`tsp_solver` nodelets, so the `STATE_PRM_UPDATED`/`STATE_TSP_UPDATED`/`STATE_WAITING`/`STATE_FLYING`/`STATE_MAP_UPDATED` states are dead states that are never entered.

## 4. Important functions

(all in `mrs_octomap_planner::Explorer`, `src/explorer.cpp`, unless noted)

- **`onInit()`** — nodelet entry point; loads parameters from `explorer.yaml`, sets up subscribers/publishers/service clients, starts `timer_main_`/`timer_path_`, and sets `state_` to `STATE_IDLE`.
- **`timerMain()`** — periodic main-loop timer; currently a no-op placeholder once the map is ready (frontier-triggered work happens in `callbackFrontiers()` instead).
- **`timerPath()`** — periodic path-update timer: calls `makePath()` to (re)plan, publishes the resulting path as debug rays on `bv_path_`, then calls `makeTrajectory()` to request/publish a new trajectory.
- **`callbackOctomap()`** — converts the incoming octomap message into `octree_`, latches the map frame onto `bv_path_` once, and sets `map_ready_`; does *not* extract frontiers (that happens out-of-process in `frontier_detection`).
- **`callbackFrontiers()`** — sets `tsp_ready_` and calls `changeState(STATE_FRONTIERS_UPDATED)` to flag that a TSP re-solve is due; PRM roadmap updates from frontiers happen internally in `prm_solver`, not here.
- **`makePath()`** — the core replanning routine: checks the current MPC prediction for imminent collisions (triggers an emergency brake to the last known free point if so); otherwise, once close enough to the current goal, calls `tsp_solver`'s `~set_start_out`/`~solve_out` services for a fresh global viewpoint tour, then steps through tour sub-segments calling `prm_solver`'s `~find_simplified_path_out` service to assemble a flyable `path_`.
- **`checkTrajectoryCollision()`** — checks the MPC prediction against `octree_` for collisions; **dead code**, never called (equivalent logic is inlined directly inside `makePath()`).
- **`makeTrajectory()`** — converts `path_` into a trajectory via `~trajectory_generation_out`, then publishes it (`fly_now=true`) via `~trajectory_reference_out`, incrementing `path_id_` as the trajectory's `input_id`.
- **`getFullStatePrediction()`** — returns the tracker's full-state MPC prediction transformed into the octree frame, or `nullopt` if diagnostics/tracker_cmd are stale (>2s) or the frame transform fails.
- **`getPosition()`** — returns the current UAV position in the octree frame, delegating to `octomap_planner_utils::getPosition`.
- **`msgToMap()`** — deserializes an `octomap_msgs/Octomap` (binary or full) into an `OcTree_t`.
- **`changeState()`** — logs and sets `state_`; see State machine section above — effectively just a diagnostic label in the current code.
- **`callbackGetPath()` / `callbackExplore()`** — service handler stubs (`~get_path_in`, `~explore_in`); both currently unimplemented.
- **`controlManagerDiagCallback()`** — diagnostics callback; currently a no-op besides the init guard.
- **`timeoutTrackerCmd()` / `timeoutOctomap()`** — `SubscribeHandler` timeout callbacks; log a throttled warning when the respective topic goes stale.

## 5. File structure

```
mrs_octomap_planner/
├── CMakeLists.txt                        # only src/explorer.cpp is compiled, into MrsOctomapPlanner_Explorer
├── package.xml
├── nodelets.xml                          # exports mrs_octomap_planner/Explorer; an older OctomapPlanner class entry is commented out
├── LICENSE
├── README.md
├── srv/
│   └── Path.srv                          # request: start/end Point; response: path Point[], success, message
├── launch/
│   └── explorer.launch                   # launches the Explorer nodelet, loads explorer.yaml, sets up all remaps
├── config/
│   ├── explorer.yaml                     # actually loaded by explorer.launch — planning/prm/frontiers/viz/timer params
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

From `package.xml`/`CMakeLists.txt` (build-time, catkin):

- `mrs_lib`, `mrs_msgs`, `mrs_modules_msgs` — MRS UAV stack core: `BatchVisualizer`, `ParamLoader`, `SubscribeHandler`, `Transformer`, `ServiceClientHandler`, and message types (`TrackerCommand`, `ControlManagerDiagnostics`, `TrajectoryReference*`, `GetPathSrv`, etc.)
- `mrs_subt_planning_lib`, `mrs_octomap_tools` — external MRS repos; `mrs_octomap_tools/octomap_methods.h` is included by `explorer.cpp`
- `octomap_msgs`, `octomap_ros`, plus `find_package(octomap REQUIRED)` and `find_package(dynamicEDT3D REQUIRED)` — octomap representation/conversion
- `nodelet`, `roscpp`, `rospy`, `pluginlib` (via `PLUGINLIB_EXPORT_CLASS`) — nodelet plumbing
- `nav_msgs`, `std_msgs`, `geometry_msgs`, `visualization_msgs`, `sensor_msgs` — standard ROS message types
- `message_generation`/`message_runtime` — generates `Path.srv`
- `Eigen3` (`find_package(Eigen3 REQUIRED)`) — used via `mrs_lib::geometry::Ray`/`BatchVisualizer` calls in `explorer.cpp`
- `octomap_planner_utils` — this project's own shared geometry/octomap helper library; `explorer.cpp` fully qualifies every call with the `octomap_planner_utils::` prefix (`isFreeSpace`, `getPosition`, etc.)
- `frontier_detection` — declared as a build dependency for its `FrontierArray` message type, consumed on `~frontiers_in`

Runtime-only (not build dependencies, but required at runtime for `Explorer` to function — separate standalone nodelets started elsewhere, e.g. from `tmux/session.yml`):

- `tsp_solver` — provides the `~set_start_out`/`~solve_out` services `makePath()` calls (also a build dependency, for the `SetStart`/`Solve` service message types)
- `prm_solver` — provides the `~find_simplified_path_out` service `makePath()` calls (also a build dependency, for the `FindSimplifiedPath` service message type)
- `frontier_detection` — the actual `Detector` nodelet must be running to publish `frontier_detection/frontiers`, which `Explorer` only consumes as a trigger signal (the same topic is independently consumed by `tsp_solver` and `prm_solver` for their own internal state)
