# tsp_solver

## Purpose

`tsp_solver` orders exploration viewpoints (frontiers, plus a start node for the UAV's
current position) into a single global visiting tour. It runs as a standalone nodelet
decoupled from `explorer`'s `Explorer` and from `path_planner`: it keeps its own
distance graph in sync with the live frontier set by querying `path_planner`'s
`~find_simplified_path_in` service for real (roadmap, raycast-simplified) travel distances
between nodes, and solves the resulting (asymmetric) TSP by shelling out to the bundled LKH
solver binary.
`Explorer::makePath()` drives it purely through two blocking services to obtain an ordered
tour, which it then hands to `path_planner` in chunks to get a flyable path.

## ROS interface

### Topics subscribed

| Topic (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~frontiers_in` | `clustering/clusters` | `frontier_detector/FrontierArray` | Keeps the internal distance graph (`TSPsolver::nodes_`/`distances_`/`id_to_index_`) in sync via `syncFrontiers()`. Fed by the `clustering` nodelet (one frontier per cluster), not the raw `frontier_detector/frontiers` feed; the raw remap is left commented out in `tsp_solver.launch`. |
| `~candidate_viewpoints_in` | `skeleton_estimator/candidate_viewpoints` | `sensor_msgs/PointCloud2` | `Solver::callbackCandidateViewpoints` converts the cloud to `pcl::PointCloud<pcl::PointXYZ>` and passes it to `TSPsolver::setKDtreeInput()` as the skeleton-derived guiding viewpoints (see `selectTourNodes`). |

### Services advertised (server)

| Service (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~set_start_in` | `~set_start` | `tsp_solver/SetStart` (`geometry_msgs/Point position` → `bool success`) | Sets/replaces the graph's start node (current UAV position). |
| `~solve_in` | `~solve` | `tsp_solver/Solve` (`geometry_msgs/Vector3 velocity` → `geometry_msgs/Point[] path`, `uint32[] ids`, `bool success`, `string message`) | Solves the TSP for the given start velocity/heading and returns the ordered viewpoint tour; `ids` holds the frontier id of each path point (parallel to `path`, start node id 0). |
| `~set_guiding_in` | `~set_guiding` | `std_srvs/SetBool` | `Solver::callbackSetGuiding` — enables (`data=true`, default) / disables guided tour node selection via `TSPsolver::setGuidingEnabled()`; when disabled, `selectTourNodes()` returns all nodes. Takes effect on the next solve. The new state is also published on `~guiding_enabled_out`. |

`~set_start_in` and `~solve_in` are called back-to-back by `Explorer::makePath()` (in `explorer`, not
part of this package) to obtain a fresh tour.

### Services called (client)

| Service (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~find_simplified_path_out` | `path_planner/find_simplified_path` | `path_planner/FindSimplifiedPath` (`geometry_msgs/Point start`, `geometry_msgs/Point[] goals`, `geometry_msgs/Vector3 velocity`, `bool use_raycast` → `path_planner/Path[] paths`, `bool[] success`) | `findPathDistances()` (called from `syncFrontiers()`/`setStart()`) — batched query to `path_planner` for real, raycast-simplified roadmap path lengths from one start node to a set of goal nodes. |

### Topics published

| Topic (private name) | Type | Purpose |
|---|---|---|
| `~guiding_enabled_out` (remapped to `~guiding_enabled`; latched) | `std_msgs/Bool` | Current guiding state: `true` at init, then `req.data` republished on every `~set_guiding_in` call. Consumed by `scripts/repeat_runs.py`, which logs guiding on/off changes to `runs.log`. |
| `~timing/<scope>/<stage>` (e.g. `~timing/callbackSolve/total`) | `std_msgs/Float64` | Wall-clock timings in ms via `explorer_utils::TimingPublisher`. `Solver`: `callbackFrontiers/total`, `callbackSetStart/total`, `callbackSolve/{solve,build_resp,total}`. `TSPsolver`: `findPathDistances/{build_req,svc_call,parse_resp,total}`, `syncFrontiers/{incoming_ids,prune_removed,add_new,total}`, `setStart/{gather_ids,find_dists,update_map,total}`. |

## Important functions/classes

- **`tsp_solver::Solver`** (`solver_nodelet.cpp`) — the nodelet: owns a `TSPsolver` instance and a `std::unique_ptr<explorer_utils::TimingPublisher> timing_` (built from the private nodehandle in `onInit`), wires up the topic subscriptions and the two services, and translates between ROS messages and `TSPsolver`'s C++ API. It does no locking of its own — `TSPsolver` is internally thread-safe (see the concurrency note below).
- **`tsp_solver::TSPsolver`** (`tsp_solver.hpp`/`tsp_solver.cpp`) — the actual solver logic, independent of ROS nodelet machinery, apart from its own `explorer_utils::TimingPublisher timing_` (constructed from the nodehandle passed to its constructor) that publishes per-stage timings of `findPathDistances`/`syncFrontiers`/`setStart`. The distance graph is stored flat rather than as nested maps: `nodes_` (a `std::vector<planner_t>`, `planner_t` holding just id/position/reachability) and a same-indexed `distances_` (`Eigen::MatrixXf`, over-allocated and grown via `conservativeResize` as needed) are kept in sync through `id_to_index_` (`std::unordered_map<unsigned long, int>`). The start node (id `START_ID` = 0) is always pinned to index 0 (`reserveStartSlot()`, run once on the first `setStart()` call) — `constructDistanceMatrix()`'s zero-cost "return to start" column depends on that. Removing any other node (`removeNode`) swaps the last node into its slot instead of shifting everything, keeping indices dense in O(1).
- **`syncFrontiers`** — reconciles the distance graph with an incoming `FrontierArray`: removes nodes for frontiers no longer present (`removeNode`, never called for `START_ID`), then for each new frontier's first viewpoint appends a node (`addNode`) with distances to all existing nodes obtained from a single batched `findPathDistances` call.
- **`setStart`** — (re)sets the start/current-position node's position and recomputes its distances to every other node via one batched `findPathDistances` call; marks unreachable neighbors (`BIG_DISTANCE`) as inaccessible. On the very first call it also reserves index 0 for the start node (`reserveStartSlot()`).
- **`findPathDistances`** — issues one `~find_simplified_path_out` call from a given start position to a batch of goal positions (a single shared Dijkstra search over all goals, per `path_planner`'s own batching design); returns each goal's path length as the sum of the returned simplified path's segment lengths, or `BIG_DISTANCE` for a goal `path_planner` couldn't reach or if the service call itself failed.
- **`setKDtreeInput`** — builds a fresh `pcl::KdTreeFLANN` over a cloud of guiding viewpoints (ignored if empty) and swaps it in as `tree_`/`pcset_` under `mutex_guide_`, so the index build never blocks a running solve.
- **`selectTourNodes`** — picks the graph indices the next tour is solved over: `START_ID` (always index 0) plus every node within `guide_radius` of a guiding viewpoint. Falls back to all nodes when guiding is disabled (`setGuidingEnabled(false)`, flag `guiding_enabled_` guarded by `mutex_guide_`), no guiding cloud has been received yet, or no node is guided, so unguided frontiers are only toured once no guided ones remain.
- **`constructDistanceMatrix`** — flattens the current `distances_`/`nodes_` into the dense `cost_matrix_`/`viewpoint_positions_`/`isAccesible_` fed to `LKHSolve()` restricted to the `selectTourNodes()` subset (row/column 0 stays `START_ID`), then adds a start-heading penalty (`dir_weight`) to the start row. Must be called with `mutex_dist_graph_` held.
- **`solve(octomath::Vector3 velocity, std::vector<unsigned long>& ids)`** — main entry point used by the nodelet: snapshots the distance graph (`constructDistanceMatrix()`, briefly under `mutex_dist_graph_`) and calls `LKHSolve()`, then maps the returned tour indices back to viewpoint positions and fills `ids` with the matching node ids (from the snapshotted `node_ids_`), discarding (returning an empty tour) if any index is out of range for the current viewpoint count.
- **LKH invocation (`GlobalParWrite` / `GlobalProblemWrite` / `LKHSolve` / `GlobalResultsRead`)** — `GlobalParWrite()` writes the LKH `.par` control file once, from the `TSPsolver` constructor (its contents are fixed for the process lifetime). Each `LKHSolve()` call writes the current cost matrix out as an explicit-weight ATSP problem file (`cost_matrix.txt`), removes any previous `solution.txt`, shells out via `std::system()` to the bundled `LKH/LKH` binary, and — only if that call exits successfully — parses the fresh `solution.txt`'s `TOUR_SECTION` back into a 0-based node index tour; a nonzero exit code is logged and returns an empty tour rather than risking a stale/mismatched `solution.txt` being parsed. Paths are resolved relative to `ros::package::getPath("tsp_solver")`.

### Concurrency

`TSPsolver` guards its own state with three mutexes instead of relying on a single external lock: `mutex_dist_graph_` protects `nodes_`/`distances_`/`id_to_index_` and is held by `syncFrontiers()`, `setStart()`, and only the brief `constructDistanceMatrix()` snapshot inside `solve()`; `mutex_guide_` protects the guiding KD-tree (`tree_`/`pcset_`), held by `setKDtreeInput()` (only for the swap) and `selectTourNodes()`; `mutex_solve_` is held for the whole of `solve()` (guarding `cost_matrix_`/`viewpoint_positions_`/`isAccesible_` and the LKH input/output files against concurrent solves). Because the two are separate, the blocking `std::system()` LKH call — up to `lkh_time_limit * lkh_runs` seconds — no longer stalls incoming frontier/start-position updates the way a single shared lock held across the whole `solve()` call would.

## File structure

```
tsp_solver/
├── CMakeLists.txt              # catkin build config; builds MrsTspSolver from tsp_solver.cpp + solver_nodelet.cpp
├── package.xml                 # package manifest / dependencies
├── nodelets.xml                # pluginlib export: tsp_solver/Solver -> tsp_solver::Solver
├── README.md                   # this file
├── config/
│   └── tsp_solver.yaml         # runtime-tunable params (tsp/max_duration, lkh_time_limit, lkh_runs, dir_weight, guide_radius; vp_weight is loaded but unused)
├── launch/
│   └── tsp_solver.launch       # standalone/nodelet-manager launch, topic/service remaps
├── srv/
│   ├── SetStart.srv            # geometry_msgs/Point position -> bool success
│   └── Solve.srv                # geometry_msgs/Vector3 velocity -> geometry_msgs/Point[] path, uint32[] ids, bool success, string message
├── include/tsp_solver/
│   └── tsp_solver.hpp          # TSPsolver class declaration + planner_t node struct
├── src/
│   ├── tsp_solver.cpp          # TSPsolver implementation (distance graph, LKH invocation)
│   └── solver_nodelet.cpp      # tsp_solver::Solver nodelet: ROS glue around TSPsolver
└── LKH/                        # vendored third-party LKH TSP solver (binary + generated params/cost-matrix/solution files); not authored in this project, do not modify
```

## Dependencies

From `package.xml`/`CMakeLists.txt`:
- **catkin build deps**: `cmake_modules`, `mrs_lib`, `nodelet`, `roscpp`, `rospy`, `octomap_msgs`, `octomap_ros`, `message_generation`/`message_runtime`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `std_srvs`, `pcl_ros`, `pcl_conversions`.
- **In-workspace package deps**: `frontier_detector` (for `frontier_detector/FrontierArray` and `Frontier`/`Viewpoint` message types consumed in `syncFrontiers`), `explorer_utils` (for the `BIG_DISTANCE` unreachable-path sentinel), and `path_planner` (for the `path_planner/FindSimplifiedPath` service type called by `findPathDistances`).
- **External libraries**: OctoMap (`octomap`/`octomath` point types), Eigen3 (cost matrix), PCL (`pcl::KdTreeFLANN`, point cloud conversions).
- **Runtime (not a linked library)**: the bundled `LKH/LKH` binary, invoked via `std::system()` at solve time — this is a hard runtime dependency of the solve path, resolved via `ros::package::getPath("tsp_solver")`. `syncFrontiers`/`setStart` additionally require a live `path_planner` nodelet serving `~find_simplified_path` — if unavailable, `findPathDistances` reports `BIG_DISTANCE` for every requested goal.
