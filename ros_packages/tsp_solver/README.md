# tsp_solver

## Purpose

`tsp_solver` orders exploration viewpoints (frontiers, plus a start node for the UAV's
current position) into a single global visiting tour. It runs as a standalone nodelet
decoupled from `mrs_octomap_planner`'s `Explorer` and from `path_planning`: it keeps its own
distance graph in sync with the live frontier set by querying `path_planning`'s
`~find_simplified_path_in` service for real (roadmap, raycast-simplified) travel distances
between nodes, and solves the resulting (asymmetric) TSP by shelling out to the bundled LKH
solver binary.
`Explorer::makePath()` drives it purely through two blocking services to obtain an ordered
tour, which it then hands to `path_planning` in chunks to get a flyable path.

## ROS interface

### Topics subscribed

| Topic (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~frontiers_in` | `frontier_detection/frontiers` | `frontier_detection/FrontierArray` | Keeps the internal distance graph (`TSPsolver::nodes_`/`distances_`/`id_to_index_`) in sync via `syncFrontiers()`. |
| `~candidate_viewpoints_in` | `skeleton_estimator/candidate_viewpoints` | `sensor_msgs/PointCloud2` | Intended to feed `TSPsolver::setKDtreeInput()` with skeleton-derived guiding viewpoints; the callback (`Solver::callbackCandidateViewpoints`) currently returns immediately after the initialization check, so this path is effectively a no-op at runtime. |

### Services advertised (server)

| Service (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~set_start_in` | `~set_start` | `tsp_solver/SetStart` (`geometry_msgs/Point position` → `bool success`) | Sets/replaces the graph's start node (current UAV position). |
| `~solve_in` | `~solve` | `tsp_solver/Solve` (`geometry_msgs/Vector3 velocity` → `geometry_msgs/Point[] path`, `bool success`, `string message`) | Solves the TSP for the given start velocity/heading and returns the ordered viewpoint tour. |

Both services are called back-to-back by `Explorer::makePath()` (in `mrs_octomap_planner`, not
part of this package) to obtain a fresh tour.

### Services called (client)

| Service (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~find_simplified_path_out` | `path_planning/find_simplified_path` | `path_planning/FindSimplifiedPath` (`geometry_msgs/Point start`, `geometry_msgs/Point[] goals`, `geometry_msgs/Vector3 velocity`, `bool use_raycast` → `path_planning/Path[] paths`, `bool[] success`) | `findPathDistances()` (called from `syncFrontiers()`/`setStart()`) — batched query to `path_planning` for real, raycast-simplified roadmap path lengths from one start node to a set of goal nodes. |

## Important functions/classes

- **`tsp_solver::Solver`** (`solver_nodelet.cpp`) — the nodelet: owns a `TSPsolver` instance, wires up the topic subscriptions and the two services, and translates between ROS messages and `TSPsolver`'s C++ API. It does no locking of its own — `TSPsolver` is internally thread-safe (see the concurrency note below).
- **`tsp_solver::TSPsolver`** (`tsp_solver.hpp`/`tsp_solver.cpp`) — the actual solver logic, independent of ROS nodelet machinery. The distance graph is stored flat rather than as nested maps: `nodes_` (a `std::vector<planner_t>`, `planner_t` holding just id/position/reachability) and a same-indexed `distances_` (`Eigen::MatrixXf`, over-allocated and grown via `conservativeResize` as needed) are kept in sync through `id_to_index_` (`std::unordered_map<unsigned long, int>`). The start node (id `START_ID` = 0) is always pinned to index 0 (`reserveStartSlot()`, run once on the first `setStart()` call) — `constructDistanceMatrix()`'s zero-cost "return to start" column depends on that. Removing any other node (`removeNode`) swaps the last node into its slot instead of shifting everything, keeping indices dense in O(1).
- **`syncFrontiers`** — reconciles the distance graph with an incoming `FrontierArray`: removes nodes for frontiers no longer present (`removeNode`, never called for `START_ID`), then for each new frontier's first viewpoint appends a node (`addNode`) with distances to all existing nodes obtained from a single batched `findPathDistances` call.
- **`setStart`** — (re)sets the start/current-position node's position and recomputes its distances to every other node via one batched `findPathDistances` call; marks unreachable neighbors (`BIG_DISTANCE`) as inaccessible. On the very first call it also reserves index 0 for the start node (`reserveStartSlot()`).
- **`findPathDistances`** — issues one `~find_simplified_path_out` call from a given start position to a batch of goal positions (a single shared Dijkstra search over all goals, per `path_planning`'s own batching design); returns each goal's path length as the sum of the returned simplified path's segment lengths, or `BIG_DISTANCE` for a goal `path_planning` couldn't reach or if the service call itself failed.
- **`setKDtreeInput`** — loads a point cloud of guiding viewpoints into a `pcl::KdTreeFLANN` used to bias the cost matrix toward them (only exercised if the candidate-viewpoints callback is re-enabled).
- **`constructDistanceMatrix`** — flattens the current `distances_`/`nodes_` into the dense `cost_matrix_`/`viewpoint_positions_`/`isAccesible_` fed to `LKHSolve()` via a single block copy (no per-node lookups), adding a start-heading penalty and (if set) KD-tree viewpoint weighting to the start row. Must be called with `mutex_dist_graph_` held.
- **`solve(octomath::Vector3 velocity)`** — main entry point used by the nodelet: snapshots the distance graph (`constructDistanceMatrix()`, briefly under `mutex_dist_graph_`) and calls `LKHSolve()`, then maps the returned tour indices back to viewpoint positions, discarding (returning an empty tour) if any index is out of range for the current viewpoint count.
- **LKH invocation (`GlobalParWrite` / `GlobalProblemWrite` / `LKHSolve` / `GlobalResultsRead`)** — `GlobalParWrite()` writes the LKH `.par` control file once, from the `TSPsolver` constructor (its contents are fixed for the process lifetime). Each `LKHSolve()` call writes the current cost matrix out as an explicit-weight ATSP problem file (`cost_matrix.txt`), removes any previous `solution.txt`, shells out via `std::system()` to the bundled `LKH/LKH` binary, and — only if that call exits successfully — parses the fresh `solution.txt`'s `TOUR_SECTION` back into a 0-based node index tour; a nonzero exit code is logged and returns an empty tour rather than risking a stale/mismatched `solution.txt` being parsed. Paths are resolved relative to `ros::package::getPath("tsp_solver")`.

### Concurrency

`TSPsolver` guards its own state with two mutexes instead of relying on a single external lock: `mutex_dist_graph_` protects `nodes_`/`distances_`/`id_to_index_` and is held by `syncFrontiers()`, `setStart()`, and only the brief `constructDistanceMatrix()` snapshot inside `solve()`; `mutex_solve_` is held for the whole of `solve()` (guarding `cost_matrix_`/`viewpoint_positions_`/`isAccesible_` and the LKH input/output files against concurrent solves). Because the two are separate, the blocking `std::system()` LKH call — up to `lkh_time_limit * lkh_runs` seconds — no longer stalls incoming frontier/start-position updates the way a single shared lock held across the whole `solve()` call would.

## File structure

```
tsp_solver/
├── CMakeLists.txt              # catkin build config; builds MrsTspSolver from tsp_solver.cpp + solver_nodelet.cpp
├── package.xml                 # package manifest / dependencies
├── nodelets.xml                # pluginlib export: tsp_solver/Solver -> tsp_solver::Solver
├── README.md                   # this file
├── config/
│   └── tsp_solver.yaml         # runtime-tunable params (tsp/max_duration, lkh_time_limit, lkh_runs)
├── launch/
│   └── tsp_solver.launch       # standalone/nodelet-manager launch, topic/service remaps
├── srv/
│   ├── SetStart.srv            # geometry_msgs/Point position -> bool success
│   └── Solve.srv                # geometry_msgs/Vector3 velocity -> geometry_msgs/Point[] path, bool success, string message
├── include/tsp_solver/
│   └── tsp_solver.hpp          # TSPsolver class declaration + planner_t node struct
├── src/
│   ├── tsp_solver.cpp          # TSPsolver implementation (distance graph, LKH invocation)
│   └── solver_nodelet.cpp      # tsp_solver::Solver nodelet: ROS glue around TSPsolver
└── LKH/                        # vendored third-party LKH TSP solver (binary + generated params/cost-matrix/solution files); not authored in this project, do not modify
```

## Dependencies

From `package.xml`/`CMakeLists.txt`:
- **catkin build deps**: `cmake_modules`, `mrs_lib`, `nodelet`, `roscpp`, `rospy`, `octomap_msgs`, `octomap_ros`, `message_generation`/`message_runtime`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `pcl_ros`, `pcl_conversions`.
- **In-workspace package deps**: `frontier_detection` (for `frontier_detection/FrontierArray` and `Frontier`/`Viewpoint` message types consumed in `syncFrontiers`), `octomap_planner_utils` (for the `BIG_DISTANCE` unreachable-path sentinel), and `path_planning` (for the `path_planning/FindSimplifiedPath` service type called by `findPathDistances`).
- **External libraries**: OctoMap (`octomap`/`octomath` point types), Eigen3 (cost matrix), PCL (`pcl::KdTreeFLANN`, point cloud conversions).
- **Runtime (not a linked library)**: the bundled `LKH/LKH` binary, invoked via `std::system()` at solve time — this is a hard runtime dependency of the solve path, resolved via `ros::package::getPath("tsp_solver")`. `syncFrontiers`/`setStart` additionally require a live `path_planning` nodelet serving `~find_simplified_path` — if unavailable, `findPathDistances` reports `BIG_DISTANCE` for every requested goal.
