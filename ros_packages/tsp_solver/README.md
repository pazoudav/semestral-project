# tsp_solver

## Purpose

`tsp_solver` orders exploration viewpoints (frontiers, plus a start node for the UAV's
current position) into a single global visiting tour. It runs as a standalone nodelet
decoupled from `mrs_octomap_planner`'s `Explorer` and from `prm_solver`: it keeps its own
distance graph in sync with the live frontier set using a fixed Euclidean distance metric
(independent of the PRM roadmap), and solves the resulting (asymmetric) TSP by shelling out
to the bundled LKH solver binary, with a greedy-construction + randomized 3-opt local search
as a fallback. `Explorer::makePath()` drives it purely through two blocking services to obtain
an ordered tour, which it then hands to `prm_solver` in chunks to get a flyable path.

## ROS interface

### Topics subscribed

| Topic (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~frontiers_in` | `frontier_detection/frontiers` | `frontier_detection/FrontierArray` | Keeps the internal distance graph (`TSPsolver::dist_map_`) in sync via `syncFrontiers()`. |
| `~candidate_viewpoints_in` | `candidate_viewpoints` | `sensor_msgs/PointCloud2` | Intended to feed `TSPsolver::setKDtreeInput()` with skeleton-derived guiding viewpoints; the callback (`Solver::callbackCandidateViewpoints`) currently returns immediately after the initialization check, so this path is effectively a no-op at runtime. |

### Services

| Service (private name) | Remap (launch) | Type | Purpose |
|---|---|---|---|
| `~set_start_in` | `~set_start` | `tsp_solver/SetStart` (`geometry_msgs/Point position` → `bool success`) | Sets/replaces the graph's start node (current UAV position). |
| `~solve_in` | `~solve` | `tsp_solver/Solve` (`geometry_msgs/Vector3 velocity` → `geometry_msgs/Point[] path`, `bool success`, `string message`) | Solves the TSP for the given start velocity/heading and returns the ordered viewpoint tour. |

Both services are called back-to-back by `Explorer::makePath()` (in `mrs_octomap_planner`, not
part of this package) to obtain a fresh tour.

## Important functions/classes

- **`tsp_solver::Solver`** (`solver_nodelet.cpp`) — the nodelet: owns a `TSPsolver` instance, wires up the topic subscriptions and the two services, and translates between ROS messages and `TSPsolver`'s C++ API.
- **`tsp_solver::TSPsolver`** (`tsp_solver.hpp`/`tsp_solver.cpp`) — the actual solver logic, independent of ROS nodelet machinery.
- **`syncFrontiers`** — reconciles the internal distance graph with an incoming `FrontierArray`: removes nodes/distances for frontiers no longer present, adds a node (with pairwise Euclidean distances to all existing nodes) for each new frontier's first viewpoint.
- **`setStart`** — (re)inserts the start/current-position node and recomputes its distances to every other node; marks unreachable neighbors (`BIG_DISTANCE`) as inaccessible.
- **`setKDtreeInput`** — loads a point cloud of guiding viewpoints into a `pcl::KdTreeFLANN` used to bias the cost matrix toward them (only exercised if the candidate-viewpoints callback is re-enabled).
- **`constructDistanceMatrix`** — flattens `dist_map_` into the dense `cost_matrix_`/`viewpoint_positions_`/`isAccesible_` used by both solve paths, adding a start-heading penalty and (if set) KD-tree viewpoint weighting to the start row.
- **`solve(octomath::Vector3 velocity)`** — main entry point used by the nodelet: builds the distance matrix and calls `LKHSolve()`, returning the ordered viewpoint positions.
- **LKH invocation (`LKHSolve` / `GlobalParWrite` / `GlobalProblemWrite` / `GlobalResultsRead`)** — writes an LKH `.par` control file and an explicit-weight ATSP problem file (`cost_matrix.txt`) under `LKH/`, shells out via `std::system()` to the bundled `LKH/LKH` binary, then parses `solution.txt`'s `TOUR_SECTION` back into a 0-based node index tour. Paths are resolved relative to `ros::package::getPath("tsp_solver")`.
- **Greedy/random fallback (`solve(cost_matrix, reuse_solution)` / `generateGreedySolution` / `generateRadnSolution` / `computeCost` / `computeDistance`)** — an alternative, non-LKH solver: nearest-neighbor construction followed by randomized 3-opt segment reversals/reinsertions for a configured time budget, keeping any improving move. Not currently called from `solve(velocity)` (which always goes through LKH), but retained as a self-contained alternative.

## File structure

```
tsp_solver/
├── CMakeLists.txt              # catkin build config; builds MrsTspSolver from tsp_solver.cpp + solver_nodelet.cpp
├── package.xml                 # package manifest / dependencies
├── nodelets.xml                # pluginlib export: tsp_solver/Solver -> tsp_solver::Solver
├── README.md                   # this file
├── config/
│   └── tsp_solver.yaml         # runtime-tunable params (tsp/max_duration, the fallback solver's time budget)
├── launch/
│   └── tsp_solver.launch       # standalone/nodelet-manager launch, topic/service remaps
├── srv/
│   ├── SetStart.srv            # geometry_msgs/Point position -> bool success
│   └── Solve.srv                # geometry_msgs/Vector3 velocity -> geometry_msgs/Point[] path, bool success, string message
├── include/tsp_solver/
│   └── tsp_solver.hpp          # TSPsolver class declaration + planner_t node struct
├── src/
│   ├── tsp_solver.cpp          # TSPsolver implementation (distance graph, LKH invocation, greedy/3-opt fallback)
│   └── solver_nodelet.cpp      # tsp_solver::Solver nodelet: ROS glue around TSPsolver
└── LKH/                        # vendored third-party LKH TSP solver (binary + generated params/cost-matrix/solution files); not authored in this project, do not modify
```

## Dependencies

From `package.xml`/`CMakeLists.txt`:
- **catkin build deps**: `cmake_modules`, `mrs_lib`, `nodelet`, `roscpp`, `rospy`, `octomap_msgs`, `octomap_ros`, `message_generation`/`message_runtime`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `pcl_ros`, `pcl_conversions`.
- **In-workspace package deps**: `frontier_detection` (for `frontier_detection/FrontierArray` and `Frontier`/`Viewpoint` message types consumed in `syncFrontiers`) and `octomap_planner_utils` (for `distance_funcion_t`, `path_info_t`, `INVALID_DISTANCE`/`BIG_DISTANCE`, and `getRand` used by the fallback solver).
- **External libraries**: OctoMap (`octomap`/`octomath` point types), Eigen3 (cost matrix), PCL (`pcl::KdTreeFLANN`, point cloud conversions).
- **Runtime (not a linked library)**: the bundled `LKH/LKH` binary, invoked via `std::system()` at solve time — this is a hard runtime dependency of the primary (non-fallback) solve path, resolved via `ros::package::getPath("tsp_solver")`.
