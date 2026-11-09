# precompute_paths

Offline precomputation of FC-Planner coverage paths for one object at several scales.
Live exploration will later pick the path whose height is closest to the object size
estimated by `skeleton_estimator` (the scale of its RANSAC transform) and follow it.
This folder covers only the offline part. FC-Planner's code is **not** modified: it runs
through a generic launch file, and the path is captured by echoing FC-Planner's own topics
to files, the same way FC-Planner's `get_views.sh` does.

## Files

| File | Purpose |
|---|---|
| `precompute_paths.sh` | Driver: scales the base model, runs FC-Planner once per scale, dumps results and writes `index.yaml`. |
| `densify_order.py` | Post-processes a dumped `order.txt`: drops consecutive duplicate waypoints (< 5 cm) and splits every segment longer than `DENSIFY_STEP` on the straight chord. Original waypoints keep their exact positions. Uses the system `python3` (PyYAML ships with ROS). |
| `scale_models.py` | Writes uniformly scaled copies (`.ply` + `.pcd`) of the base model for heights `h_min, h_min+STEP, ..., h_max`. |
| `precompute.launch` | Generic FC-Planner launch (copy of FC-Planner's `turbine-90-40.launch` with model/output/start/size params as args). |
| `requirements.txt` | Python deps for `scale_models.py` (installed into `.venv/` automatically). |

## Prerequisites

- An **unmodified upstream** FC-Planner, built as its own catkin workspace. The driver sources its
  `devel/setup.bash` itself. Default `~/user_ros_workspace/src/FC-Planner-offline/FC-Planner`, override with
  `FC_PLANNER_DIR=...`. Do not use the `~/user_ros_workspace/src/FC-Planner` copy: its ROSA was modified to take a
  live octomap cloud instead of loading `rosa_main/pcd`, so offline planning aborts in `ROSA_main::pcloud_read_off()`.
  Setup (the root `CATKIN_IGNORE` keeps the main workspace's `catkin build` from picking it up):
  ```bash
  cd ~/user_ros_workspace/src
  git clone https://github.com/HKUST-Aerial-Robotics/FC-Planner.git FC-Planner-offline
  touch FC-Planner-offline/CATKIN_IGNORE
  cd FC-Planner-offline/FC-Planner
  catkin config --extend /opt/ros/noetic -DCMAKE_BUILD_TYPE=Release
  catkin build -p 1 -j 4 --cmake-args -Wno-dev
  cd src/hierarchical_coverage_planner/solution/LKH && make
  ```
  Keep the build parallelism low (`-p 1 -j 4`). `hierarchical_coverage_planner` (heavy Eigen/PCL/OpenCV templates)
  runs gcc out of memory with catkin's defaults, which shows up as `internal compiler error: Segmentation fault`.
- Base model as `<worlds>/<name>/<name>.ply` + `<worlds>/<name>/<name>.pcd`, default this repo's `worlds/`
  (override with `WORLDS_DIR=...`). `<name>` is the world name, e.g. `turbine-90-40`. The `.pcd` sits next to the
  world's `.ply`/`.dae`/`.world` (like the rest of `worlds/`, it is machine-local and gitignored).
- **The base model frame must match `skeleton_estimator`'s source skeleton**
  (`ros_packages/skeleton_estimator/data/<world_type>/skeleton.outmsg`): tower axis at XY (0,0), ground at z=0.
  Models are scaled about the origin, the same way the RANSAC transform scales.
- No simulation running: the driver publishes on `/move_base_simple/goal` and starts its own `roscore` if none is up.

## Usage

```bash
./precompute_paths.sh <base_model_name> <h_min> <h_max> [launch_arg:=value ...]

# examples
./precompute_paths.sh turbine-90-40 20 120
./precompute_paths.sh pylon-34 20 50 estimation_num:=700 upper_bound_original_points_num:=15000 global_runs:=300 local_runs:=300
```

- `<world_type>` (`turbine`/`pylon`/`bridge`, which picks the output dir) is read from the world's
  `worlds/<base_model_name>/world_info.yaml`, the same way `tmux/session.yml` does.

- Heights run `h_min, h_min+STEP, ..., h_max` (20 m is a sensible `h_min`; very small scales tend to fail, see
  below). An `h_min` of 0 is allowed, but the 0 m scale itself is skipped and the run starts at `STEP`. Path `hH` is planned on the
  base model scaled by `H / h_base`, where `h_base` is the base `.ply`'s max z.
- Extra `launch_arg:=value` pairs are forwarded to `precompute.launch` for every scale. The size-dependent ones are
  `estimation_num`, `upper_bound_original_points_num`, `global_runs` and `local_runs` (defaults are the turbine values;
  the pylon values are shown above). Camera/stand-off params (`fov_h`, `fov_w`, `max_dist`, `viewpoints_distance`, ...)
  can be overridden the same way. Keep them constant across one run and consistent with the sim camera.

Environment variables:

| Variable | Default | Meaning |
|---|---|---|
| `STEP` | `5` | Height increment [m]. |
| `START` | `"0.0 0.0 0.0"` | FC-Planner start position (`hcplanner/current_{x,y,z}_`). The tour order depends on it. |
| `DENSIFY_STEP` | `1.0` | Max waypoint spacing [m] in the final `order.txt` (`densify_order.py`); `0` keeps FC-Planner's raw waypoints. FC-Planner spaces them up to `1.2 · viewpoints_distance · tan(min(fov)/2)` (3.6 m with the defaults) apart, and that bound has no launch arg. |
| `TIMEOUT` | `3600` | Max seconds to wait for one scale's planning to finish. |
| `WORLDS_DIR` | `<repo>/worlds` | Base model location (`<WORLDS_DIR>/<name>/<name>.{ply,pcd}`). |
| `FC_PLANNER_DIR` | `~/user_ros_workspace/src/FC-Planner-offline/FC-Planner` | FC-Planner catkin workspace (unmodified upstream). |

Each scale takes as long as one FC-Planner run, which grows with the model size. Progress is printed per scale.

## What it does per scale

1. `roslaunch precompute.launch` with the scaled model, in the background (node output → `log.txt`).
2. Waits until `hcplanner_exec` subscribes to `/move_base_simple/goal`. The subscriber is created only after the
   model is loaded and the map is built, so this replaces `get_views.sh`'s fixed `sleep`.
3. Publishes the trigger goal. FC-Planner plans synchronously inside that callback, then its visualization timers
   republish the result every 0.5 s, so the first echoed message is always the finished result.
4. `rostopic echo -n 1` → `order.txt` and `viewpoints.txt`. FC-Planner writes `TrajInfo_*.txt` itself.
5. If `DENSIFY_STEP > 0`: `order.txt` → `order_raw.txt`, and `densify_order.py` writes the deduplicated, densified `order.txt`.
6. `rosnode kill /hcplanner_exec`, then reads the path length and coverage from the log into `index.yaml`.

## Output

Written to `ros_packages/skeleton_estimator/data/<world_type>/paths/`. A rerun overwrites the index and each `hH/` it regenerates:

```
paths/
├── index.yaml                      # world_type, base_model, h_base, step, densify_step, start, and per-scale entries
└── h<H>/
    ├── order.txt                   # /hcoppTraj/drone_status, deduplicated + densified to DENSIFY_STEP
    ├── order_raw.txt               # raw /hcoppTraj/drone_status dump (only when DENSIFY_STEP > 0)
    ├── viewpoints.txt              # /hcopp/finalvps_drones
    ├── TrajInfo_<type>-h<H>.txt    # dense trajectory
    └── log.txt                     # FC-Planner output
```

| File | Content | How to read it |
|---|---|---|
| `order.txt` | `visualization_msgs/MarkerArray` (TEXT_VIEW_FACING) as YAML. One marker per trajectory waypoint (viewpoints plus connector points) **in tour order**. `text` is the index and `pose.position` the waypoint. With `DENSIFY_STEP > 0` it also holds the points inserted on the chords (at most `DENSIFY_STEP` apart), renumbered; `order_raw.txt` is the untouched dump. | The path positions. |
| `viewpoints.txt` | `visualization_msgs/MarkerArray` (MESH_RESOURCE) as YAML. One marker per final viewpoint. `pose.position` is the viewpoint and `pose.orientation` its **yaw** (rotation about Z). Grouped by sub-space, **not** in tour order. | Match to `order.txt` by position (≈ equal) to get yaw and which waypoints are viewpoints. |
| `TrajInfo_*.txt` | `TIMESTAMP, X, Y, Z, PITCH, YAW` sampled every 0.1 s along FC-Planner's MINCO trajectory. | Fallback: yaw for connector points, or a densely sampled path. |

`rostopic echo` output is valid YAML (one document, terminated by `---`), so the files can be loaded with yaml-cpp/PyYAML
directly. All coordinates are in the scaled model's frame (the `frame_id: uav1/world_origin` in the markers is just
FC-Planner's hardcoded label). Mapping a path into the live world is `p_live = R(yaw_est) * (s_est / scale) * p + t_est`,
where `s_est / scale` ≈ 1 for the closest height.

`index.yaml` entry per scale:

```yaml
  - height: 45          # [m] target height of this scaled model
    scale: 0.5          # factor applied to the base model
    dir: h45
    status: ok          # ok | failed_init | failed_planning | failed_viewpoints | failed_densify
    path_length: 312.4  # [m] from FC-Planner's [PathAnalyzer]
    coverage_percent: 97.1
```

Very small scales (a few meters) can fail or give poor paths, because stand-off distance, safe height and map
resolution stay fixed in meters. Those entries are kept with their `status` instead of aborting the run.

Intermediate files (scaled models, FC-Planner's polynomial coefficient and visible-cloud dumps) go to `work/` here
and are gitignored.

## Checking a result

Replay one scale in RViz next to the scaled mesh (`work/<world_type>/models/<type>-h<H>.ply`):

```bash
rostopic pub -l /check/order      visualization_msgs/MarkerArray -f ros_packages/skeleton_estimator/data/<type>/paths/h<H>/order.txt
rostopic pub -l /check/viewpoints visualization_msgs/MarkerArray -f ros_packages/skeleton_estimator/data/<type>/paths/h<H>/viewpoints.txt
```

Set the RViz fixed frame to `uav1/world_origin`. Check that the path stays outside the mesh, that the viewpoints face
the surface, and that coverage in `index.yaml` is acceptable.
