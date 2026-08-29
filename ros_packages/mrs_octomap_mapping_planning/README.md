# mrs_octomap_mapping_planning

## Purpose

This package has no C++/Python source of its own. It exists purely to wire together, in one place, the chain of nodes/nodelets needed to go from raw sensor point clouds to a flyable exploration plan: point cloud filtering, octomap building, octomap visualization, and (when enabled) this project's exploration planner. Its actual content is a set of `roslaunch` files plus a `.gitman.yml` declaring the external MRS repos those launch files depend on.

## What it launches

The top-level launch file is [`launch/mapplan.launch`](launch/mapplan.launch). It takes `UAV_NAME`, `standalone`/`debug` flags, a `nodelet_manager_name` (default `$(arg UAV_NAME)_mapping_nodelet_manager`), a `world_frame_id`, and a set of `custom_config` args (mostly unused by the includes below other than `config_pcl_filter_*`/`config_octomap_server`/`config_octomap_visualizer`), then includes, in order:

1. **Nodelet manager** — [`launch/nodelet_manager.launch`](launch/nodelet_manager.launch), started under `ns="$(arg UAV_NAME)"`. When `standalone` is false it starts a `nodelet manager` node named `$(arg nodelet_manager_name)` (`num_worker_threads` = `n_threads`, default 16) that the other includes load their nodelets into; when `standalone` is true this group is skipped and each nodelet runs as its own standalone node instead.

2. **Ouster point cloud filter** — `mrs_pcl_tools`'s `launch/pcl_filter.launch`, `name_suffix="ouster"`, remapping `topic_3d_lidar_in` from `os_cloud_nodelet/points`, configured via `config_pcl_filter_ouster`.

3. **RealSense front depth → point cloud** — this package's own [`launch/depth_to_pc.launch`](launch/depth_to_pc.launch), `name_suffix="_realsense_front"`. This wraps a `depth_image_proc/point_cloud_xyz` nodelet, remapping `camera_info`/`image_rect`/`points` from `front_rgbd/aligned_depth_to_color/{camera_info,image_raw}` to `front_rgbd/points`.

4. **RealSense down depth → point cloud** — same `depth_to_pc.launch`, `name_suffix="_realsense_down"`, converting `down_rgbd/aligned_depth_to_color/{camera_info,image_raw}` to `down_rgbd/points`.

5. **RealSense front point cloud filter** — another `mrs_pcl_tools` `pcl_filter.launch` instance, `name_suffix="rs_front"`, filtering `front_rgbd/points`, configured via `config_pcl_filter_rs_front`.

6. **RealSense down point cloud filter** — another `mrs_pcl_tools` `pcl_filter.launch` instance, `name_suffix="rs_down"`, filtering `down_rgbd/points`, configured via `config_pcl_filter_rs_down`.

7. **Octomap server** — `mrs_octomap_server`'s `launch/octomap.launch`, configured via `config_octomap_server`. It's fed all three filtered/converted streams: `pcl_filter_ouster/points_processed` (lidar 3D slot 0, plus its `points_over_max_range`), `pcl_filter_rs_front/points_processed` (depth camera slot 0), `pcl_filter_rs_down/points_processed` (depth camera slot 1), each paired with its `*_over_max_range` and the matching `camera_info` topic, plus a 2D lidar input from `rplidar/scan`. `world_frame_id` is passed through from the top-level arg.

8. **Octomap RViz visualizers** — two instances of `mrs_octomap_tools`'s `launch/octomap_rviz_visualizer.launch`, configured via `config_octomap_visualizer`: `node_name="octomap_global_vis"` visualizing `octomap_server/octomap_global_full`, and `node_name="octomap_local_vis"` visualizing `octomap_server/octomap_local_full`.

9. **Octomap planner** — `mrs_octomap_planner`'s `launch/explorer.launch`, started with `standalone="true"` (hardcoded, not tied to the top-level `standalone` arg) and `debug="false"`; its `nodelet_manager_name`/`custom_config` args are present but commented out, so it always runs as its own standalone node rather than loading into the shared nodelet manager or taking `config_octomap_planner`.

**Commented out:** three includes are present in `mapplan.launch` but disabled —
`frontier_detection`'s `launch/detector.launch`, `prm_solver`'s `launch/prm_solver.launch`, and `tsp_solver`'s `launch/tsp_solver.launch` (lines ~164–195, each wrapped in an XML comment, `standalone="true"`, `debug="false"`). In the actual running system these three nodelets are instead started directly from `tmux/session.yml` at the workspace root (outside this package), not from `mapplan.launch`. `skeleton_estimator` has no include here at all — it too is started from `tmux/session.yml`.

Two further launch files exist but are not included by `mapplan.launch`:
- [`launch/nodelet_manager.launch`](launch/nodelet_manager.launch) — described above; reused by `mapplan.launch` and `vio_indoor-none.launch`.
- [`launch/depth_to_pc.launch`](launch/depth_to_pc.launch) — described above; reused twice by `mapplan.launch`.
- [`launch/vio_indoor-none.launch`](launch/vio_indoor-none.launch) — an alternate, simpler top-level launch file (indoor/VIO variant, `world_frame_id` defaults to `$(arg UAV_NAME)/vio_origin` instead of `.../world_origin`). It skips the point-cloud filter chain entirely and feeds the octomap server directly from `rgbd/points_processed` / `pico_flexx/points_processed` (with matching `camera_info` topics), then includes the same octomap visualizers and `mrs_octomap_planner`'s `explorer.launch` — but here `standalone`/`nodelet_manager_name`/`custom_config` for the planner are passed through from the top-level args rather than hardcoded. It has no frontier/PRM/TSP includes at all, commented or otherwise. Not referenced by `tmux/session.yml` as far as this package's own files show.

## File structure

```
mrs_octomap_mapping_planning/
├── .gitman.yml                  # gitman manifest: external MRS repos to fetch into ros_packages/ siblings
├── CMakeLists.txt               # catkin_package() only; installs launch/ and tmux/ (tmux/ dir not present in this package)
├── package.xml                  # catkin package manifest and declared dependencies
└── launch/
    ├── mapplan.launch           # main integration launch file, see "What it launches" above
    ├── nodelet_manager.launch   # starts the shared nodelet manager (or is a no-op when standalone)
    ├── depth_to_pc.launch       # wraps depth_image_proc/point_cloud_xyz to convert a depth image to a point cloud
    └── vio_indoor-none.launch   # alternate top-level launch file for an indoor/VIO setup, skips PCL filtering
```

There is no `config/` directory in this package; all config overrides used by the includes above (`config_pcl_filter_*`, `config_octomap_server`, `config_octomap_visualizer`, `config_octomap_planner`) are passed in from outside (empty by default here, overridden at the call site — e.g. from `tmux/session.yml`/`tmux/config/` at the workspace root).

## Dependencies

**Declared in `package.xml`/`CMakeLists.txt`:**
- `catkin` (buildtool)
- `cmake_modules`
- `rospy`
- `mrs_octomap_server`
- `mrs_octomap_tools`
- `mrs_subt_planning_lib`
- `mrs_octomap_planner` is listed in `package.xml` but commented out — this project's own `mrs_octomap_planner` package (a sibling under `ros_packages/`) is built locally in this workspace rather than depended on as an external package.

**gitman-managed external MRS repos** (from [`.gitman.yml`](.gitman.yml), fetched via `gitman`, not vendored under this repo's `src/`):
- `mrs_octomap_tools` — https://github.com/ctu-mrs/octomap_tools.git
- `mrs_octomap_server` — https://github.com/ctu-mrs/mrs_octomap_server.git
- `mrs_subt_planning_lib` — https://github.com/ctu-mrs/subt_planning_lib.git
- `mrs_octomap_planner` — https://github.com/ctu-mrs/mrs_octomap_planner.git — also listed here as a gitman source (rev `master`, linked to `mrs_octomap_planner`), even though this project already provides its own local `ros_packages/mrs_octomap_planner` and `package.xml`'s dependency on it is commented out; this entry appears to be a stale leftover from before the planner was developed locally.

All four are pulled in at the `rev: master` of their respective repos, each with a `git submodule update --init --recursive` post-fetch script, and none of them are checked into this repository's `src/` tree — they must be fetched via `gitman` before a build.
