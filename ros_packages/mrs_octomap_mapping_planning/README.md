# mrs_octomap_mapping_planning

## Purpose

This package has no C++/Python source of its own. It exists purely to wire together, in one place, the chain of nodes/nodelets needed to go from raw sensor point clouds to an octomap: point cloud filtering, octomap building, and octomap visualization. Its actual content is a set of `roslaunch` files plus a `.gitman.yml` declaring the external MRS repos those launch files depend on. The exploration planner include is present but commented out.

## What it launches

The top-level launch file is [`launch/mapplan.launch`](launch/mapplan.launch). It takes `UAV_NAME`, `standalone`/`debug` flags, a `nodelet_manager_name` (default `$(arg UAV_NAME)_mapping_nodelet_manager`), a `world_frame_id`, and a set of `custom_config` args (mostly unused by the includes below other than `config_pcl_filter_*`/`config_octomap_server`/`config_octomap_visualizer`), then includes, in order:

1. **Nodelet manager** — [`launch/nodelet_manager.launch`](launch/nodelet_manager.launch), started under `ns="$(arg UAV_NAME)"`. When `standalone` is false it starts a `nodelet manager` node named `$(arg nodelet_manager_name)` (`num_worker_threads` = `n_threads`, default 16) that the other includes load their nodelets into; when `standalone` is true this group is skipped and each nodelet runs as its own standalone node instead.

2. **Ouster point cloud filter** — `mrs_pcl_tools`'s `launch/pcl_filter.launch`, `name_suffix="ouster"`, remapping `topic_3d_lidar_in` from `os_cloud_nodelet/points`, configured via `config_pcl_filter_ouster`.

3. **Octomap server** — `mrs_octomap_server`'s `launch/octomap.launch`, configured via `config_octomap_server`. Its inputs are `pcl_filter_ouster/points_processed` (lidar 3D slot 0, plus its `points_over_max_range`), `pcl_filter_rs_front/points_processed` (depth camera slot 0), `pcl_filter_rs_down/points_processed` (depth camera slot 1), each paired with its `*_over_max_range` and the matching `camera_info` topic (`front_rgbd/aligned_depth_to_color/camera_info`, `down_rgbd/aligned_depth_to_color/camera_info`), plus a 2D lidar input from `rplidar/scan`. `world_frame_id` is passed through from the top-level arg. Only the Ouster filter is launched by this file; the `pcl_filter_rs_front`/`pcl_filter_rs_down` topics must come from filters started elsewhere (the `config_pcl_filter_rs_front`/`config_pcl_filter_rs_down` args are declared but not used by any include).

4. **Octomap RViz visualizers** — two instances of `mrs_octomap_tools`'s `launch/octomap_rviz_visualizer.launch`, configured via `config_octomap_visualizer`: `node_name="octomap_global_vis"` visualizing `octomap_server/octomap_global_binary`, and `node_name="octomap_local_vis"` visualizing `octomap_server/octomap_local_binary`.

**Commented out:** four includes are present in `mapplan.launch` but disabled, each wrapped in an XML comment —
`frontier_detector`'s `launch/detector.launch`, `prm_solver`'s `launch/prm_solver.launch`, `tsp_solver`'s `launch/tsp_solver.launch` (all `standalone="true"`, `debug="false"`), and `explorer`'s `launch/explorer.launch` (`standalone="true"`, `nodelet_manager_name`, `custom_config` = `config_octomap_planner`, `debug="false"`). In the actual running system these nodelets are instead started directly from `tmux/session.yml` at the workspace root (outside this package), not from `mapplan.launch`. The `prm_solver` block is stale: that package no longer exists in this project. `path_planner` and `skeleton_estimator` have no include here at all — they too are started from `tmux/session.yml`.

`mapplan.launch` is the only top-level launch file in this package. [`launch/nodelet_manager.launch`](launch/nodelet_manager.launch) is a helper included by it (described above).

## File structure

```
mrs_octomap_mapping_planning/
├── .gitman.yml                  # gitman manifest: external MRS repos to fetch into ros_packages/ siblings
├── CMakeLists.txt               # catkin_package() only; installs launch/ and tmux/ (tmux/ dir not present in this package)
├── package.xml                  # catkin package manifest and declared dependencies
└── launch/
    ├── mapplan.launch           # main integration launch file, see "What it launches" above
    └── nodelet_manager.launch   # starts the shared nodelet manager (or is a no-op when standalone)
```

There is no `config/` directory in this package; all config overrides used by the includes above (`config_pcl_filter_*`, `config_octomap_server`, `config_octomap_visualizer`) are passed in from outside (empty by default here, overridden at the call site — e.g. from `tmux/session.yml`/`tmux/config/` at the workspace root).

## Dependencies

**Declared in `package.xml`/`CMakeLists.txt`:**
- `catkin` (buildtool)
- `cmake_modules`
- `rospy`
- `mrs_octomap_server`
- `mrs_octomap_tools`
- `mrs_subt_planning_lib`
- `mrs_octomap_planner` (the upstream CTU-MRS package) is listed in `package.xml` but commented out — this project's own planner, the local `explorer` package (a sibling under `ros_packages/`, formerly a fork named `mrs_octomap_planner`), is built locally in this workspace instead.

**gitman-managed external MRS repos** (from [`.gitman.yml`](.gitman.yml), fetched via `gitman`, not vendored under this repo's `src/`):
- `mrs_octomap_tools` — https://github.com/ctu-mrs/octomap_tools.git
- `mrs_octomap_server` — https://github.com/ctu-mrs/mrs_octomap_server.git
- `mrs_subt_planning_lib` — https://github.com/ctu-mrs/subt_planning_lib.git
- `mrs_octomap_planner` — https://github.com/ctu-mrs/mrs_octomap_planner.git — also listed here as a gitman source (rev `master`, linked to `mrs_octomap_planner`), even though this project already provides its own local planner (`ros_packages/explorer`) and `package.xml`'s dependency on it is commented out; this entry appears to be a stale leftover from before the planner was developed locally.

All four are pulled in at the `rev: master` of their respective repos, each with a `git submodule update --init --recursive` post-fetch script, and none of them are checked into this repository's `src/` tree — they must be fetched via `gitman` before a build.
