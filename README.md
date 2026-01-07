# Octomap Mapping & Planning

Metapackage containing MRS Octomap-based 3D mapper and planner.

![](./.fig/octomap_server.png)

> :warning: **Attention please: This README is outdated.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid.

## Dependencies

* [mrs_uav_core](http://github.com/ctu-mrs/mrs_uav_core)
* [mrs_uav_modules](http://github.com/ctu-mrs/mrs_uav_modules)

## Packages

* mrs_octomap_mapping_planning - launch files, example tmux session
* [mrs_octomap_server](https://github.com/ctu-mrs/mrs_octomap_server) - Uses Octomap to build global & local map
* [mrs_octomap_planner](https://github.com/ctu-mrs/mrs_octomap_planner) - 3D planner for UAVs
* [mrs_octomap_tools](https://github.com/ctu-mrs/mrs_octomap_tools) - MRS Tools and libraries for Octomap

## Example session

Example tmuxinator session is provided in the `tmux` subfolder.

## Main launch file

The [launch file](./ros_packages/octomap_mapping_planning/launch/mapplan.launch)
```
./ros_packages/octomap_mapping_planning/launch/mapplan.launch
```
is prepared to launch

* PointCloud filter ([mrs_pcl_tools](https://github.com/ctu-mrs/mrs_pcl_tools)),
* Octomap Server,
* Octomap Planner,
* Octomap RVIZ Visualizer,
* Nodelet manager.

Please, use provided arguments and custom config files to customize the behaviour of the nodes.

drwxrwxr-x 3 david david 4096 Dec 21 09:13 ./
drwxrwxr-x 6 david david 4096 Dec 21 09:13 ../
-rw-rw-r-- 1 david david 1182 Dec 21 09:13 .gitman.yml
drwxrwxr-x 4 david david 4096 Dec 21 09:13 mrs_octomap_mapping_planning/
lrwxrwxrwx 1 david david   29 Dec 21 09:13 mrs_octomap_planner -> .gitman/mrs_octomap_planner/.
lrwxrwxrwx 1 david david   28 Dec 21 09:13 mrs_octomap_server -> .gitman/mrs_octomap_server/.
lrwxrwxrwx 1 david david   27 Dec 21 09:13 mrs_octomap_tools -> .gitman/mrs_octomap_tools/.
lrwxrwxrwx 1 david david   31 Dec 21 09:13 mrs_subt_planning_lib -> .gitman/mrs_subt_planning_lib/.
