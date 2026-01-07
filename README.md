# Octomap Mapping & Planning


* [mrs_uav_core](http://github.com/ctu-mrs/mrs_uav_core)
* [mrs_octomap_mapping_planning](http://github.com/ctu-mrs/mrs_octomap_mapping_planning)

## Packages

## Example session

to build and run the package (-c flag cleans the catkin workspace before running)

run.sh [-c]


## implementation

main par of the implementation is in ros_packages/mrs_octomap_planner/
- explorer.cpp
- prm.cpp
- frontier_manager.cpp
- fis.cpp
- utils.cpp
- tsp_solver.cpp
- (octomap_planner.cpp/hpp kept only for reference and isn't used)


most of the paramerts can be set in explorer.yaml


## Main launch file

The [launch file](./ros_packages/octomap_mapping_planning/launch/mapplan.launch)
```
./ros_packages/octomap_mapping_planning/launch/mapplan.launch
```
is prepared to launch

* PointCloud filter ([mrs_pcl_tools](https://github.com/ctu-mrs/mrs_pcl_tools)),
* Octomap Server,
* Octomap Planner <- implemented in mrs_octomap_planner
* Octomap RVIZ Visualizer,
* Nodelet manager.

Please, use provided arguments and custom config files to customize the behaviour of the nodes.

