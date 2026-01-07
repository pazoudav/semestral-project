# Octomap Mapping & Planning


* [mrs_uav_core](http://github.com/ctu-mrs/mrs_uav_core)
* [mrs_octomap_mapping_planning](http://github.com/ctu-mrs/mrs_octomap_mapping_planning)

## Packages

## Example session

to build and run the package fot the first time
./run.sh -c

 subsequently 
./run.sh

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

