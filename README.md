# Energy-aware UAV coverage path planning in scanning of outdoor structures


# DEMO VIDEO

https://youtu.be/lA2REzf3Iso


## Run info  

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


most of the parameters can be set in explorer.yaml


## how in works / what does it do

- visualize data in rviz explorer tab
- transparent cubes are frontiers (color coded)
- pink cubes are viewpoints
- green lines are a global path thru all viewpoint
- path and trajectory fro the drone

.
- when new global map is recieved, extract frontiers in observed area

.
- periodicaly makes and updates PRM for navigation, 
- PRM navigaiton nodes are dynamicaly creaded and destroid based on their age or if a new scan find that they are no longer in a free space,

.
- periodicaly creates a shortest path (TSP) across all viewpoints,
- makes path to closest viewpoint,
- sets trasjectory, checks if trajectory is viable during flight

## Packages

* [mrs_uav_core](http://github.com/ctu-mrs/mrs_uav_core)
* [mrs_octomap_mapping_planning](http://github.com/ctu-mrs/mrs_octomap_mapping_planning)


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

