#!/bin/bash



if [ $1 == '-c' ]; then
    catkin clean -y
fi
set -e
catkin init

clear

catkin build mrs_octomap_planner
catkin build mrs_octomap_mapping_planning

./ros_packages/mrs_octomap_mapping_planning/tmux/simulation_example/start.sh 