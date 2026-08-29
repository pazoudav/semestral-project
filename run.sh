#!/bin/bash



if [ $1 == '-c' ]; then
    catkin clean -y
fi
set -e
catkin init

clear

catkin build octomap_planner_utils
catkin build frontier_detection
catkin build prm_solver
catkin build mrs_octomap_planner
catkin build mrs_octomap_mapping_planning

export WORLD_NAME="${@: -1}"
WB=$(/home/david/user_ros_workspace/data/dist/worldbounds/worldbounds $WORLD_NAME)
ar=($WB)
echo "$WB"
export SPAWN_POSITION="$((ar[0] - 1)) 0 1 0"
export ZONE_X=$((2*ar[0]))
export ZONE_Y=$((2*ar[1]))
export ZONE_Z=$((ar[2]))

bash tmux/start.sh 
