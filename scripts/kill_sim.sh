#!/bin/bash
# Kills the whole simulation: the mrs tmux server and every gazebo/ros/px4 process left behind, including nodelets
# stopped by gdb (mapplan.launch debug:=true), which ignore everything but SIGKILL.
# Used by crash_hunt.sh between runs, also handy on its own when the session won't close.
#
# NOTE: kills all gazebo/ros/px4 processes on this machine, not only the ones of the mrs session.

SIM_PROCS="gzserver gzclient roslaunch rosmaster roscore rosout nodelet px4 rviz rostopic rosservice"
# catch-all for every other node (mavros_node, static_transform_publisher, mrs_uav_status, ...), matched by the path of
# its binary/script. Orphans of a SIGKILLed roslaunch survive otherwise, a leftover mavros_node keeps the px4 mavlink
# port so the next run never gets /uav1/hw_api/connected and hangs on waitForHw.
ROS_PROC_PATTERN="^(/usr/bin/python3 )?(/opt/ros/|$HOME/user_ros_workspace/devel/)"

sim_running() {
  for p in $SIM_PROCS; do
    pgrep -x "$p" > /dev/null && return 0
  done
  pgrep -f "$ROS_PROC_PATTERN" > /dev/null && return 0
  return 1
}

# first, so killing the watched nodelets is not reported as a crash
pkill -f pause_on_crash.py
# the whole server: a surviving one would not pick up CRASH_MARKER from start.sh
tmux -L mrs kill-server 2> /dev/null
pkill -INT -x roslaunch
for _ in $(seq 20); do
  sim_running || break
  sleep 1
done
pkill -9 -f "gdb.*nodelet"
for p in $SIM_PROCS; do
  pkill -9 -x "$p"
done
pkill -9 -f "$ROS_PROC_PATTERN"
for _ in $(seq 10); do
  sim_running || exit 0
  sleep 1
done
echo "some simulation processes survived SIGKILL:" >&2
for p in $SIM_PROCS; do
  pgrep -a -x "$p" >&2
done
pgrep -a -f "$ROS_PROC_PATTERN" >&2
exit 1
