#!/bin/bash
# Offline FC-Planner coverage path precomputation for one object at several scales.
#
# Scales a base model to heights <h_min>, <h_min>+STEP, ..., <h_max> (scale_models.py), runs
# FC-Planner on each scaled copy (precompute.launch), and dumps the resulting path by
# echoing FC-Planner's own topics to files, the same way FC-Planner's get_views.sh does.
# Results go to ros_packages/skeleton_estimator/data/<world_type>/paths/h<H>/, indexed
# by ros_packages/skeleton_estimator/data/<world_type>/paths/index.yaml.
#
# Usage: precompute_paths.sh <base_model_name> <h_min> <h_max> [launch_arg:=value ...]
# The base model is read from worlds/<base_model_name>/<base_model_name>.{ply,pcd}, and
# <world_type> from that world's world_info.yaml.
# Extra launch_arg:=value pairs are forwarded to precompute.launch for every scale.
# See README.md in this directory for details.
set -e

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
REPO_ROOT=$(readlink -f "$SCRIPTPATH/../..")
WORKSPACE_ROOT=$(readlink -f "$REPO_ROOT/../..")

WORLDS_DIR=${WORLDS_DIR:-"$REPO_ROOT/worlds"}
# unmodified upstream FC-Planner (its catkin workspace is the repo's FC-Planner/ subdir); the src/FC-Planner copy
# next to it has a live-input ROSA that no longer loads the model .pcd, so it can't plan offline
FC_PLANNER_DIR=${FC_PLANNER_DIR:-"$WORKSPACE_ROOT/src/FC-Planner-offline/FC-Planner"}
STEP=${STEP:-5}
TIMEOUT=${TIMEOUT:-3600}
START=${START:-"0.0 0.0 0.0"}
# order.txt segments longer than this are split on the straight chord (densify_order.py; raw dump kept as
# order_raw.txt); 0 disables. FC-Planner itself spaces waypoints up to 1.2 * viewpoints_distance * tan(fov/2) apart
DENSIFY_STEP=${DENSIFY_STEP:-1.0}

if [ $# -lt 3 ]; then
    echo "usage: $0 <base_model_name> <h_min> <h_max> [launch_arg:=value ...]" >&2
    exit 1
fi

BASE_NAME=$1
H_MIN=$2
H_MAX=$3
shift 3
LAUNCH_ARGS=("$@")

read -r START_X START_Y START_Z <<< "$START"

BASE_PLY="$WORLDS_DIR/$BASE_NAME/$BASE_NAME.ply"
BASE_PCD="$WORLDS_DIR/$BASE_NAME/$BASE_NAME.pcd"
WORLD_INFO="$WORLDS_DIR/$BASE_NAME/world_info.yaml"
for f in "$BASE_PLY" "$BASE_PCD" "$WORLD_INFO" "$FC_PLANNER_DIR/devel/setup.bash"; do
    if [ ! -f "$f" ]; then
        echo "missing $f" >&2
        exit 1
    fi
done

# same parsing as tmux/session.yml's pre_window
WORLD_TYPE=$(sed -n "s/^world_type: *\"\(.*\)\"/\1/p" "$WORLD_INFO")
if [ -z "$WORLD_TYPE" ]; then
    echo "no world_type in $WORLD_INFO" >&2
    exit 1
fi
echo "world_type: $WORLD_TYPE (from $WORLD_INFO)"

WORK_DIR="$SCRIPTPATH/work/$WORLD_TYPE"
MODEL_DIR="$WORK_DIR/models"
OUT_ROOT="$REPO_ROOT/ros_packages/skeleton_estimator/data/$WORLD_TYPE/paths"
INDEX="$OUT_ROOT/index.yaml"
mkdir -p "$MODEL_DIR" "$OUT_ROOT"

VENV="$SCRIPTPATH/.venv"
if [ ! -x "$VENV/bin/python3" ]; then
    python3 -m venv "$VENV"
    "$VENV/bin/pip" install --quiet -r "$SCRIPTPATH/requirements.txt"
fi

# setup.bash references unset variables, so it can't run under `set -u`; source it before anything else
# shellcheck disable=SC1091
source "$FC_PLANNER_DIR/devel/setup.bash"

if [ ! -x "$FC_PLANNER_DIR/devel/lib/hierarchical_coverage_planner/hcplanner_exec" ]; then
    echo "hcplanner_exec not built in $FC_PLANNER_DIR, build it first (see README.md, Prerequisites)" >&2
    exit 1
fi

echo "scaling $BASE_NAME from ${H_MIN} m to ${H_MAX} m in ${STEP} m steps"
SCALES=$("$VENV/bin/python3" "$SCRIPTPATH/scale_models.py" "$BASE_PLY" "$BASE_PCD" "$MODEL_DIR" "$WORLD_TYPE" "$H_MIN" "$H_MAX" "$STEP")
H_BASE=$(echo "$SCALES" | awk '$1 == "h_base" {print $2}')

# own roscore, so rostopic calls don't race roslaunch's auto-started master (which also dies with every node kill)
ROSCORE_PID=""
if ! rostopic list >/dev/null 2>&1; then
    roscore >/dev/null 2>&1 &
    ROSCORE_PID=$!
    until rostopic list >/dev/null 2>&1; do sleep 0.5; done
fi

LAUNCH_PID=""
cleanup() {
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        rosnode kill /hcplanner_exec >/dev/null 2>&1 || true
        kill "$LAUNCH_PID" 2>/dev/null || true
    fi
    if [ -n "$ROSCORE_PID" ]; then
        kill "$ROSCORE_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

cat > "$INDEX" <<EOF
world_type: $WORLD_TYPE
base_model: $BASE_NAME
h_base: $H_BASE        # [m] base model max z; path h<H> was planned on the base model scaled by scale = H / h_base about the origin
step: $STEP
densify_step: $DENSIFY_STEP  # [m] max waypoint spacing in order.txt after densification (0 = raw FC-Planner waypoints)
start: [$START_X, $START_Y, $START_Z]
paths:
EOF

# strips the ANSI color codes FC-Planner puts around its ROS_INFO output, then pulls the number after "<key> = "
log_value() {
    sed 's/\x1b\[[0-9;]*m//g' "$1" | grep -a "$2" | tail -n 1 | sed -n 's/.*= *\([-0-9.]*\).*/\1/p'
}

# echo_once <topic> <file> <timeout_s>: dumps one message of <topic> to <file>; fails on timeout or as soon as the
# planner's roslaunch exits (e.g. hcplanner_exec crashed, which takes roslaunch down via required="true")
echo_once() {
    timeout "$3" rostopic echo -n 1 "$1" > "$2" &
    local echo_pid=$!
    while kill -0 "$echo_pid" 2>/dev/null; do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            kill "$echo_pid" 2>/dev/null || true
            wait "$echo_pid" 2>/dev/null || true
            return 1
        fi
        sleep 1
    done
    wait "$echo_pid"
}

# here-string rather than a pipe, so the loop runs in this shell and cleanup() sees LAUNCH_PID
while read -r H K; do
    [ "$H" = "h_base" ] && continue

    NAME="$WORLD_TYPE-h$H"
    OUT_DIR="$OUT_ROOT/h$H"
    SCRATCH_DIR="$WORK_DIR/h$H"
    rm -rf "$OUT_DIR"
    mkdir -p "$OUT_DIR" "$SCRATCH_DIR"
    LOG="$OUT_DIR/log.txt"

    echo "[$NAME] planning (scale $K), log: $LOG"
    roslaunch "$SCRIPTPATH/precompute.launch" \
        name:="$NAME" model_dir:="$MODEL_DIR" out_dir:="$OUT_DIR" work_dir:="$SCRATCH_DIR" \
        start_x:="$START_X" start_y:="$START_Y" start_z:="$START_Z" \
        "${LAUNCH_ARGS[@]}" > "$LOG" 2>&1 &
    LAUNCH_PID=$!

    # the trigger subscriber is created at the end of HCOPP::init, i.e. once the models are loaded and the map is built
    STATUS=ok
    until rostopic info /move_base_simple/goal 2>/dev/null | grep -q hcplanner_exec; do
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            STATUS=failed_init
            break
        fi
        sleep 0.5
    done

    if [ "$STATUS" = ok ]; then
        rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped \
            "{header: {seq: 0, stamp: 0, frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" >/dev/null

        # planning runs synchronously inside the trigger callback and the visualization timers only fire after it
        # returns, so the first message on each topic is the finished result
        if ! echo_once /hcoppTraj/drone_status "$OUT_DIR/order.txt" "$TIMEOUT"; then
            STATUS=failed_planning
        elif ! echo_once /hcopp/finalvps_drones "$OUT_DIR/viewpoints.txt" 60; then
            STATUS=failed_viewpoints
        fi
    fi

    # system python3 (PyYAML ships with ROS), not the venv
    if [ "$STATUS" = ok ] && awk "BEGIN {exit !($DENSIFY_STEP > 0)}"; then
        mv "$OUT_DIR/order.txt" "$OUT_DIR/order_raw.txt"
        if ! python3 "$SCRIPTPATH/densify_order.py" "$OUT_DIR/order_raw.txt" "$OUT_DIR/order.txt" "$DENSIFY_STEP"; then
            STATUS=failed_densify
        fi
    fi

    rosnode kill /hcplanner_exec >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    LAUNCH_PID=""

    LENGTH=$(log_value "$LOG" "path length")
    COVERAGE=$(log_value "$LOG" "trajectory coverage rate")

    cat >> "$INDEX" <<EOF
  - height: $H
    scale: $K
    dir: h$H
    status: $STATUS
    path_length: ${LENGTH:-null}
    coverage_percent: ${COVERAGE:-null}
EOF
    echo "[$NAME] $STATUS (length ${LENGTH:-?} m, coverage ${COVERAGE:-?} %)"
    sleep 1
done <<< "$SCALES"

echo "done, index: $INDEX"
