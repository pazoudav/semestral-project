#!/bin/bash

set -e

BUILD=false
CLEAN=false

while getopts "cb" opt; do
    case "$opt" in
        c) CLEAN=true; BUILD=true ;;
        b) BUILD=true ;;
    esac
done
shift $((OPTIND - 1))

WORLD_NAME="$1"

if [ -z "$WORLD_NAME" ]; then
    echo "usage: $0 [-c|-b] <world_name>" >&2
    exit 1
fi

if [ "$CLEAN" == true ]; then
    catkin clean -y
fi

if [ "$BUILD" == true ]; then
    catkin init

    clear

    catkin build skeleton_estimator mrs_octomap_planner mrs_octomap_mapping_planning frontier_detection  octomap_planner_utils path_planning #prm_solver
fi



bash tmux/start.sh "$WORLD_NAME"


# Absolute path to this script and the workspace it lives in
# (.../user_ros_workspace/src/semestral-project/run.sh), so nothing here
# hardcodes a username or install location.
# SCRIPT=$(readlink -f "$0")
# SCRIPTPATH=$(dirname "$SCRIPT")
# WORLDS_DIR="$SCRIPTPATH/worlds"

# WORLDBOUNDS_DIR="$SCRIPTPATH/scripts/worldbounds"
# WORLDBOUNDS_VENV="$WORLDBOUNDS_DIR/.venv"
# WORLD_NAME="${@: -1}"
# WORLD_DIR="$WORLDS_DIR/$WORLD_NAME"
# WORLD_FILE="$WORLD_DIR/$WORLD_NAME.world"
# PLY_FILE="$WORLD_DIR/$WORLD_NAME.ply"

# if [ ! -f "$WORLD_FILE" ] || [ ! -f "$PLY_FILE" ]; then
#     echo "error: world '$WORLD_NAME' not found (expected $WORLD_FILE and $PLY_FILE)" >&2
#     echo "available worlds:" >&2
#     ls "$WORLDS_DIR" | sed 's/^/  /' >&2
#     exit 1
# fi

# BOUNDS_FILE="$WORLD_DIR/bounds.yaml"

# if [ ! -f "$BOUNDS_FILE" ]; then
#     echo "no cached bounds for '$WORLD_NAME', computing from $PLY_FILE..."

#     if [ ! -x "$WORLDBOUNDS_VENV/bin/python3" ]; then
#         python3 -m venv "$WORLDBOUNDS_VENV"
#         "$WORLDBOUNDS_VENV/bin/pip" install --quiet -r "$WORLDBOUNDS_DIR/requirements.txt"
#     fi

#     "$WORLDBOUNDS_VENV/bin/python3" "$WORLDBOUNDS_DIR/worldbounds.py" "$PLY_FILE" > "$BOUNDS_FILE"
# fi

# cat "$BOUNDS_FILE"

