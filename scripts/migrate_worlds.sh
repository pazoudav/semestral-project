#!/bin/bash
# One-off migration: copies each world's .world, .ply and .dae from the
# external workspace-level worlds/, models/ply/ and models/dae/ dirs into
# this repo's own worlds/<name>/, rewrites the copied .world's mesh <uri> to
# a relative path resolved via GAZEBO_MODEL_PATH (see run.sh), and generates
# bounds.yaml for any world that doesn't already have one cached.
set -e

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
REPO_ROOT=$(readlink -f "$SCRIPTPATH/..")
WORKSPACE_ROOT=$(readlink -f "$REPO_ROOT/../..")

SRC_WORLDS_DIR="$WORKSPACE_ROOT/worlds"
SRC_MODELS_DIR="$WORKSPACE_ROOT/models/ply"
SRC_DAE_DIR="$WORKSPACE_ROOT/models/dae"
DEST_WORLDS_DIR="$REPO_ROOT/worlds"

WORLDBOUNDS_DIR="$SCRIPTPATH/worldbounds"
WORLDBOUNDS_VENV="$WORLDBOUNDS_DIR/.venv"

if [ ! -x "$WORLDBOUNDS_VENV/bin/python3" ]; then
    python3 -m venv "$WORLDBOUNDS_VENV"
    "$WORLDBOUNDS_VENV/bin/pip" install --quiet -r "$WORLDBOUNDS_DIR/requirements.txt"
fi

for world_file in "$SRC_WORLDS_DIR"/*.world; do
    name=$(basename "$world_file" .world)
    ply_file="$SRC_MODELS_DIR/$name.ply"
    dest_dir="$DEST_WORLDS_DIR/$name"

    if [ ! -f "$ply_file" ]; then
        echo "skipping '$name': no matching .ply at $ply_file" >&2
        continue
    fi

    mkdir -p "$dest_dir"
    cp "$world_file" "$dest_dir/$name.world"
    cp "$ply_file" "$dest_dir/$name.ply"

    dae_file="$SRC_DAE_DIR/$name.dae"
    if [ -f "$dae_file" ]; then
        cp "$dae_file" "$dest_dir/$name.dae"
        # relative to DEST_WORLDS_DIR, which run.sh adds to GAZEBO_MODEL_PATH
        sed -i "s|file://$SRC_DAE_DIR/$name.dae|file://$name/$name.dae|" "$dest_dir/$name.world"
    else
        echo "warning: no .dae for '$name' at $dae_file" >&2
    fi

    bounds_file="$dest_dir/bounds.yaml"
    if [ ! -f "$bounds_file" ]; then
        "$WORLDBOUNDS_VENV/bin/python3" "$WORLDBOUNDS_DIR/worldbounds.py" "$dest_dir/$name.ply" > "$bounds_file"
    fi

    echo "$name: ok"
done
