#!/usr/bin/env python3
"""Compute a world's flight-zone bounds from its .ply point cloud.

One-off generator: run manually (or via scripts/migrate_worlds.sh) to populate
that world's cached info file under worlds/<world_name>/world_info.yaml. Not
invoked on every run.sh call.

Usage: worldbounds.py <path/to/world.ply> [world_type]
Prints world_type (if given: turbine, pylon or bridge) and the bounds as flat
YAML scalars to stdout.
"""

import sys

import numpy as np
from plyfile import PlyData

WORLD_TYPES = ("turbine", "pylon", "bridge")
ZONE_MARGIN = 5  # [m]


def main():
    if len(sys.argv) not in (2, 3):
        print(f"usage: {sys.argv[0]} <path/to/world.ply> [world_type]", file=sys.stderr)
        return 1

    ply_path = sys.argv[1]
    world_type = sys.argv[2] if len(sys.argv) == 3 else None
    if world_type is not None and world_type not in WORLD_TYPES:
        print(f"unknown world_type '{world_type}', expected one of {WORLD_TYPES}", file=sys.stderr)
        return 1
    plydata = PlyData.read(ply_path)

    data = plydata.elements[0].data
    data = np.array([np.array(list(d)) for d in data])
    data = data[:, :3]

    lb = np.abs(np.min(data, axis=0))
    ub = np.abs(np.max(data, axis=0))

    extent = np.round(np.max([lb, ub], axis=0)).astype(int)
    # flight-zone margin around the object: offline FC-Planner viewpoints sit ~3 m (up to several m) from the surface, and
    # path_planner only samples roadmap nodes inside the zone, so it must cover them
    top = extent + ZONE_MARGIN

    spawn_x = extent[0] + 1  # 1 m off the object's x extent, independent of the margin
    zone_x = 2 * top[0]
    zone_y = 2 * top[1]
    zone_z = top[2]

    if world_type is not None:
        print(f'world_type: "{world_type}"')
    print(f'spawn_position: "{spawn_x} 0 1 0"')
    print(f"zone_x: {zone_x}")
    print(f"zone_y: {zone_y}")
    print(f"zone_z: {zone_z}")
    if world_type == "pylon":
        # yaw symmetry order about the vertical axis, read by skeleton_estimator; 2 = 180 deg symmetric (set 1 by hand for none)
        print("symmetry_order: 2")
    return 0


if __name__ == "__main__":
    sys.exit(main())
