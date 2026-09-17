#!/usr/bin/env python3
"""Compute a world's flight-zone bounds from its .ply point cloud.

One-off generator: run manually (or let run.sh run it once, the first time a
world is used) to populate that world's cached bounds file under
worlds/<world_name>/bounds.yaml. Not invoked on every run.sh call.

Usage: worldbounds.py <path/to/world.ply>
Prints the bounds as flat YAML scalars to stdout.
"""

import sys

import numpy as np
from plyfile import PlyData


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <path/to/world.ply>", file=sys.stderr)
        return 1

    ply_path = sys.argv[1]
    plydata = PlyData.read(ply_path)

    data = plydata.elements[0].data
    data = np.array([np.array(list(d)) for d in data])
    data = data[:, :3]

    lb = np.abs(np.min(data, axis=0))
    ub = np.abs(np.max(data, axis=0))

    top = np.max([lb, ub], axis=0)
    top = np.round(top)
    top += 2.0
    top = top.astype(int)

    spawn_x = top[0] - 1
    zone_x = 2 * top[0]
    zone_y = 2 * top[1]
    zone_z = top[2]

    print(f'spawn_position: "{spawn_x} 0 1 0"')
    print(f"zone_x: {zone_x}")
    print(f"zone_y: {zone_y}")
    print(f"zone_z: {zone_z}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
