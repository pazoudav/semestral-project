#!/usr/bin/env python3
"""Generate uniformly scaled copies of a base model for FC-Planner path precomputation.

Each copy is scaled about the origin (the RANSAC transform in skeleton_estimator also
scales about the origin, with no Z offset), so the base model must have its axis at
XY (0,0) and its ground at z=0.

Usage: scale_models.py <base.ply> <base.pcd> <out_dir> <name_prefix> <h_min> <h_max> [step]

For every target height H in h_min, h_min+step, ..., h_max (step defaults to 5 m; a 0 m
h_min is skipped, starting at step instead), writes
<out_dir>/<name_prefix>-h<H>.ply and .pcd, scaled by k = H / h_base, where h_base is
the base .ply's max z. Prints "h_base <h_base>" followed by one "<H> <k>" line per
scale to stdout, for precompute_paths.sh to consume.
"""

import os
import sys

import numpy as np
from plyfile import PlyData


def scale_ply(src, dst, k):
    ply = PlyData.read(src)
    vertex = ply["vertex"].data
    # uniform scaling leaves normals unchanged, so only x/y/z are touched
    for axis in ("x", "y", "z"):
        vertex[axis] *= k
    ply.write(dst)


def read_pcd_header(f):
    header = []
    while True:
        line = f.readline()
        if not line:
            raise ValueError("unexpected EOF in PCD header")
        header.append(line)
        if line.startswith(b"DATA"):
            return header


def scale_pcd(src, dst, k):
    with open(src, "rb") as f:
        header = read_pcd_header(f)
        body = f.read()

    fields = {}
    for line in header:
        parts = line.decode().split()
        if parts:
            fields[parts[0]] = parts[1:]

    names = fields["FIELDS"]
    data_mode = fields["DATA"][0]
    xyz = [names.index(axis) for axis in ("x", "y", "z")]

    if data_mode == "ascii":
        points = np.loadtxt(body.decode().splitlines(), ndmin=2)
        points[:, xyz] *= k
        out_body = "".join(" ".join(f"{v:.5f}" for v in row) + "\n" for row in points).encode()
    elif data_mode == "binary":
        dtype = np.dtype([(n, f"{'<'}{t.lower()}{s}") for n, t, s in zip(names, fields["TYPE"], fields["SIZE"])])
        points = np.frombuffer(body, dtype=dtype).copy()
        for axis in ("x", "y", "z"):
            points[axis] *= k
        out_body = points.tobytes()
    else:
        raise ValueError(f"unsupported PCD DATA mode '{data_mode}' in {src}")

    with open(dst, "wb") as f:
        f.writelines(header)
        f.write(out_body)


def main():
    if len(sys.argv) not in (7, 8):
        print(f"usage: {sys.argv[0]} <base.ply> <base.pcd> <out_dir> <name_prefix> <h_min> <h_max> [step]", file=sys.stderr)
        return 1

    ply_path, pcd_path, out_dir, prefix = sys.argv[1:5]
    h_min = float(sys.argv[5])
    h_max = float(sys.argv[6])
    step = float(sys.argv[7]) if len(sys.argv) == 8 else 5.0
    if h_min > h_max:
        print(f"h_min {h_min} > h_max {h_max}", file=sys.stderr)
        return 1

    vertex = PlyData.read(ply_path)["vertex"].data
    h_base = float(np.max(vertex["z"]))
    if h_base <= 0.0:
        print(f"base model max z {h_base:.3f} <= 0, expected ground at z=0", file=sys.stderr)
        return 1

    os.makedirs(out_dir, exist_ok=True)
    print(f"h_base {h_base:.3f}")

    # a 0 m model is degenerate, so h_min = 0 starts at the first step instead
    heights = np.arange(h_min if h_min > 0.0 else step, h_max + 1e-6, step)
    for h in heights:
        k = h / h_base
        name = f"{prefix}-h{int(round(h))}"
        scale_ply(ply_path, os.path.join(out_dir, name + ".ply"), k)
        scale_pcd(pcd_path, os.path.join(out_dir, name + ".pcd"), k)
        print(f"{int(round(h))} {k:.6f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
