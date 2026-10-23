#!/usr/bin/env python3
"""Convert a sampled .ply point cloud into a visualization_msgs/MarkerArray.

Output is the YAML text form printed by `rostopic echo` (same layout as
~/user_ros_workspace/skeletons/turbine-init-viewpoints.outmsg), so it can be
replayed with:

    cat <file>.outmsg | rostopic pub /source_viewpoints visualization_msgs/MarkerArray

Every point becomes a SPHERE marker in ns "init_vps_pos" (the namespace
skeleton_estimator reads). The point's unit surface normal (pointing from the
surface towards the viewpoint, so the viewpoint looks along -normal) is
encoded in the sphere's pose.orientation as the rotation taking the +x axis
onto the normal (the same convention RViz uses for ARROW markers);
skeleton_estimator recovers it as orientation * (1, 0, 0). Normals are read
from the .ply's nx ny nz (as written by sample_surface.py); if it has none,
--mesh is required and the normal is taken as the direction from the nearest
point of the mesh surface (approximated by a dense surface sample and a
KD-tree) to the viewpoint.

With --arrows, each sphere is preceded by an ARROW marker in ns "init_vps_dir"
pointing from the viewpoint towards the surface (along -normal), like the
reference file; marker ids then alternate arrow/sphere. Arrows are only for
visualization, skeleton_estimator ignores them.

Usage: ply_to_marker_array.py <points.ply> [--mesh mesh.ply] [--arrows]
                              [--arrow-length L] [--frame-id F] [-o out.outmsg]
Defaults to <points>.outmsg next to the input.

Needs numpy, plyfile and scipy (scipy only with --mesh).
"""

import argparse
import os
import sys
import time

import numpy as np
from plyfile import PlyData

ARROW_SCALE = (0.2, 0.3, 0.2)
ARROW_COLOR = (0.0, 0.4, 0.9, 0.8)
SPHERE_SCALE = (0.5, 0.5, 0.5)
SPHERE_COLOR = (0.0, 0.0, 0.0, 1.0)

# mean spacing of the surface sample used to find each viewpoint's nearest surface point
MESH_SAMPLE_SPACING = 0.05


def load_points(ply_path):
    """Returns (points, normals); normals is None if the .ply has no nx ny nz."""
    vertex = PlyData.read(ply_path)["vertex"]
    points = np.column_stack([vertex["x"], vertex["y"], vertex["z"]]).astype(np.float64)

    normals = None
    if all(k in vertex.data.dtype.names for k in ("nx", "ny", "nz")):
        normals = np.column_stack([vertex["nx"], vertex["ny"], vertex["nz"]]).astype(np.float64)
    return points, normals


def normalized(vectors, fallback):
    """Normalizes each row, replacing (near) zero-length rows with fallback."""
    vectors = vectors.copy()
    length = np.linalg.norm(vectors, axis=1)
    degenerate = length < 1e-9
    vectors[degenerate] = fallback
    length[degenerate] = 1.0
    return vectors / length[:, None]


def quaternion_from_x_axis(direction):
    """Quaternion (x, y, z, w) of the shortest rotation taking (1, 0, 0) onto the unit vector direction."""
    dx, dy, dz = direction
    # half-angle form: w = 1 + x.d, xyz = x cross d = (0, -dz, dy), then normalize
    q = np.array([0.0, -dz, dy, 1.0 + dx])
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        # direction is -x: any 180 deg rotation about an axis perpendicular to x works
        return (0.0, 0.0, 1.0, 0.0)
    return tuple(q / norm)


def nearest_surface_points(points, mesh_path, rng):
    from scipy.spatial import cKDTree

    from sample_surface import load_triangles, sample_on_triangles

    vertices, _, triangles = load_triangles(mesh_path)
    a, b, c = vertices[triangles[:, 0]], vertices[triangles[:, 1]], vertices[triangles[:, 2]]
    double_areas = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    area = double_areas.sum() / 2.0
    n_ref = int(min(max(np.ceil(area / MESH_SAMPLE_SPACING**2), 10 * len(points)), 5_000_000))
    _, _, _, reference = sample_on_triangles(a, b, c, double_areas / double_areas.sum(), n_ref, rng)

    _, idx = cKDTree(reference).query(points, k=1, workers=-1)
    return reference[idx]


def fmt_vec(indent, name, xyz, keys="xyz"):
    pad = " " * indent
    lines = [f"{pad}{name}: "]
    lines += [f"{pad}  {k}: {float(v)!r}" for k, v in zip(keys, xyz)]
    return lines


def fmt_marker(
    stamp, frame_id, ns, marker_id, marker_type, position, scale, color, points=(), orientation=(0.0, 0.0, 0.0, 1.0)
):
    secs, nsecs = stamp
    lines = [
        "  - ",
        "    header: ",
        "      seq: 0",
        "      stamp: ",
        f"        secs: {secs}",
        f"        nsecs: {nsecs}",
        f'      frame_id: "{frame_id}"',
        f'    ns: "{ns}"',
        f"    id: {marker_id}",
        f"    type: {marker_type}",
        "    action: 0",
        "    pose: ",
    ]
    lines += fmt_vec(6, "position", position)
    lines += fmt_vec(6, "orientation", orientation, keys="xyzw")
    lines += fmt_vec(4, "scale", scale)
    lines += fmt_vec(4, "color", color, keys="rgba")
    lines += [
        "    lifetime: ",
        "      secs: 0",
        "      nsecs:         0",
        "    frame_locked: False",
    ]
    if len(points) == 0:
        lines.append("    points: []")
    else:
        lines.append("    points: ")
        for p in points:
            lines.append("      - ")
            lines += [f"        {k}: {float(v)!r}" for k, v in zip("xyz", p)]
    lines += [
        "    colors: []",
        "    text: ''",
        "    mesh_resource: ''",
        "    mesh_use_embedded_materials: False",
    ]
    return lines


def main():
    parser = argparse.ArgumentParser(description="Convert a sampled .ply point cloud into a MarkerArray .outmsg.")
    parser.add_argument("ply_path", help="input .ply point cloud (e.g. from sample_surface.py)")
    parser.add_argument(
        "--mesh", help="object mesh .ply; normals point away from its nearest surface point (only if the input has no normals)"
    )
    parser.add_argument("--arrows", action="store_true", help="add arrow markers pointing from each point towards the surface")
    parser.add_argument("--arrow-length", type=float, default=3.0, help="arrow length in meters (default: 3)")
    parser.add_argument("--frame-id", default="world", help='header frame_id (default: "world")')
    parser.add_argument("-o", "--output", help="output path (default: <points>.outmsg next to the input)")
    args = parser.parse_args()

    output = args.output or f"{os.path.splitext(args.ply_path)[0]}.outmsg"

    points, normals = load_points(args.ply_path)
    if len(points) == 0:
        print(f"error: {args.ply_path} has no points", file=sys.stderr)
        return 1

    if normals is not None:
        if args.mesh:
            print("input has normals, ignoring --mesh", file=sys.stderr)
        # a zero normal has no defined direction; point it straight up
        normals = normalized(normals, (0.0, 0.0, 1.0))
    elif args.mesh:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        targets = nearest_surface_points(points, args.mesh, np.random.default_rng(0))
        # a point lying on the surface has no defined direction; point it straight up
        normals = normalized(points - targets, (0.0, 0.0, 1.0))
    else:
        print(f"error: {args.ply_path} has no normals (nx ny nz), pass --mesh to estimate them", file=sys.stderr)
        return 1

    now = time.time_ns()
    stamp = (now // 1_000_000_000, now % 1_000_000_000)

    lines = ["markers: "]
    marker_id = 0
    for i, p in enumerate(points):
        if args.arrows:
            tip = p - args.arrow_length * normals[i]
            lines += fmt_marker(
                stamp, args.frame_id, "init_vps_dir", marker_id, 0, (0.0, 0.0, 0.0), ARROW_SCALE, ARROW_COLOR, (p, tip)
            )
            marker_id += 1
        lines += fmt_marker(
            stamp,
            args.frame_id,
            "init_vps_pos",
            marker_id,
            2,
            p,
            SPHERE_SCALE,
            SPHERE_COLOR,
            orientation=quaternion_from_x_axis(normals[i]),
        )
        marker_id += 1
    lines.append("---")

    with open(output, "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"wrote {marker_id} markers ({len(points)} points) to {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
