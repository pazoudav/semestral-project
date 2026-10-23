#!/usr/bin/env python3
"""Uniformly sample points across the surface of a .ply triangle mesh.

Triangles are picked with probability proportional to their area, then a
point is drawn uniformly inside each picked triangle (barycentric sampling),
so the resulting points are uniformly distributed over the whole surface.
Polygonal faces are fan-triangulated. Every point gets a unit normal: the
normal of its face (direction follows face winding), or with --smooth-normals
the face's vertex normals (nx ny nz from the .ply) interpolated at the point's
barycentric coordinates. With --offset D, each point is pushed D meters along
its normal, producing an "envelope" point cloud around the surface (smooth
with --smooth-normals, faceted otherwise; a negative D moves points the other
way). The written normal always points from the surface towards the point, so
it is flipped for a negative D (a viewpoint at the point looks along -normal).
With an offset, points closer than
CLEARANCE_RATIO * |D| to any part of the mesh (e.g. pushed into a
neighbouring beam or a concave corner) are rejected too; the distance to the
mesh is approximated by the distance to the nearest point of a dense
reference cloud sampled on the (un-offset) surface, via a KD-tree. Rejected points
(including those below z=0) are resampled, so exactly N points are sampled.

Optional post-processing, --max-density K: visits the points in random order
and, for each one still present, counts the points inside the 1 m^3 cube
centered on it; if there are more than K, random points inside that cube are
removed until K remain. After this pass every remaining point's cube holds at
most K points, so the written file has fewer than N points.

Usage: sample_surface.py <path/to/mesh.ply> <N> [--offset D] [--smooth-normals]
                         [--max-density K] [--seed S] [-o out.ply]
Writes a vertex-only ascii .ply point cloud (x y z nx ny nz) to -o if given, otherwise
next to the input as <mesh>_sampled_<N>[_offset_<D>][_smooth][_density_<K>].ply.

Needs numpy, plyfile and scipy.
"""

# scripts/worldbounds/.venv/bin/python scripts/sample_surface.py worlds/pylon-30/pylon-30.ply 50000 --offset 2.5 --smooth-normals --max-density 5 -o pylon_sampled.ply
# scripts/worldbounds/.venv/bin/python scripts/sample_surface.py worlds/turbine-40-15/turbine-40-15.ply 25000 --offset 2.5 --smooth-normals --max-density 5 -o turbine_sampled.ply

import argparse
import os
import sys

import numpy as np
from plyfile import PlyData, PlyElement
from scipy.spatial import cKDTree

# offset points must stay at least this fraction of |offset| away from the mesh
CLEARANCE_RATIO = 0.8

# mean spacing of the clearance reference cloud, as a fraction of the clearance
# distance (the nearest-reference distance overestimates the true mesh distance
# by at most about this spacing)
REFERENCE_SPACING_RATIO = 0.02

# cap on the reference cloud size, to bound memory/time on huge meshes
MAX_REFERENCE_POINTS = 5_000_000


def load_triangles(ply_path):
    plydata = PlyData.read(ply_path)

    if "face" not in plydata:
        raise ValueError(f"{ply_path} has no 'face' element, cannot sample a surface")

    vertex = plydata["vertex"]
    vertices = np.column_stack([vertex["x"], vertex["y"], vertex["z"]]).astype(np.float64)

    vertex_normals = None
    if all(k in vertex.data.dtype.names for k in ("nx", "ny", "nz")):
        vertex_normals = np.column_stack([vertex["nx"], vertex["ny"], vertex["nz"]]).astype(np.float64)

    face = plydata["face"]
    index_prop = "vertex_indices" if "vertex_indices" in face.data.dtype.names else "vertex_index"

    triangles = []
    for poly in face[index_prop]:
        poly = np.asarray(poly, dtype=np.int64)
        if len(poly) < 3:
            continue
        # fan triangulation: (v0, vi, vi+1)
        fan = np.column_stack([np.full(len(poly) - 2, poly[0]), poly[1:-1], poly[2:]])
        triangles.append(fan)

    if not triangles:
        raise ValueError(f"{ply_path} has no faces with 3+ vertices")

    return vertices, vertex_normals, np.concatenate(triangles)


def sample_on_triangles(a, b, c, probs, m, rng):
    """Draw m points uniformly over the triangles (a, b, c), weighted by probs (area)."""
    picked = rng.choice(len(a), size=m, p=probs)

    # uniform point in a triangle: reflect (u, v) samples falling outside it
    u = rng.random(m)
    v = rng.random(m)
    outside = u + v > 1.0
    u[outside] = 1.0 - u[outside]
    v[outside] = 1.0 - v[outside]

    pa, pb, pc = a[picked], b[picked], c[picked]
    points = pa + u[:, None] * (pb - pa) + v[:, None] * (pc - pa)
    return picked, u, v, points


def sample_surface(vertices, triangles, n, rng, offset=0.0, vertex_normals=None):
    """Returns (points, normals). If vertex_normals is given, each point's normal
    (and so its offset direction) is interpolated from them instead of using the
    flat face normal. Normals point from the surface towards the (offset) point."""
    a = vertices[triangles[:, 0]]
    b = vertices[triangles[:, 1]]
    c = vertices[triangles[:, 2]]

    cross = np.cross(b - a, c - a)
    double_areas = np.linalg.norm(cross, axis=1)
    total = double_areas.sum()
    if total <= 0.0:
        raise ValueError("mesh has zero surface area")

    probs = double_areas / total

    tree = None
    clearance = CLEARANCE_RATIO * abs(offset)
    if offset != 0.0:
        spacing = REFERENCE_SPACING_RATIO * clearance
        n_ref = int(min(max(np.ceil(total / 2.0 / spacing**2), n), MAX_REFERENCE_POINTS))
        _, _, _, reference = sample_on_triangles(a, b, c, probs, n_ref, rng)
        tree = cKDTree(reference)

    # points ending up below z=0 (under the ground) or too close to the mesh are
    # rejected; keep drawing batches until n points survive
    kept = []
    kept_normals = []
    n_kept = 0
    n_drawn = 0
    n_below = 0
    n_close = 0
    while n_kept < n:
        m = n - n_kept
        picked, u, v, points = sample_on_triangles(a, b, c, probs, m, rng)

        normals = cross[picked] / double_areas[picked, None]
        if vertex_normals is not None:
            # barycentric weights of the point are (1 - u - v, u, v)
            tri = triangles[picked]
            smooth = (
                (1.0 - u - v)[:, None] * vertex_normals[tri[:, 0]]
                + u[:, None] * vertex_normals[tri[:, 1]]
                + v[:, None] * vertex_normals[tri[:, 2]]
            )
            length = np.linalg.norm(smooth, axis=1)
            # vertex normals can cancel out (e.g. on sharp edges); fall back to the face normal there
            ok = length > 1e-6
            normals[ok] = smooth[ok] / length[ok, None]
        points += offset * normals
        if offset < 0.0:
            normals = -normals

        above = points[:, 2] >= 0.0
        n_below += np.count_nonzero(~above)
        points = points[above]
        normals = normals[above]

        if tree is not None:
            dist, _ = tree.query(points, k=1, distance_upper_bound=clearance, workers=-1)
            close = dist < clearance
            n_close += np.count_nonzero(close)
            points = points[~close]
            normals = normals[~close]

        kept.append(points)
        kept_normals.append(normals)
        n_kept += len(points)
        n_drawn += m

        if n_drawn >= 100 * n and n_kept < n:
            raise ValueError(f"only {n_kept}/{n_drawn} sampled points passed the z=0/clearance checks, giving up")

    print(f"surface area: {total / 2.0:.3f} m^2, {len(triangles)} triangles", file=sys.stderr)
    print(f"rejected {n_below} points below z=0", file=sys.stderr)
    if tree is not None:
        print(
            f"rejected {n_close} points closer than {clearance:g} m to the mesh "
            f"({tree.n} reference points)",
            file=sys.stderr,
        )
    return np.concatenate(kept), np.concatenate(kept_normals)


def thin_to_density(points, max_density, rng):
    """Remove random points until no 1 m^3 cube centered on a remaining point
    holds more than max_density points. Returns the boolean mask of kept points."""
    half = 0.5  # half the edge of a 1 m^3 cube

    # hash grid of 1 m cells: a 1 m cube centered anywhere overlaps at most the
    # 3x3x3 block of cells around the center's cell
    cells = np.floor(points).astype(np.int64)
    grid = {}
    for i, cell in enumerate(map(tuple, cells)):
        grid.setdefault(cell, []).append(i)
    offsets = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)]

    alive = np.ones(len(points), dtype=bool)
    for i in rng.permutation(len(points)):
        if not alive[i]:
            continue

        cx, cy, cz = cells[i]
        candidates = []
        for dx, dy, dz in offsets:
            candidates.extend(grid.get((cx + dx, cy + dy, cz + dz), ()))
        candidates = np.asarray(candidates, dtype=np.int64)
        candidates = candidates[alive[candidates]]
        inside = np.all(np.abs(points[candidates] - points[i]) <= half, axis=1)
        inside = candidates[inside]

        excess = len(inside) - int(max_density)
        if excess > 0:
            alive[rng.choice(inside, size=excess, replace=False)] = False

    print(f"density thinning removed {np.count_nonzero(~alive)} points", file=sys.stderr)
    return alive


def write_ply(path, points, normals):
    cloud = np.empty(
        len(points), dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("nx", "f4"), ("ny", "f4"), ("nz", "f4")]
    )
    cloud["x"], cloud["y"], cloud["z"] = points.T
    cloud["nx"], cloud["ny"], cloud["nz"] = normals.T
    PlyData([PlyElement.describe(cloud, "vertex")], text=True).write(path)


def main():
    parser = argparse.ArgumentParser(description="Uniformly sample N points across a .ply mesh surface.")
    parser.add_argument("ply_path", help="input .ply mesh (must contain faces)")
    parser.add_argument("n", type=int, help="number of points to sample")
    parser.add_argument("--offset", type=float, default=0.0, help="move points this many meters along the face normal")
    parser.add_argument(
        "--smooth-normals",
        action="store_true",
        help="use vertex normals interpolated across the face (for the output normals and the offset) instead of the flat face normal",
    )
    parser.add_argument(
        "--max-density",
        type=float,
        default=None,
        help="post-process: thin points until every 1 m^3 cube centered on a point holds at most this many",
    )
    parser.add_argument("--seed", type=int, default=None, help="RNG seed for reproducible sampling")
    parser.add_argument(
        "-o", "--output", help="output .ply path (default: <mesh>_sampled_<N>[...].ply next to the input)"
    )
    args = parser.parse_args()

    if args.n <= 0:
        print("N must be positive", file=sys.stderr)
        return 1
    if args.max_density is not None and args.max_density < 1:
        print("max density must be at least 1", file=sys.stderr)
        return 1

    suffix = f"_offset_{args.offset:g}" if args.offset != 0.0 else ""
    if args.smooth_normals:
        suffix += "_smooth"
    if args.max_density is not None:
        suffix += f"_density_{args.max_density:g}"
    output = args.output or f"{os.path.splitext(args.ply_path)[0]}_sampled_{args.n}{suffix}.ply"

    rng = np.random.default_rng(args.seed)

    try:
        vertices, vertex_normals, triangles = load_triangles(args.ply_path)
        if args.smooth_normals and vertex_normals is None:
            raise ValueError(f"{args.ply_path} has no vertex normals (nx ny nz), cannot use --smooth-normals")
        points, normals = sample_surface(
            vertices,
            triangles,
            args.n,
            rng,
            args.offset,
            vertex_normals if args.smooth_normals else None,
        )
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    if args.max_density is not None:
        kept = thin_to_density(points, args.max_density, rng)
        points, normals = points[kept], normals[kept]

    write_ply(output, points, normals)
    print(f"wrote {len(points)} points to {output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
