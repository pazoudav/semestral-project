#!/usr/bin/env python3
"""Waypoint spacing statistics of the offline FC-Planner paths in skeleton_estimator/data/<world_type>/paths/h*/.

Waypoints are read exactly like skeleton_estimator's PrecomputedPaths::loadPath(): order.txt TEXT markers sorted
by their tour index (text), viewpoints.txt markers matched onto them within VIEWPOINT_MATCH_DIST. For every path
this prints the distribution of consecutive waypoint distances (all segments, and viewpoint-to-viewpoint along
the tour), the tour length vs. index.yaml's path_length, and counts of near-duplicate / long segments.

usage: analyze_path_spacing.py [world_type ...] [--long L] [--dup D] [--csv FILE] [--data-dir DIR]
       (default world types: every directory under data/ that has paths/index.yaml)
"""
import argparse
import csv
import os
import sys

import numpy as np
import yaml

VIEWPOINT_MATCH_DIST = 0.05  # [m], same as precomputed_paths.cpp

DATA_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "..", "ros_packages", "skeleton_estimator", "data"))

Loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)


def load_echo_file(path):
    """One `rostopic echo` YAML document, cut at the first `---` (same as loadEchoFile())."""
    with open(path) as f:
        content = f.read()
    end = content.find("\n---")
    if end != -1:
        content = content[:end]
    node = yaml.load(content, Loader=Loader)
    return node.get("markers") or []


def marker_pos(m):
    p = m["pose"]["position"]
    return (float(p["x"]), float(p["y"]), float(p["z"]))


def load_path(d):
    indexed = []
    for m in load_echo_file(os.path.join(d, "order.txt")):
        try:
            indexed.append((int(str(m["text"])), marker_pos(m)))
        except (KeyError, ValueError, TypeError):
            continue
    indexed.sort(key=lambda e: e[0])  # stable, like std::sort on distinct indices
    pts = np.array([p for _, p in indexed], dtype=float).reshape(-1, 3)

    vps = np.array([marker_pos(m) for m in load_echo_file(os.path.join(d, "viewpoints.txt"))], dtype=float).reshape(-1, 3)
    is_vp = np.zeros(len(pts), dtype=bool)
    if len(vps) and len(pts):
        dist = np.linalg.norm(pts[:, None, :] - vps[None, :, :], axis=2)
        is_vp = (dist < VIEWPOINT_MATCH_DIST).any(axis=1)
    return pts, is_vp, len(vps)


def stats(x):
    if len(x) == 0:
        return dict(n=0, min=np.nan, p10=np.nan, median=np.nan, mean=np.nan, p90=np.nan, max=np.nan)
    return dict(n=len(x), min=x.min(), p10=np.percentile(x, 10), median=np.median(x), mean=x.mean(),
                p90=np.percentile(x, 90), max=x.max())


def fmt_row(cols, widths):
    return "  ".join(str(c).rjust(w) for c, w in zip(cols, widths))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world_types", nargs="*")
    ap.add_argument("--long", type=float, default=5.0, help="segments longer than this count as long [m] (default 5)")
    ap.add_argument("--dup", type=float, default=0.05, help="segments shorter than this count as duplicates [m] (default 0.05)")
    ap.add_argument("--csv", help="also write one row per path to this CSV file")
    ap.add_argument("--data-dir", default=DATA_DIR, help="directory holding <world_type>/paths/ (default: skeleton_estimator/data)")
    args = ap.parse_args()
    data_dir = args.data_dir

    world_types = args.world_types or sorted(
        w for w in os.listdir(data_dir) if os.path.isfile(os.path.join(data_dir, w, "paths", "index.yaml")))
    if not world_types:
        sys.exit("no data/<world_type>/paths/index.yaml found under " + data_dir)

    rows = []
    for wt in world_types:
        paths_dir = os.path.join(data_dir, wt, "paths")
        with open(os.path.join(paths_dir, "index.yaml")) as f:
            index = yaml.load(f, Loader=Loader)
        entries = sorted(index.get("paths") or [], key=lambda e: e["height"])

        print(f"\n=== {wt}  (base model {index.get('base_model')}, h_base {index.get('h_base')} m)")
        print(f"    seg = consecutive waypoints, vp = consecutive viewpoints along the tour; distances in m;"
              f" dup < {args.dup} m, long > {args.long} m;"
              f" vp_rev = waypoints matched to a viewpoint beyond the viewpoints.txt marker count (tour revisits a viewpoint)")
        hdr = ["h", "status", "wps", "vps", "vp_rev", "len", "idx_len",
               "seg_min", "seg_p10", "seg_med", "seg_mean", "seg_p90", "seg_max", "dup", "long",
               "vp_med", "vp_mean", "vp_max", "wp/vp"]
        widths = [4, 8, 5, 5, 6, 8, 8, 7, 7, 7, 8, 7, 7, 4, 4, 6, 7, 6, 5]
        print(fmt_row(hdr, widths))

        for e in entries:
            d = os.path.join(paths_dir, str(e["dir"]))
            status = e.get("status", "")
            if not os.path.isdir(d) or not os.path.isfile(os.path.join(d, "order.txt")):
                print(fmt_row([e["height"], status, "-"] + ["missing"], widths[:4]))
                continue
            pts, is_vp, n_vp_file = load_path(d)
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1) if len(pts) > 1 else np.array([])
            vp_pts = pts[is_vp]
            vp_seg = np.linalg.norm(np.diff(vp_pts, axis=0), axis=1) if len(vp_pts) > 1 else np.array([])
            s, v = stats(seg), stats(vp_seg)
            n_vp = int(is_vp.sum())
            row = dict(world_type=wt, height=e["height"], status=status, waypoints=len(pts), viewpoints=n_vp,
                       viewpoints_revisited=n_vp - n_vp_file, length=seg.sum(),
                       index_path_length=float(e.get("path_length", np.nan)),
                       **{f"seg_{k}": s[k] for k in ("min", "p10", "median", "mean", "p90", "max")},
                       seg_dup=int((seg < args.dup).sum()), seg_long=int((seg > args.long).sum()),
                       **{f"vp_{k}": v[k] for k in ("median", "mean", "max")},
                       wp_per_vp=len(pts) / n_vp if n_vp else np.nan)
            rows.append(row)
            print(fmt_row([e["height"], status, len(pts), n_vp, n_vp - n_vp_file, f"{seg.sum():.1f}",
                           f"{row['index_path_length']:.1f}",
                           *(f"{s[k]:.3f}" if k == "min" else f"{s[k]:.2f}" for k in ("min", "p10", "median", "mean", "p90", "max")),
                           row["seg_dup"], row["seg_long"],
                           *(f"{v[k]:.2f}" for k in ("median", "mean", "max")),
                           f"{row['wp_per_vp']:.2f}"], widths))

        # longest segments of the whole world type, to see where the big jumps are
        print(f"\n    longest segments per path (index i -> i+1, length, dz):")
        for e in entries:
            d = os.path.join(paths_dir, str(e["dir"]))
            if not os.path.isfile(os.path.join(d, "order.txt")):
                continue
            pts, is_vp, _ = load_path(d)
            if len(pts) < 2:
                continue
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            top = np.argsort(seg)[::-1][:3]
            desc = ", ".join(f"{i}->{i + 1} {seg[i]:.2f}m (dz {pts[i + 1, 2] - pts[i, 2]:+.2f}"
                             f"{', ' + ('vp' if is_vp[i] else 'c') + '-' + ('vp' if is_vp[i + 1] else 'c')})"
                             for i in top)
            print(f"    h{e['height']:<4} {desc}")

    if args.csv and rows:
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {args.csv}")


if __name__ == "__main__":
    main()
