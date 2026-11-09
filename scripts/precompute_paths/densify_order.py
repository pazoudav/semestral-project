#!/usr/bin/env python3
"""Deduplicate and densify an FC-Planner order.txt so consecutive waypoints are at most <step> m apart.

First, a waypoint closer than <dedup> m to the previous kept one is dropped (FC-Planner sometimes emits the same
viewpoint twice in a row; with the default 0.05 m, the same tolerance skeleton_estimator matches viewpoints with, a
dropped point is indistinguishable from the kept one). Non-consecutive revisits of a position are kept.

order.txt is a `rostopic echo` of /hcoppTraj/drone_status: a MarkerArray of TEXT markers, one per waypoint in tour
order (text = tour index). Every segment longer than <step> is split into ceil(d / step) equal pieces by inserting
points on the straight chord; all original waypoints (viewpoints included) are kept at their exact positions, so
skeleton_estimator's position-based viewpoint matching (viewpoints.txt) is unaffected. Markers are renumbered
(id and text = new tour index); inserted markers copy their segment start's marker otherwise.

The output is again one YAML document terminated by `---`, readable by skeleton_estimator's loader and by
`rostopic pub -f`.

usage: densify_order.py <in_order.txt> <out_order.txt> [step_m=1.0] [dedup_m=0.05]
"""
import copy
import math
import sys

import yaml

Loader = getattr(yaml, "CSafeLoader", yaml.SafeLoader)
Dumper = getattr(yaml, "CSafeDumper", yaml.SafeDumper)


def load_echo(path):
    with open(path) as f:
        content = f.read()
    end = content.find("\n---")
    if end != -1:
        content = content[:end]
    return yaml.load(content, Loader=Loader)


def main():
    if len(sys.argv) not in (3, 4, 5):
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    step = float(sys.argv[3]) if len(sys.argv) >= 4 else 1.0
    dedup = float(sys.argv[4]) if len(sys.argv) >= 5 else 0.05
    if step <= 0.0:
        sys.exit("step must be > 0")

    doc = load_echo(src)
    markers = []
    for m in doc.get("markers") or []:
        try:
            markers.append((int(str(m["text"])), m))
        except (KeyError, ValueError, TypeError):
            continue  # same as the C++ loader: markers without an integer index are ignored
    markers.sort(key=lambda e: e[0])
    markers = [m for _, m in markers]

    def pos(m):
        p = m["pose"]["position"]
        return [float(p["x"]), float(p["y"]), float(p["z"])]

    n_raw = len(markers)
    kept = []
    for m in markers:
        if kept and math.dist(pos(kept[-1]), pos(m)) < dedup:
            continue
        kept.append(m)
    markers = kept

    out = []
    for i, m in enumerate(markers):
        out.append(m)
        if i + 1 == len(markers):
            break
        a, b = pos(m), pos(markers[i + 1])
        d = math.dist(a, b)
        pieces = math.ceil(d / step - 1e-9)
        for k in range(1, pieces):
            f = k / pieces
            q = copy.deepcopy(m)
            q["pose"]["position"] = {"x": a[0] + f * (b[0] - a[0]), "y": a[1] + f * (b[1] - a[1]),
                                     "z": a[2] + f * (b[2] - a[2])}
            out.append(q)

    for i, m in enumerate(out):
        m["id"] = i
        m["text"] = str(i)
    doc["markers"] = out

    with open(dst, "w") as f:
        yaml.dump(doc, f, Dumper=Dumper, sort_keys=False, default_flow_style=False)
        f.write("---\n")
    print(f"{n_raw} -> {len(out)} waypoints ({n_raw - len(markers)} duplicates < {dedup} m removed, step {step} m)")


if __name__ == "__main__":
    main()
