#!/usr/bin/env python3
"""Replaces the over-dense bottom band of a prerecorded turbine viewpoint set with a copy of the band above it.

Viewpoint markers with z in [0, 8) are dropped and replaced by copies of the markers with z in [8, 16),
shifted down by 8 m (x, y and orientation unchanged). Marker ids are renumbered sequentially afterwards.
Works on the `rostopic echo`-style .outmsg text directly, so the rest of the file's formatting is preserved.
Re-running on an already-cleaned file gives the same viewpoints (only ids may be renumbered).

Usage: turbine_viewpoint_cleanup.py [viewpoints.outmsg] [--no-backup]
"""

import argparse
import os
import re
import shutil

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ros_packages", "skeleton_estimator", "data", "turbine", "viewpoints.outmsg")

REPLACE_MIN, REPLACE_MAX = 0.0, 8.0  # band to drop
SOURCE_MIN, SOURCE_MAX = 8.0, 16.0  # band copied down into it
SHIFT = SOURCE_MIN - REPLACE_MIN

# first 8-space-indented `z:` in a marker block is pose.position.z (orientation.z comes after it, scale.z is 6-space)
Z_RE = re.compile(r"(?m)^(        z: )(\S+)$")
ID_RE = re.compile(r"(?m)^    id: \d+$")
BLOCK_SEP_RE = re.compile(r"(?m)^  - \n")


def cleanup(text):
    head, rest = text.split("\n", 1)  # "markers: "
    body, tail = rest.rsplit("\n---", 1)

    blocks = BLOCK_SEP_RE.split(body)
    assert blocks[0] == "", "unexpected content before first marker"
    blocks = blocks[1:]

    keep, copies = [], []
    for block in blocks:
        z = float(Z_RE.search(block).group(2))
        if REPLACE_MIN <= z < REPLACE_MAX:
            continue
        keep.append(block)
        if SOURCE_MIN <= z < SOURCE_MAX:
            copies.append(Z_RE.sub(lambda m: m.group(1) + repr(z - SHIFT), block, count=1))

    out = [ID_RE.sub("    id: %d" % i, block, count=1) for i, block in enumerate(keep + copies)]
    stats = (len(blocks), len(blocks) - len(keep), len(copies), len(out))
    return head + "\n" + "".join("  - \n" + block for block in out) + "\n---" + tail, stats


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("path", nargs="?", default=DEFAULT_PATH, help="viewpoints .outmsg file (default: skeleton_estimator turbine data)")
    parser.add_argument("--no-backup", action="store_true", help="don't copy the original to <path>.bak first")
    args = parser.parse_args()

    path = os.path.normpath(args.path)
    with open(path) as f:
        text = f.read()

    if not args.no_backup:
        shutil.copy2(path, path + ".bak")
        print("backup: %s.bak" % path)

    text, (total, removed, added, final) = cleanup(text)
    with open(path, "w") as f:
        f.write(text)

    print("%s: %d -> %d markers (removed %d in [%g, %g) m, added %d copied from [%g, %g) m)"
          % (path, total, final, removed, REPLACE_MIN, REPLACE_MAX, added, SOURCE_MIN, SOURCE_MAX))


if __name__ == "__main__":
    main()
