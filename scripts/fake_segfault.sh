#!/bin/bash
# Sends SIGSEGV to a mapping nodelet to test the crash pipeline (pause_on_crash.py -> crash marker -> crash_hunt.sh).
# Only the real "nodelet" process is signalled, the gdb / bash wrappers carry the same pattern in their command line.
#
# usage: scripts/fake_segfault.sh [pattern]    (default: mrs_octomap_server/MrsOctomapServer)
#   e.g. scripts/fake_segfault.sh __name:=uav1_mapping_nodelet_manager    # shared manager, without debug:=true

PATTERN="${1:-mrs_octomap_server/MrsOctomapServer}"

SENT=0
for p in $(pgrep -f "$PATTERN"); do
  [ "$(cat /proc/$p/comm 2> /dev/null)" = nodelet ] || continue
  if kill -SEGV "$p"; then
    echo "sent SIGSEGV to pid $p ($(grep State /proc/$p/status 2> /dev/null | cut -f2))"
    SENT=$((SENT + 1))
  fi
done

if [ "$SENT" -eq 0 ]; then
  echo "no running nodelet matches '$PATTERN'" >&2
  exit 1
fi
