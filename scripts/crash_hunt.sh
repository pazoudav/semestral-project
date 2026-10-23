#!/bin/bash
# Restarts the simulation until the rare mapping crash shows up.
#
# Each run starts tmux/start.sh in the background and is watched by motion_watchdog.py:
#   - drone stopped moving (exploration finished or stuck), no takeoff, or 2 h elapsed (--max-run)
#                                                                          -> kill the session, start a new run
#   - pause_on_crash.py caught a crash (crash marker) or sim time froze    -> stop, leave the session up for inspection
#
# usage: scripts/crash_hunt.sh [-n max_runs] <world_name> [motion_watchdog.py args...]
#   e.g. scripts/crash_hunt.sh -n 50 example --window 60 --min-motion 0.3
#
# Ctrl+C stops the hunt and leaves the current session running.
# Attach to the running session with scripts/attach_sim.sh (not a nested tmux -L mrs a, see there).
#
# NOTE: every run kills all gazebo/ros/px4 processes on this machine (scripts/kill_sim.sh), not only the ones of this
# session. Run scripts/kill_sim.sh yourself to close a session the hunt left up.

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
REPO_DIR=$(dirname "$SCRIPT_DIR")
LOG_DIR="$SCRIPT_DIR/crash_hunt_logs"
export CRASH_MARKER="$LOG_DIR/crash_marker"

MAX_RUNS=0
while getopts "n:" opt; do
  case "$opt" in
    n) MAX_RUNS="$OPTARG" ;;
    *) exit 1 ;;
  esac
done
shift $((OPTIND - 1))

WORLD_NAME="$1"
shift
if [ -z "$WORLD_NAME" ]; then
  echo "usage: $0 [-n max_runs] <world_name> [motion_watchdog.py args...]" >&2
  exit 1
fi
if [ ! -f "$REPO_DIR/worlds/$WORLD_NAME/world_info.yaml" ]; then
  echo "world '$WORLD_NAME' has no worlds/$WORLD_NAME/world_info.yaml" >&2
  exit 1
fi

mkdir -p "$LOG_DIR"
RUNS_LOG="$LOG_DIR/runs.log"

STOP=false
trap 'STOP=true' INT

echo "$(date '+%F %T') hunt started, world $WORLD_NAME" >> "$RUNS_LOG"

RUN=0
while true; do
  RUN=$((RUN + 1))
  RUN_LOG=$(printf "%s/run_%03d.log" "$LOG_DIR" "$RUN")
  echo "=== run $RUN, log $RUN_LOG"

  "$SCRIPT_DIR/kill_sim.sh" || echo "continuing anyway" >&2
  rm -f "$CRASH_MARKER"

  RUN_START=$(date +%s)
  NO_ATTACH=1 bash "$REPO_DIR/tmux/start.sh" "$WORLD_NAME" > "$RUN_LOG" 2>&1

  python3 "$SCRIPT_DIR/motion_watchdog.py" "$@" 2>&1 | tee -a "$RUN_LOG"
  CODE=${PIPESTATUS[0]}
  DURATION=$(($(date +%s) - RUN_START))

  if [ "$STOP" == true ] || [ "$CODE" == 130 ]; then
    echo "$(date '+%F %T') run $RUN: interrupted after ${DURATION}s" >> "$RUNS_LOG"
    echo "interrupted, session left running (attach: $SCRIPT_DIR/attach_sim.sh)"
    exit 130
  fi

  case "$CODE" in
    0) REASON="stopped moving" ;;
    2) REASON="CRASH: $(cat "$CRASH_MARKER" 2> /dev/null)" ;;
    3) REASON="SIM TIME FROZEN without crash marker" ;;
    4) REASON="no takeoff" ;;
    5) REASON="max run time reached" ;;
    *) REASON="watchdog failed with code $CODE" ;;
  esac
  echo "$(date '+%F %T') run $RUN: $REASON (${DURATION}s)" >> "$RUNS_LOG"
  echo "=== run $RUN: $REASON (${DURATION}s)"

  if [ "$CODE" == 2 ] || [ "$CODE" == 3 ]; then
    command -v notify-send > /dev/null && notify-send -u critical "crash_hunt" "run $RUN: $REASON"
    printf '\a'
    echo "session left running for inspection, attach: $SCRIPT_DIR/attach_sim.sh"
    exit 0
  fi
  if [ "$CODE" != 0 ] && [ "$CODE" != 4 ] && [ "$CODE" != 5 ]; then
    echo "watchdog failed, stopping (see $RUN_LOG)" >&2
    exit 1
  fi
  if [ "$MAX_RUNS" -gt 0 ] && [ "$RUN" -ge "$MAX_RUNS" ]; then
    echo "reached $MAX_RUNS runs without a crash"
    "$SCRIPT_DIR/kill_sim.sh"
    exit 0
  fi
done
