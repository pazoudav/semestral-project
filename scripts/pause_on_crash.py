#!/usr/bin/env python3
"""Pauses Gazebo physics when a watched process crashes.

Made for catching the intermittent SIGSEGV of the mapping nodelets. Processes are found by matching their command
line and polled through /proc. The trigger is either
  - the process is gone (or a zombie): the shared-manager setup, where the crash just kills the process, or
  - the process is in a ptrace stop ('t' in /proc/<pid>/stat) for longer than --stop-grace: the debug:=true setup,
    where gdb catches the SIGSEGV and leaves the process alive but stopped at the (gdb) prompt.
Then /gazebo/pause_physics (std_srvs/Empty) is called, --delay seconds later.

A pid can't tell a crash from a clean shutdown, so shutting the whole session down also triggers the call. It is
harmless: the service is either gone or the sim is closing anyway.

Usage:
  ./pause_on_crash.py                    # shared manager: watches uav1_mapping_nodelet_manager
  ./pause_on_crash.py --debug            # mapplan.launch debug:=true: watches the 4 standalone mapping nodelets
  ./pause_on_crash.py --pattern "nodelet manager __name:=my_manager" --pattern "..."   # custom, repeatable
  ./pause_on_crash.py --dry-run          # only report, do not call the service
"""

import argparse
import os
import subprocess
import sys
import time

SERVICE = "/gazebo/pause_physics"

MANAGER_PATTERNS = ["__name:=uav1_mapping_nodelet_manager"]
# the plugin names the standalone nodelets are started with under debug:=true (see mapplan.launch)
DEBUG_PATTERNS = ["mrs_octomap_server/MrsOctomapServer", "mrs_pcl_tools/PCLFiltration", "mrs_octomap_tools/OctomapRvizVisualizer"]
# wrappers that carry the debugged command line as an argument, the real process has comm "nodelet"
WRAPPER_COMMS = {"gdb", "bash", "sh", "timeout", "tmux", "python3", "roslaunch"}


def ancestors():
    """This process and all its parents, so a wrapper carrying the pattern isn't matched."""
    pids, pid = set(), os.getpid()
    while pid > 1:
        pids.add(pid)
        try:
            with open("/proc/%d/stat" % pid) as f:
                pid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return pids


def find_pids(pattern):
    """All pids whose command line contains pattern, without this script, its parents and gdb/bash wrappers."""
    skip = ancestors()
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) in skip:
            continue
        try:
            with open("/proc/%s/cmdline" % entry, "rb") as f:
                cmdline = f.read().replace(b"\0", b" ").decode(errors="replace")
            if pattern not in cmdline:
                continue
            with open("/proc/%s/comm" % entry) as f:
                if f.read().strip() in WRAPPER_COMMS:
                    continue
        except OSError:
            continue
        found.append(int(entry))
    return found


def state(pid):
    """One-letter process state from /proc, or None if the process does not exist."""
    try:
        with open("/proc/%d/stat" % pid) as f:
            # state is the field after the ')' that closes the command name
            return f.read().rsplit(")", 1)[1].split()[0]
    except OSError:
        return None


def init_rospy():
    """Registers a rospy node if the master is up (rospy blocks forever otherwise). True if rospy is usable."""
    try:
        import rosgraph
        import rospy

        if not rosgraph.is_master_online():
            return False
        rospy.init_node("pause_on_crash", anonymous=True, disable_signals=True)
        return True
    except Exception as e:
        print("could not init a rospy node: %s" % e, file=sys.stderr)
        return False


def pause_physics(rospy_ready):
    """Calls /gazebo/pause_physics, first through rospy, then through the rosservice CLI."""
    try:
        import rospy
        from std_srvs.srv import Empty

        if not rospy_ready and not init_rospy():
            raise RuntimeError("no ROS master")
        rospy.wait_for_service(SERVICE, timeout=2.0)
        rospy.ServiceProxy(SERVICE, Empty)()
        return True
    except Exception as e:  # rospy missing, master gone, service missing...
        print("rospy call failed (%s), trying rosservice" % e, file=sys.stderr)
    try:
        subprocess.run(["rosservice", "call", SERVICE], check=True, timeout=5)
        return True
    except Exception as e:
        print("rosservice call failed: %s" % e, file=sys.stderr)
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pattern", action="append", help="substring of a watched command line, repeatable")
    parser.add_argument("--debug", action="store_true", help="watch the standalone nodelets of mapplan.launch debug:=true")
    parser.add_argument("--interval", type=float, default=0.02, help="poll period [s]")
    parser.add_argument("--stop-grace", type=float, default=0.5,
                        help="a ptrace stop must last this long [s] to count, gdb stops processes briefly on its own")
    parser.add_argument("--delay", type=float, default=5.0, help="wait this long [s] after the crash before pausing")
    parser.add_argument("--dry-run", action="store_true", help="do not call the service, only report")
    parser.add_argument("--marker", default=os.environ.get("CRASH_MARKER"),
                        help="file to write the crash cause to (default: $CRASH_MARKER), read by crash_hunt.sh")
    args = parser.parse_args()

    patterns = args.pattern or (DEBUG_PATTERNS if args.debug else MANAGER_PATTERNS)

    # registering the node now keeps the service call fast once the crash happens
    rospy_ready = init_rospy()

    print("waiting for processes matching %s ..." % patterns, flush=True)
    watched = {}  # pid -> pattern
    stopped_since = {}  # pid -> time the ptrace stop was first seen
    last_scan = 0.0
    cause = None

    while cause is None:
        now = time.time()
        # new processes keep appearing while the session starts (and the visualizers come in twos), so rescan often
        if now - last_scan > 0.5:
            last_scan = now
            for pattern in patterns:
                for pid in find_pids(pattern):
                    if pid not in watched:
                        watched[pid] = pattern
                        print("watching pid %d (%s)" % (pid, pattern), flush=True)

        for pid, pattern in watched.items():
            s = state(pid)
            if s is None or s == "Z":
                cause = "pid %d (%s) is gone" % (pid, pattern)
                break
            if s == "t":
                stopped_since.setdefault(pid, now)
                if now - stopped_since[pid] >= args.stop_grace:
                    cause = "pid %d (%s) is stopped by a debugger" % (pid, pattern)
                    break
            else:
                stopped_since.pop(pid, None)
        time.sleep(args.interval)

    print("[%s] %s" % (time.strftime("%H:%M:%S"), cause), flush=True)
    if args.marker:
        try:
            with open(args.marker, "w") as f:
                f.write("[%s] %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), cause))
        except OSError as e:
            print("could not write marker %s: %s" % (args.marker, e), file=sys.stderr)
    time.sleep(args.delay)
    if args.dry_run:
        print("dry run, not pausing", flush=True)
        return 0
    if pause_physics(rospy_ready):
        print("[%s] physics paused" % time.strftime("%H:%M:%S"), flush=True)
        return 0
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
