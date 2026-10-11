#!/usr/bin/env python3
"""Watches one simulation run and exits once the run is no longer useful for crash hunting.

Started once per run by crash_hunt.sh (a fresh process per run, rospy can't re-register with a new roscore).
The drone counts as stuck when it moved less than --min-motion over the last --window seconds of *sim* time, so a
Gazebo physics pause (sim time stops) never fills the window and can't be mistaken for a stuck drone.

Exit codes (read by crash_hunt.sh):
  0  drone stopped moving (exploration finished or the drone is stuck)  -> restart
  2  crash marker written by pause_on_crash.py                         -> keep the session
  3  sim time frozen without a marker (physics paused / gazebo hung)    -> keep the session
  4  the drone did not take off within --startup-timeout                -> restart
  5  --max-run elapsed without a crash                                  -> restart
The crash marker is checked before every other condition, so a crashed run is always kept.
"""

import argparse
import collections
import math
import os
import sys
import threading
import time

EXIT_STOPPED, EXIT_CRASH, EXIT_FROZEN, EXIT_STARTUP, EXIT_MAX_RUN = 0, 2, 3, 4, 5


def log(msg):
    print("[%s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


class Watchdog:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.first_pos = None
        self.max_from_first = 0.0  # takeoff detection
        self.positions = collections.deque()  # (sim time, (x, y, z)), sampled every >= --sample seconds
        self.last_odom_wall = None
        self.sim_time = None
        self.sim_time_changed_wall = None

    def callback_clock(self, msg):
        t = msg.clock.to_sec()
        with self.lock:
            if self.sim_time is None or t != self.sim_time:
                self.sim_time = t
                self.sim_time_changed_wall = time.time()

    def callback_odom(self, msg):
        p = msg.pose.pose.position
        pos = (p.x, p.y, p.z)
        with self.lock:
            self.last_odom_wall = time.time()
            if self.first_pos is None:
                self.first_pos = pos
            self.max_from_first = max(self.max_from_first, math.dist(pos, self.first_pos))
            t = self.sim_time if self.sim_time is not None else msg.header.stamp.to_sec()
            if not self.positions or t - self.positions[-1][0] >= self.args.sample:
                self.positions.append((t, pos))
            # keep slightly more than the window so its full length can be checked
            while self.positions and t - self.positions[0][0] > self.args.window + 2 * self.args.sample:
                self.positions.popleft()

    def motion_in_window(self):
        """Largest distance from the latest position within the window, None if the window is not full yet."""
        with self.lock:
            if len(self.positions) < 2:
                return None
            t_last, last = self.positions[-1]
            if t_last - self.positions[0][0] < self.args.window:
                return None
            return max(math.dist(pos, last) for t, pos in self.positions if t_last - t <= self.args.window)


def wait_for_master(deadline):
    import rosgraph

    while time.time() < deadline:
        if rosgraph.is_master_online():
            return True
        time.sleep(1.0)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--marker", default=os.environ.get("CRASH_MARKER"), help="crash marker file (default: $CRASH_MARKER)")
    parser.add_argument("--odom", default="/uav1/estimation_manager/odom_main", help="nav_msgs/Odometry topic")
    parser.add_argument("--window", type=float, default=40.0, help="stuck window [s, sim time]")
    parser.add_argument("--min-motion", type=float, default=0.5, help="moving less than this [m] over the window is stuck")
    parser.add_argument("--sample", type=float, default=0.2, help="position sampling period [s, sim time]")
    parser.add_argument("--takeoff-motion", type=float, default=1.0, help="moving this far [m] counts as taken off")
    parser.add_argument("--startup-timeout", type=float, default=180.0, help="time to take off [s, wall]")
    parser.add_argument("--freeze-timeout", type=float, default=15.0, help="sim time not advancing this long [s, wall] is frozen")
    parser.add_argument("--odom-timeout", type=float, default=15.0, help="no odometry this long [s, wall] while sim time runs counts as stopped")
    parser.add_argument("--max-run", type=float, default=7200.0, help="restart after this long [s, wall] regardless, 0 = never")
    args = parser.parse_args()
    if not args.marker:
        parser.error("no crash marker, pass --marker or set CRASH_MARKER")

    start = time.time()
    startup_deadline = start + args.startup_timeout

    if not wait_for_master(startup_deadline):
        log("ROS master did not come up within %.0f s" % args.startup_timeout)
        return EXIT_STARTUP

    import rospy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    rospy.init_node("motion_watchdog", anonymous=True, disable_signals=True)
    wd = Watchdog(args)
    rospy.Subscriber("/clock", Clock, wd.callback_clock, queue_size=10)
    rospy.Subscriber(args.odom, Odometry, wd.callback_odom, queue_size=10)

    log("waiting for takeoff (%s moving %.1f m)" % (args.odom, args.takeoff_motion))
    taken_off = False
    while True:
        time.sleep(0.5)
        now = time.time()

        # checked first: a crash also freezes sim time and stops the drone
        if os.path.exists(args.marker):
            with open(args.marker) as f:
                log("crash marker found: %s" % f.read().strip())
            return EXIT_CRASH

        with wd.lock:
            sim_changed = wd.sim_time_changed_wall
            last_odom = wd.last_odom_wall
            moved = wd.max_from_first

        if not taken_off:
            if moved >= args.takeoff_motion:
                taken_off = True
                log("took off after %.0f s, watching motion (< %.2f m over %.0f s sim time is stuck)"
                    % (now - start, args.min_motion, args.window))
            elif now > startup_deadline:
                log("no takeoff within %.0f s (odometry %s, moved %.2f m)"
                    % (args.startup_timeout, "received" if last_odom else "never received", moved))
                return EXIT_STARTUP
            continue

        if sim_changed is not None and now - sim_changed > args.freeze_timeout:
            # pause_on_crash.py writes the marker before it pauses, give it one more look
            if os.path.exists(args.marker):
                continue
            log("sim time frozen for %.0f s without a crash marker" % (now - sim_changed))
            return EXIT_FROZEN

        if last_odom is not None and now - last_odom > args.odom_timeout:
            log("no odometry for %.0f s while sim time runs" % (now - last_odom))
            return EXIT_STOPPED

        motion = wd.motion_in_window()
        if motion is not None and motion < args.min_motion:
            log("drone moved only %.2f m over the last %.0f s sim time (finished or stuck), run took %.0f s"
                % (motion, args.window, now - start))
            return EXIT_STOPPED

        if args.max_run > 0 and now - start > args.max_run:
            log("max run time %.0f s reached" % args.max_run)
            return EXIT_MAX_RUN


if __name__ == "__main__":
    try:
        code = main()
    except KeyboardInterrupt:
        code = 130
    # rospy's threads would keep the process alive
    os._exit(code)
