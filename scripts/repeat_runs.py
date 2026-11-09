#!/usr/bin/env python3
"""Runs the simulation over and over: starts the session, waits for the explorer to report end of exploration, kills it, repeats.

Each run starts tmux/start.sh in the background (NO_ATTACH=1) and is watched by a fresh child process of this script
(--watch, rospy can't re-register with a new roscore), which exits once the explorer publishes true on
/$UAV_NAME/explorer/exploration_finished. The whole simulation is then killed by scripts/kill_sim.sh and the next run starts.
The watcher also follows /$UAV_NAME/skeleton_estimator/precomputed_path and appends to runs.log every time the published
precomputed path becomes stable or loses stability (wall time since run start + sim time; on loss also which stability check
failed, against the thresholds read from skeleton_estimator's precomputed/stability/* params), and when tsp_solver's guiding
viewpoint selection is switched off/on (latched /$UAV_NAME/tsp_solver/guiding_enabled).

usage: scripts/repeat_runs.py [-n max_runs] [--max-run SECONDS] <world_name>
  e.g. scripts/repeat_runs.py -n 10 --max-run 3600 example

Ctrl+C stops the loop and leaves the current session running (attach: scripts/attach_sim.sh, close: scripts/kill_sim.sh).

NOTE: every run kills all gazebo/ros/px4 processes on this machine (scripts/kill_sim.sh), not only the ones of this session.
"""

import argparse
import math
import os
import subprocess
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_DIR = os.path.dirname(SCRIPT_DIR)
LOG_DIR = os.path.join(SCRIPT_DIR, "repeat_runs_logs")

EXIT_FINISHED, EXIT_TIMEOUT = 0, 5


def log(msg):
    print("[%s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def watch(topic, max_run, path_topic, guiding_topic, run, run_start, runs_log_path):
    """Child mode: block until `topic` carries true, or max_run seconds (wall time) pass.

    Meanwhile appends a line to runs_log_path on every stability change of the precomputed path on `path_topic` and on
    every change of tsp_solver's guiding state on `guiding_topic`."""
    import rospy
    from std_msgs.msg import Bool

    # retries until the roscore of the freshly started session is up
    rospy.init_node("repeat_runs_watch", anonymous=True, disable_signals=True)
    log("waiting for true on %s" % topic)

    finished = []
    rospy.Subscriber(topic, Bool, lambda msg: msg.data and finished.append(True), queue_size=1)

    def log_event(event):
        log(event)
        with open(runs_log_path, "a") as f:
            f.write("%s run %d: %s\n" % (time.strftime("%F %T"), run, event))

    guiding = [True]  # tsp_solver starts with guiding enabled

    def on_guiding(msg):
        if msg.data == guiding[0]:
            return
        guiding[0] = msg.data
        log_event("guiding viewpoints turned %s at %s (sim %.1fs)" % ("on" if msg.data else "off", fmt_duration(time.time() - run_start),
                                                                     rospy.get_time()))

    rospy.Subscriber(guiding_topic, Bool, on_guiding, queue_size=10)

    try:
        from skeleton_estimator.msg import PrecomputedPath
    except ImportError:
        log("skeleton_estimator msgs not found (workspace not built/sourced?), not logging path stability")
    else:
        was_stable = [False]
        stability_ns = path_topic.rsplit("/", 1)[0] + "/precomputed/stability/"

        def instability_reason(msg):
            """Which stability checks of skeleton_estimator's checkStability() failed, with the value vs. its threshold."""
            try:
                window = rospy.get_param(stability_ns + "window")
                eps_height = rospy.get_param(stability_ns + "eps_height")
                eps_yaw_deg = rospy.get_param(stability_ns + "eps_yaw_deg")
                eps_trans = rospy.get_param(stability_ns + "eps_trans")
            except (KeyError, rospy.ROSException) as e:
                return "thresholds unavailable (%s): d_height %.2f m, d_yaw %.2f deg, d_trans %.2f m, window %d" % (
                    e, msg.max_delta_height, math.degrees(msg.max_delta_yaw), msg.max_delta_translation, msg.history_size)
            failed = []
            if msg.history_size < window:
                failed.append("window %d/%d" % (msg.history_size, window))
            if msg.max_delta_height >= eps_height:
                failed.append("d_height %.2f >= %.2f m" % (msg.max_delta_height, eps_height))
            if math.degrees(msg.max_delta_yaw) >= eps_yaw_deg:
                failed.append("d_yaw %.2f >= %.2f deg" % (math.degrees(msg.max_delta_yaw), eps_yaw_deg))
            if msg.max_delta_translation >= eps_trans:
                failed.append("d_trans %.2f >= %.2f m" % (msg.max_delta_translation, eps_trans))
            return ", ".join(failed) if failed else "no check failed?"

        def on_path(msg):
            if msg.stable == was_stable[0]:
                return
            was_stable[0] = msg.stable
            elapsed = fmt_duration(time.time() - run_start)
            if msg.stable:
                event = "path became stable at %s (sim %.1fs), height %.2f m -> h%.0f" % (elapsed, msg.header.stamp.to_sec(), msg.height, msg.path_height)
            else:
                event = "path lost stability at %s (sim %.1fs), height %.2f m: %s" % (elapsed, msg.header.stamp.to_sec(), msg.height,
                                                                                     instability_reason(msg))
            log_event(event)

        rospy.Subscriber(path_topic, PrecomputedPath, on_path, queue_size=10)

    start = time.time()
    while not finished:
        if max_run > 0 and time.time() - start > max_run:
            return EXIT_TIMEOUT
        time.sleep(1.0)
    return EXIT_FINISHED


def fmt_duration(seconds):
    m, s = divmod(int(seconds), 60)
    h, m = divmod(m, 60)
    return "%d:%02d:%02d" % (h, m, s)


def kill_sim():
    if subprocess.call([os.path.join(SCRIPT_DIR, "kill_sim.sh")]) != 0:
        log("kill_sim.sh failed, continuing anyway")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("world_name", nargs="?")
    parser.add_argument("-n", "--max-runs", type=int, default=0, help="stop after this many runs (0 = forever)")
    parser.add_argument("--max-run", type=float, default=0, help="kill a run after this many seconds of wall time (0 = never)")
    parser.add_argument("--uav-name", default="uav1")
    parser.add_argument("--watch", metavar="TOPIC", help=argparse.SUPPRESS)  # internal child mode
    parser.add_argument("--run", type=int, default=0, help=argparse.SUPPRESS)  # child mode: run number
    parser.add_argument("--run-start", type=float, default=0, help=argparse.SUPPRESS)  # child mode: run start (epoch)
    args = parser.parse_args()

    if args.watch:
        path_topic = "/%s/skeleton_estimator/precomputed_path" % args.uav_name
        guiding_topic = "/%s/tsp_solver/guiding_enabled" % args.uav_name
        sys.exit(watch(args.watch, args.max_run, path_topic, guiding_topic, args.run, args.run_start, os.path.join(LOG_DIR, "runs.log")))

    if not args.world_name:
        parser.error("world_name is required")
    if not os.path.isfile(os.path.join(REPO_DIR, "worlds", args.world_name, "world_info.yaml")):
        parser.error("world '%s' has no worlds/%s/world_info.yaml" % (args.world_name, args.world_name))

    topic = "/%s/explorer/exploration_finished" % args.uav_name
    os.makedirs(LOG_DIR, exist_ok=True)
    runs_log = open(os.path.join(LOG_DIR, "runs.log"), "a")
    runs_log.write("%s started, world %s\n" % (time.strftime("%F %T"), args.world_name))
    runs_log.flush()

    run = 0
    while args.max_runs <= 0 or run < args.max_runs:
        run += 1
        run_log = os.path.join(LOG_DIR, "run_%03d.log" % run)
        log("=== run %d, log %s" % (run, run_log))

        kill_sim()
        run_start = time.time()
        with open(run_log, "w") as f:
            subprocess.call(["bash", os.path.join(REPO_DIR, "tmux", "start.sh"), args.world_name],
                            env=dict(os.environ, NO_ATTACH="1"), stdout=f, stderr=subprocess.STDOUT)

        watcher = subprocess.Popen([sys.executable, os.path.realpath(__file__), "--watch", topic, "--max-run", str(args.max_run),
                                    "--uav-name", args.uav_name, "--run", str(run), "--run-start", str(run_start)])
        try:
            code = watcher.wait()
        except KeyboardInterrupt:
            watcher.wait()
            duration = time.time() - run_start
            runs_log.write("%s run %d: interrupted after %s\n" % (time.strftime("%F %T"), run, fmt_duration(duration)))
            log("=== run %d: interrupted, lasted %s" % (run, fmt_duration(duration)))
            log("interrupted, session left running (attach: %s/attach_sim.sh)" % SCRIPT_DIR)
            sys.exit(130)

        duration = time.time() - run_start
        reason = {EXIT_FINISHED: "exploration finished", EXIT_TIMEOUT: "max run time reached"}.get(code, "watcher failed with code %d" % code)
        runs_log.write("%s run %d: %s, lasted %s\n" % (time.strftime("%F %T"), run, reason, fmt_duration(duration)))
        runs_log.flush()
        log("=== run %d: %s, lasted %s" % (run, reason, fmt_duration(duration)))

        if code not in (EXIT_FINISHED, EXIT_TIMEOUT):
            log("stopping, session left running")
            sys.exit(1)

    log("done, %d runs" % run)
    kill_sim()


if __name__ == "__main__":
    main()
