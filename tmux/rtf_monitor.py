#!/usr/bin/env python3
"""Prints the simulation's Real-Time Factor (RTF) by comparing /clock (sim time) to wall time."""

import time

import rospy
from rosgraph_msgs.msg import Clock


class RTFMonitor:
    def __init__(self, report_period):
        self.report_period = report_period

        self.start_wall = None
        self.start_sim = None
        self.last_report_wall = None
        self.last_report_sim = None

        self.sub = rospy.Subscriber("/clock", Clock, self.callback, queue_size=10)

    def callback(self, msg):
        wall_now = time.monotonic()
        sim_now = msg.clock.to_sec()

        if self.start_wall is None or sim_now < self.last_report_sim:
            # first message, or sim time jumped backward (e.g. simulation restarted)
            rospy.loginfo("RTF monitor: (re)starting reference clock at sim t=%.3fs", sim_now)
            self.start_wall = wall_now
            self.start_sim = sim_now
            self.last_report_wall = wall_now
            self.last_report_sim = sim_now
            return

        if wall_now - self.last_report_wall >= self.report_period:
            instant_rtf = (sim_now - self.last_report_sim) / (wall_now - self.last_report_wall)
            average_rtf = (sim_now - self.start_sim) / (wall_now - self.start_wall)

            rospy.loginfo(
                "sim t=%.1fs | instant RTF=%.3f | average RTF=%.3f",
                sim_now, instant_rtf, average_rtf,
            )

            self.last_report_wall = wall_now
            self.last_report_sim = sim_now


def main():
    rospy.init_node("rtf_monitor", anonymous=True)
    report_period = rospy.get_param("~report_period", 2.0)
    RTFMonitor(report_period)
    rospy.spin()


if __name__ == "__main__":
    main()
