#!/usr/bin/env python3
"""Drop-in replacement for yahboom_laser_filter that cannot emit a future-dated scan.

WHY
---
On 2026-09-13 AMCL silently stopped localising for hours. Its own logs said:

    Message Filter dropping message: frame 'base_link' ...
    reason 'the timestamp on the message is earlier than all the data in the transform cache'

and /scan was measured at **-0.117 s** - stamped 117 ms in the FUTURE. Every scan was
discarded, so AMCL never updated, so it never broadcast map->odom, so every map coordinate the
robot computed was meaningless. It kept publishing /amcl_pose throughout, which is why it took
so long to spot.

The Orin's RTC has no battery (it reads 1970 at boot), so systemd-timesyncd steps and then
slews the system clock. A driver that derives stamps from a base captured at ITS startup plus
monotonic elapsed time will drift into the future as the system clock is slewed underneath it.
Measured: +20 ms at 2 minutes of uptime, -117 ms after ~2 hours.

This node does exactly what the vendor filter does (blank returns below MIN_RANGE, same as
laser_filter_processor.cpp) and additionally clamps any stamp that is ahead of ROS time.
Losing a few ms of true capture time is nothing next to losing localisation entirely.

It COUNTS and REPORTS every clamp, so the underlying drift stays visible rather than being
silently papered over - if this node is clamping constantly, the clock problem is still there
and wants fixing at the source.
"""
import sys, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

MIN_RANGE = 0.18      # matches the vendor filter's blanking distance
REPORT_EVERY = 100

class ScanRestamp(Node):
    def __init__(self):
        super().__init__("scan_restamp")
        self.declare_parameter("in_topic", "/scan_multi")
        self.declare_parameter("out_topic", "/scan")
        self.declare_parameter("min_range", MIN_RANGE)
        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.min_range = float(self.get_parameter("min_range").value)
        self.pub = self.create_publisher(LaserScan, self.out_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.in_topic, self.cb, qos_profile_sensor_data)
        self.n = 0; self.clamped = 0; self.worst = 0.0
        print("scan_restamp: %s -> %s, blanking below %.2f m, clamping future stamps"
              %(self.in_topic, self.out_topic, self.min_range))

    def cb(self, msg):
        out = msg
        rng = list(out.ranges)
        for i, r in enumerate(rng):
            if r < self.min_range:
                rng[i] = float("inf")
        out.ranges = rng

        now = self.get_clock().now()
        st = rclpy.time.Time.from_msg(msg.header.stamp)
        age = (now.nanoseconds - st.nanoseconds) / 1e9
        if age < 0.0:                       # stamped in the future - this is the whole point
            out.header.stamp = now.to_msg()
            self.clamped += 1
            self.worst = min(self.worst, age)
        self.n += 1
        self.pub.publish(out)
        if self.n % REPORT_EVERY == 0 and self.clamped:
            print("   clamped %d of %d scans; worst was %+.3f s into the future"
                  %(self.clamped, self.n, self.worst))

def main():
    rclpy.init(); n = ScanRestamp()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    print("scan_restamp: %d scans, %d clamped (worst %+.3f s)"%(n.n, n.clamped, n.worst))
    rclpy.shutdown()

if __name__ == "__main__": sys.exit(main())
