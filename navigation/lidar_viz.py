#!/usr/bin/env python3
"""
lidar_viz.py — Merge the ROSMASTER M3PRO's two LiDARs and render a top-down PNG.

Subscribes to /scan0 (laser0_frame, back-left) and /scan1 (laser1_frame, front-right),
reprojects both into base_link using the URDF static mounting offsets (translation only,
rpy=0 — exactly what ira_laser_tools' laserscan_multi_merger does), and renders a top-down
view with each lidar color-coded plus the combined 360deg picture.

No motion is commanded. Read-only sensor visualization.

Usage (on the robot, after sourcing ROS + workspaces):
    python3 lidar_viz.py --out /tmp/lidar.png --seconds 2.0 --max-range 4.0
"""
import argparse
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Static base_link -> laserN_frame offsets from M3Pro.urdf (metres; rpy = 0,0,0).
LIDAR_OFFSETS = {
    "/scan0": (-0.11617, 0.09156),   # laser0_frame, back-left
    "/scan1": (0.10766, -0.09078),   # laser1_frame, front-right
}
COLORS = {"/scan0": "#e4572e", "/scan1": "#2e86e4"}  # scan0 red, scan1 blue


def scan_to_baselink_xy(msg, off_x, off_y):
    """Convert a LaserScan to (x, y) points in base_link (forward=+x, left=+y)."""
    ranges = np.asarray(msg.ranges, dtype=np.float64)
    n = ranges.size
    angles = msg.angle_min + np.arange(n) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    r = ranges[valid]
    a = angles[valid]
    x = r * np.cos(a) + off_x   # rpy=0 -> pure translation into base_link
    y = r * np.sin(a) + off_y
    return x, y


class LidarCollector(Node):
    def __init__(self, topics):
        super().__init__("lidar_viz")
        self.latest = {t: None for t in topics}
        self.counts = {t: 0 for t in topics}
        for t in topics:
            self.create_subscription(LaserScan, t, self._make_cb(t), 10)

    def _make_cb(self, topic):
        def cb(msg):
            self.latest[topic] = msg
            self.counts[topic] += 1
        return cb


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", default="/tmp/lidar_viz.png", help="output PNG path")
    ap.add_argument("--seconds", type=float, default=2.0, help="collect this long before rendering")
    ap.add_argument("--max-range", type=float, default=4.0, help="plot extent in metres (per side)")
    ap.add_argument("--topics", nargs="+", default=["/scan0", "/scan1"])
    args = ap.parse_args()

    rclpy.init()
    node = LidarCollector(args.topics)

    # Spin for the requested window collecting scans (latest of each is kept).
    end = node.get_clock().now().nanoseconds + int(args.seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    for t in args.topics:
        print(f"{t}: received {node.counts[t]} scans")

    # Render (headless).
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    fig, ax = plt.subplots(figsize=(8, 8))
    R = args.max_range

    merged_pts = 0
    for t in args.topics:
        msg = node.latest[t]
        if msg is None:
            print(f"WARNING: no data on {t}")
            continue
        off_x, off_y = LIDAR_OFFSETS.get(t, (0.0, 0.0))
        x, y = scan_to_baselink_xy(msg, off_x, off_y)
        merged_pts += x.size
        # Plot: forward (base_link +x) = up, left (base_link +y) = left.
        ax.scatter(-y, x, s=3, c=COLORS.get(t, "#444"), label=f"{t} ({x.size} pts)", alpha=0.75)

    # Range rings + robot marker at origin.
    for rr in range(1, int(R) + 1):
        ax.add_patch(Circle((0, 0), rr, fill=False, ec="#bbbbbb", lw=0.6, ls="--"))
        ax.text(0, rr, f"{rr} m", color="#888", fontsize=7, ha="center", va="bottom")
    # Robot footprint (approx 0.20 x 0.22 m) and heading arrow (+x forward = up).
    ax.add_patch(Rectangle((-0.11, -0.10), 0.22, 0.20, fill=True, fc="#333", ec="k", alpha=0.5, zorder=5))
    ax.annotate("", xy=(0, 0.35), xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color="k", lw=2), zorder=6)
    ax.text(0.02, 0.36, "fwd", fontsize=8)

    ax.set_xlim(-R, R)
    ax.set_ylim(-R, R)
    ax.set_aspect("equal")
    ax.set_xlabel("left  <-  y (m)  ->  right")
    ax.set_ylabel("back  <-  x (m)  ->  forward")
    ax.set_title(f"Merged LiDAR (base_link) — {merged_pts} points")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, color="#eee", lw=0.5)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"saved {args.out} ({merged_pts} merged points)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
