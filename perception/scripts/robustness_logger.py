#!/usr/bin/env python3
"""Phase 1.3 robustness logger.

Subscribes to /cube/detections and /camera/color/image_raw. Over a
fixed window (default 30 s), counts:
  - color frames received
  - detections received per track (apriltag / hsv / none)
  - position jitter on consecutive same-track detections
  - mean detection score

Prints a summary at the end so we have measurable numbers per scenario.

Usage:
  python3 robustness_logger.py [--window 30] [--label "scenario_name"]

Run alongside cube_detector.py. To compare scenarios, run with --label
each time. The script appends to ~/project0/perception/logs/robustness.csv
so you can diff scenarios afterwards.
"""

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray

LOG_DIR = Path("/home/jetson/project0/perception/logs")
LOG_CSV = LOG_DIR / "robustness.csv"


class Logger(Node):
    def __init__(self, window_s, label):
        super().__init__("robustness_logger")
        self.window_s = window_s
        self.label = label
        self.start_t = time.monotonic()
        self.color_frames = 0
        self.det_frames = 0
        self.track_counts = {"apriltag_3": 0, "hsv_red": 0}
        self.scores = []
        self.last_pos_per_track = {}
        self.jitter_per_track = {"apriltag_3": [], "hsv_red": []}
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self.cb_color, qos_profile_sensor_data)
        self.create_subscription(Detection3DArray, "/cube/detections",
                                 self.cb_det, 10)

    def cb_color(self, _msg):
        self.color_frames += 1

    def cb_det(self, msg):
        self.det_frames += 1
        if not msg.detections:
            return
        d = msg.detections[0]
        if not d.results:
            return
        h = d.results[0]
        cls = h.hypothesis.class_id
        self.track_counts[cls] = self.track_counts.get(cls, 0) + 1
        self.scores.append(h.hypothesis.score)
        p = h.pose.pose.position
        cur = (p.x, p.y, p.z)
        prev = self.last_pos_per_track.get(cls)
        if prev is not None:
            dx = cur[0] - prev[0]
            dy = cur[1] - prev[1]
            dz = cur[2] - prev[2]
            self.jitter_per_track[cls].append(
                math.sqrt(dx * dx + dy * dy + dz * dz)
            )
        self.last_pos_per_track[cls] = cur

    def remaining(self):
        return self.window_s - (time.monotonic() - self.start_t)

    def summary(self):
        elapsed = time.monotonic() - self.start_t
        total_track_dets = sum(self.track_counts.values())
        none_count = max(0, self.det_frames - total_track_dets)

        def pct(n, d):
            return 100.0 * n / max(1, d)

        def mm_stats(arr):
            if not arr:
                return "—"
            mn = 1000.0 * min(arr)
            mx = 1000.0 * max(arr)
            mean = 1000.0 * sum(arr) / len(arr)
            return f"mean={mean:.1f} max={mx:.1f} min={mn:.1f} mm (n={len(arr)})"

        lines = [
            f"=== robustness summary  label='{self.label}'  window={elapsed:.1f}s ===",
            f"color frames: {self.color_frames}  ({self.color_frames/elapsed:.1f} Hz)",
            f"det messages: {self.det_frames}  ({self.det_frames/elapsed:.1f} Hz)",
            f"  apriltag_3: {self.track_counts.get('apriltag_3', 0):4d}  "
            f"({pct(self.track_counts.get('apriltag_3', 0), self.det_frames):.1f}%)",
            f"  hsv_red:    {self.track_counts.get('hsv_red', 0):4d}  "
            f"({pct(self.track_counts.get('hsv_red', 0), self.det_frames):.1f}%)",
            f"  none:       {none_count:4d}  "
            f"({pct(none_count, self.det_frames):.1f}%)",
            f"score mean: "
            f"{(sum(self.scores)/len(self.scores)):.3f}" if self.scores
            else "score mean: —",
            f"frame-to-frame position jitter (apriltag): "
            f"{mm_stats(self.jitter_per_track['apriltag_3'])}",
            f"frame-to-frame position jitter (hsv):      "
            f"{mm_stats(self.jitter_per_track['hsv_red'])}",
        ]
        print("\n".join(lines))

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        new_file = not LOG_CSV.exists()
        with open(LOG_CSV, "a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow([
                    "timestamp", "label", "window_s",
                    "color_frames", "det_frames",
                    "apriltag_count", "hsv_count", "none_count",
                    "apriltag_pct", "hsv_pct", "none_pct",
                    "score_mean",
                    "jitter_apriltag_mean_mm", "jitter_apriltag_max_mm",
                    "jitter_hsv_mean_mm", "jitter_hsv_max_mm",
                ])
            ja = self.jitter_per_track["apriltag_3"]
            jh = self.jitter_per_track["hsv_red"]
            w.writerow([
                int(time.time()), self.label, f"{elapsed:.1f}",
                self.color_frames, self.det_frames,
                self.track_counts.get("apriltag_3", 0),
                self.track_counts.get("hsv_red", 0),
                none_count,
                f"{pct(self.track_counts.get('apriltag_3', 0), self.det_frames):.1f}",
                f"{pct(self.track_counts.get('hsv_red', 0), self.det_frames):.1f}",
                f"{pct(none_count, self.det_frames):.1f}",
                f"{(sum(self.scores)/len(self.scores)):.3f}" if self.scores else "",
                f"{1000*sum(ja)/len(ja):.1f}" if ja else "",
                f"{1000*max(ja):.1f}" if ja else "",
                f"{1000*sum(jh)/len(jh):.1f}" if jh else "",
                f"{1000*max(jh):.1f}" if jh else "",
            ])
        print(f"\nappended row to {LOG_CSV}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window", type=float, default=30.0,
                    help="logging window in seconds (default 30)")
    ap.add_argument("--label", type=str, default="unlabeled",
                    help="scenario label (e.g. 'plain', 'clutter')")
    args = ap.parse_args()

    rclpy.init()
    node = Logger(args.window, args.label)
    print(f"[robustness] logging '{args.label}' for {args.window:.0f} s …")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.remaining() <= 0:
                break
    except KeyboardInterrupt:
        pass
    node.summary()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
