#!/usr/bin/env python3
"""Interactive HSV tuner for the red cuboid.

Subscribes to /camera/color/image_raw and shows three windows:
  - "live"   : the raw frame
  - "mask"   : the binary mask after the current HSV thresholds + morph
  - "result" : the live frame with the contour bbox overlay

Trackbars in the "controls" window let you adjust the two H ranges
(red wraps around 0/180), S min/max, V min/max, kernel size, and the
contour area floor. Press 's' to save the current settings to
~/project0/perception/config/hsv_red.yaml. Press 'q' to quit.

Run on the robot (needs $DISPLAY):
  source /opt/ros/humble/setup.bash
  source ~/yahboomcar_ws/install/setup.bash
  export ROS_DOMAIN_ID=30
  python3 ~/project0/perception/scripts/hsv_tuner_red.py
"""

import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

CFG_PATH = Path("/home/jetson/project0/perception/config/hsv_red.yaml")

# Reasonable starting values for indoor LED + saturated red.
# Red wraps around 0/180 in OpenCV's H scale, so two ranges are needed.
DEFAULTS = {
    "h_low_1": 0,    "h_high_1": 10,
    "h_low_2": 170,  "h_high_2": 179,
    "s_min": 100,    "s_max": 255,
    "v_min": 60,     "v_max": 255,
    "morph_kernel": 5,
    "min_area": 500,
    "aspect_min_x10": 14,  # min aspect ratio * 10 (1.4)
    "aspect_max_x10": 35,  # max aspect ratio * 10 (3.5)
}


def nothing(_):
    pass


class Tuner(Node):
    def __init__(self):
        super().__init__("hsv_tuner")
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(
            Image, "/camera/color/image_raw", self.cb, qos_profile_sensor_data
        )

    def cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def load_or_default():
    if CFG_PATH.exists():
        try:
            return yaml.safe_load(CFG_PATH.read_text()) or DEFAULTS.copy()
        except Exception:
            return DEFAULTS.copy()
    return DEFAULTS.copy()


def save(values):
    CFG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CFG_PATH.write_text(yaml.safe_dump(values, sort_keys=True))
    print(f"[hsv_tuner] saved to {CFG_PATH}")


def main():
    cfg = load_or_default()

    cv2.namedWindow("controls", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("controls", 460, 560)
    for name, ub in [
        ("h_low_1", 179), ("h_high_1", 179),
        ("h_low_2", 179), ("h_high_2", 179),
        ("s_min", 255),   ("s_max", 255),
        ("v_min", 255),   ("v_max", 255),
        ("morph_kernel", 25),
        ("min_area", 5000),
        ("aspect_min_x10", 60),
        ("aspect_max_x10", 60),
    ]:
        cv2.createTrackbar(name, "controls", int(cfg[name]), ub, nothing)

    rclpy.init()
    node = Tuner()
    last_saved = None
    last_save_t = 0.0
    print(f"[hsv_tuner] auto-saves to {CFG_PATH} on every change.")
    print("[hsv_tuner] click any cv2 window and press 'q' OR Ctrl-C in the terminal to quit.")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.frame is None:
            continue

        v = {k: cv2.getTrackbarPos(k, "controls") for k in [
            "h_low_1", "h_high_1", "h_low_2", "h_high_2",
            "s_min", "s_max", "v_min", "v_max",
            "morph_kernel", "min_area",
            "aspect_min_x10", "aspect_max_x10",
        ]}
        v["morph_kernel"] = max(1, v["morph_kernel"] | 1)  # force odd >=1
        # Auto-save (debounced 0.4 s) whenever the values change.
        import time as _t
        nowt = _t.monotonic()
        if v != last_saved and (nowt - last_save_t) > 0.4:
            save(v)
            last_saved = dict(v)
            last_save_t = nowt

        hsv = cv2.cvtColor(node.frame, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, (v["h_low_1"], v["s_min"], v["v_min"]),
                              (v["h_high_1"], v["s_max"], v["v_max"]))
        m2 = cv2.inRange(hsv, (v["h_low_2"], v["s_min"], v["v_min"]),
                              (v["h_high_2"], v["s_max"], v["v_max"]))
        mask = cv2.bitwise_or(m1, m2)
        k = np.ones((v["morph_kernel"], v["morph_kernel"]), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        result = node.frame.copy()
        n_kept = 0
        n_dropped_aspect = 0
        amin = v["aspect_min_x10"] / 10.0
        amax = v["aspect_max_x10"] / 10.0
        for c in contours:
            if cv2.contourArea(c) < v["min_area"]:
                continue
            rect = cv2.minAreaRect(c)
            (cx, cy), (w, h), ang = rect
            asp = max(w, h) / max(1.0, min(w, h))
            if not (amin <= asp <= amax):
                n_dropped_aspect += 1
                # draw rejected in red
                box = cv2.boxPoints(rect).astype(int)
                cv2.drawContours(result, [box], 0, (0, 0, 255), 1)
                cv2.putText(result, f"asp={asp:.2f} REJ",
                            (int(cx) - 50, int(cy) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                continue
            n_kept += 1
            box = cv2.boxPoints(rect).astype(int)
            cv2.drawContours(result, [box], 0, (0, 255, 0), 2)
            cv2.putText(result, f"asp={asp:.2f} ang={ang:.0f}",
                        (int(cx) - 60, int(cy) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 0), 1)

        cv2.putText(result,
                    f"kept={n_kept}  rej_by_aspect={n_dropped_aspect}  "
                    f"asp_range=[{amin:.1f},{amax:.1f}]",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1)
        cv2.imshow("live", node.frame)
        cv2.imshow("mask", mask)
        cv2.imshow("result", result)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("s"):
            save(v)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
