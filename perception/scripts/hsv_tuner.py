#!/usr/bin/env python3
"""Interactive HSV tuner for the red cuboid.

Subscribes to /camera/color/image_raw, /camera/depth/image_raw, and
/camera/ir/image_raw, and shows five windows:
  - "live"   : the raw colour frame
  - "mask"   : the binary mask after the current HSV thresholds + morph
  - "result" : the live frame with the contour bbox overlay
  - "depth"  : depth image, colormapped for visualisation
  - "ir"     : IR image, normalised to 8-bit

Trackbars in the "controls" window let you adjust the two H ranges
(red wraps around 0/180), S min/max, V min/max, kernel size, and the
contour area floor. Press 's' to save the current settings to
~/project0/perception/config/hsv_red.yaml. Press 'q' to quit.

Run on the robot (needs $DISPLAY):
  source /opt/ros/humble/setup.bash
  source ~/yahboomcar_ws/install/setup.bash
  export ROS_DOMAIN_ID=30
  python3 ~/project0/perception/scripts/hsv_tuner.py
"""

import math
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

CFG_PATH = Path("/home/jetson/project0/perception/config/hsv_blue.yaml")

# Reasonable starting values for indoor LED + saturated blue. Blue is a
# single H band (no wraparound like red). Both H ranges are set to the
# same blue band; the OR'd mask is identical to a single inRange.
DEFAULTS = {
    "h_low_1": 100,  "h_high_1": 130,
    "h_low_2": 100,  "h_high_2": 130,
    "s_min": 80,     "s_max": 255,
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
        self.depth = None
        self.ir = None
        self.create_subscription(
            Image, "/camera/color/image_raw", self.cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw", self.cb_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/camera/ir/image_raw", self.cb_ir, qos_profile_sensor_data
        )

    def cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def cb_depth(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            self.depth = None

    def cb_ir(self, msg):
        try:
            self.ir = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            self.ir = None


def depth_to_viz(depth):
    if depth is None:
        return None
    d = depth.astype(np.float32)
    valid = d > 0
    if not np.any(valid):
        return np.zeros(d.shape, dtype=np.uint8)
    dmin = float(d[valid].min())
    dmax = float(d[valid].max())
    if dmax - dmin < 1e-6:
        norm = np.zeros_like(d, dtype=np.uint8)
    else:
        norm = np.clip((d - dmin) / (dmax - dmin) * 255.0, 0, 255).astype(np.uint8)
    norm[~valid] = 0
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)


def ir_to_viz(ir):
    if ir is None:
        return None
    if ir.dtype == np.uint8:
        return ir
    img = ir.astype(np.float32)
    lo, hi = float(img.min()), float(img.max())
    if hi - lo < 1e-6:
        return np.zeros(img.shape, dtype=np.uint8)
    return np.clip((img - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)


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
                cv2.drawContours(result, [c], -1, (0, 0, 255), 1)
                cv2.putText(result, f"asp={asp:.2f} REJ",
                            (int(cx) - 50, int(cy) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                continue
            n_kept += 1
            cv2.drawContours(result, [c], -1, (0, 255, 0), 2)
            # Orientation from image moments: stable because it integrates
            # over every interior pixel, not just the contour boundary.
            M = cv2.moments(c)
            if M["m00"] > 0:
                ccx = M["m10"] / M["m00"]
                ccy = M["m01"] / M["m00"]
                mu20 = M["mu20"] / M["m00"]
                mu02 = M["mu02"] / M["m00"]
                mu11 = M["mu11"] / M["m00"]
                theta = 0.5 * math.atan2(2.0 * mu11, mu20 - mu02)
                common = math.sqrt((mu20 - mu02) ** 2 + 4.0 * mu11 ** 2)
                half_major = 2.0 * math.sqrt(max(0.0, 0.5 * (mu20 + mu02 + common)))
                half_minor = 2.0 * math.sqrt(max(0.0, 0.5 * (mu20 + mu02 - common)))
                dx, dy = half_major * math.cos(theta), half_major * math.sin(theta)
                px, py = half_minor * math.cos(theta + math.pi / 2), half_minor * math.sin(theta + math.pi / 2)
                cv2.arrowedLine(result,
                                (int(ccx - dx), int(ccy - dy)),
                                (int(ccx + dx), int(ccy + dy)),
                                (0, 255, 255), 2, tipLength=0.15)
                cv2.line(result,
                         (int(ccx - px), int(ccy - py)),
                         (int(ccx + px), int(ccy + py)),
                         (0, 200, 200), 1)
                axis_deg = math.degrees(theta)
            else:
                axis_deg = float("nan")
            cv2.putText(result, f"asp={asp:.2f} axis={axis_deg:.0f}d",
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
        depth_viz = depth_to_viz(node.depth)
        if depth_viz is not None:
            cv2.imshow("depth", depth_viz)
        ir_viz = ir_to_viz(node.ir)
        if ir_viz is not None:
            cv2.imshow("ir", ir_viz)

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
