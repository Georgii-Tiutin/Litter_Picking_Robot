#!/usr/bin/env python3
"""Cube detector v1 — stripped-down sibling of hsv_tuner.py with the
tuning GUI removed. Loads HSV thresholds from
~/project0/perception/config/hsv_blue.yaml (falling back to baked-in
defaults if the file is missing) and visualises the resulting blue-cube
detection in four OpenCV windows:
  - "live"     : raw camera frame
  - "mask"     : raw HSV-blue binary mask (no specular recovery)
  - "mask_aug" : augmented mask after specular recovery + hull fill
  - "result"   : live frame with kept contours, orientation arrows, stats

Specular recovery: diffuse-light glare on a matte blue surface produces
desaturated/value-clipped pixels (looks white) that the blue HSV mask
will never catch. We grab high-V/low-S "specular candidates" and union
in only those touching the dilated blue mask, then fill the convex hull
of each surviving contour to capture interior glare patches.

Run on the robot (needs $DISPLAY):
  source /opt/ros/humble/setup.bash
  source ~/yahboomcar_ws/install/setup.bash
  export ROS_DOMAIN_ID=30
  python3 ~/project0/perception/scripts/cube_detector_v1.py
"""

import math
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

DEFAULTS = {
    "h_low_1": 100,  "h_high_1": 130,
    "h_low_2": 100,  "h_high_2": 130,
    "s_min": 80,     "s_max": 255,
    "v_min": 60,     "v_max": 255,
    "morph_kernel": 5,
    "min_area": 500,
    "aspect_min_x10": 14,
    "aspect_max_x10": 35,
}

SPEC_V_MIN = 220
SPEC_S_MAX = 60
NEAR_BLUE_DILATE_PX = 21
FILL_CONVEX_HULL = True


class CubeDetectorV1(Node):
    def __init__(self):
        super().__init__("cube_detector_v1")
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
            return {**DEFAULTS, **(yaml.safe_load(CFG_PATH.read_text()) or {})}
        except Exception:
            return DEFAULTS.copy()
    return DEFAULTS.copy()


def main():
    v = load_or_default()
    v["morph_kernel"] = max(1, int(v["morph_kernel"]) | 1)

    rclpy.init()
    node = CubeDetectorV1()
    print(f"[cube_detector_v1] loaded hsv config: {v}")
    print(f"[cube_detector_v1] source: {CFG_PATH if CFG_PATH.exists() else 'defaults'}")
    print("[cube_detector_v1] click any cv2 window and press 'q' OR Ctrl-C to quit.")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.frame is None:
            continue

        hsv = cv2.cvtColor(node.frame, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, (v["h_low_1"], v["s_min"], v["v_min"]),
                              (v["h_high_1"], v["s_max"], v["v_max"]))
        m2 = cv2.inRange(hsv, (v["h_low_2"], v["s_min"], v["v_min"]),
                              (v["h_high_2"], v["s_max"], v["v_max"]))
        mask = cv2.bitwise_or(m1, m2)
        k = np.ones((v["morph_kernel"], v["morph_kernel"]), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        # Specular recovery: high-V / low-S pixels (glare on matte plastic
        # reads as near-white) that are spatially adjacent to confident
        # blue pixels get pulled into the mask.
        spec = cv2.inRange(hsv, (0, 0, SPEC_V_MIN), (179, SPEC_S_MAX, 255))
        dk = np.ones((NEAR_BLUE_DILATE_PX, NEAR_BLUE_DILATE_PX), np.uint8)
        near_blue = cv2.dilate(mask, dk)
        spec_near_blue = cv2.bitwise_and(spec, near_blue)
        mask_aug = cv2.bitwise_or(mask, spec_near_blue)
        mask_aug = cv2.morphologyEx(mask_aug, cv2.MORPH_CLOSE, k)

        if FILL_CONVEX_HULL:
            hull_contours, _ = cv2.findContours(
                mask_aug, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            hull_fill = np.zeros_like(mask_aug)
            for c in hull_contours:
                if cv2.contourArea(c) < v["min_area"]:
                    continue
                cv2.fillConvexPoly(hull_fill, cv2.convexHull(c), 255)
            mask_aug = cv2.bitwise_or(mask_aug, hull_fill)

        contours, _ = cv2.findContours(mask_aug, cv2.RETR_EXTERNAL,
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
        cv2.imshow("mask_aug", mask_aug)
        cv2.imshow("result", result)

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
