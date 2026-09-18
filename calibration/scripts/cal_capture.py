#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture one camera->floor homography correspondence.

Usage: python3 cal_capture.py X Y
  X,Y = floor coordinates (metres) where the cube currently sits, in the
        arm-base reference frame (X forward, Y left, origin = robot base
        front-centre on the floor).

Grabs one live color frame at the FIXED observation pose, runs best.pt,
takes the highest-confidence cuboid box, and uses its bounding-box
bottom-centre pixel as the floor-contact point (u,v). Appends
{X,Y,u,v,conf} to homography_points.json and saves an annotated snapshot
with the picked pixel marked, so it can be verified over SSH.
"""
import sys
import os
import json
import time

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MODEL = "/home/jetson/models/cuboid_v1/best.pt"
OUT = "/home/jetson/project0/calibration/floor_homography"
JSON = os.path.join(OUT, "homography_points.json")
CONF = 0.25


def main():
    if len(sys.argv) < 3:
        print("USAGE: cal_capture.py X Y")
        return
    X = float(sys.argv[1])
    Y = float(sys.argv[2])
    os.makedirs(OUT, exist_ok=True)
    model = YOLO(MODEL)

    rclpy.init()
    node = rclpy.create_node("cal_capture")
    bridge = CvBridge()
    got = {}

    def cb(msg):
        got["frame"] = bridge.imgmsg_to_cv2(msg, "bgr8")

    node.create_subscription(Image, "/camera/color/image_raw", cb, 10)
    t0 = time.time()
    while "frame" not in got and time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "frame" not in got:
        print("NO_FRAME"); node.destroy_node(); rclpy.shutdown(); return

    frame = got["frame"]
    r = model.predict(frame, conf=CONF, device=0, verbose=False)[0]
    if len(r.boxes) == 0:
        print("NO_DETECTION (no cuboid in view)")
        node.destroy_node(); rclpy.shutdown(); return

    best = max(r.boxes, key=lambda b: float(b.conf))
    x1, y1, x2, y2 = [float(v) for v in best.xyxy[0]]
    u = (x1 + x2) / 2.0
    v = y2  # bottom-centre = floor contact of a cube standing on the floor
    conf = float(best.conf)

    data = json.load(open(JSON)) if os.path.exists(JSON) else []
    data.append({"X": X, "Y": Y, "u": u, "v": v, "conf": conf})
    json.dump(data, open(JSON, "w"), indent=2)

    ann = r.plot()
    cv2.circle(ann, (int(u), int(v)), 7, (0, 255, 255), -1)
    cv2.putText(ann, f"#{len(data)} X={X:.3f} Y={Y:.3f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(os.path.join(OUT, f"cal_{len(data):02d}.jpg"), ann)
    cv2.imwrite(os.path.join(OUT, "cal_latest.jpg"), ann)
    print(f"OK point {len(data)}: X={X} Y={Y} u={u:.1f} v={v:.1f} conf={conf:.2f}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
