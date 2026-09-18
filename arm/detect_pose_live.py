#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LIVE cuboid orientation detector for the pre-grasp (observation) pose.

Runs the same pipeline as detect_pose.py on every camera frame and shows the
annotated view on the robot's monitor via cv2.imshow (DISPLAY=:0). Also
publishes to /detect_pose/image and writes ~/cube_tracker/pose_out/live.jpg
so it can be verified headless over SSH.

Pipeline per frame:
  YOLO best.pt -> highest-conf box (or full-frame fallback if YOLO misses) ->
  saturation segmentation inside ROI (shadow-immune) -> minAreaRect ->
  center + long-axis angle. Overlays box, center, image-center, angle, offset.

Keys: q or ESC to quit.
Env: CUBOID_MODEL, CUBOID_CONF, SAT_MIN, ROI_PAD, SHOW (1/0).
"""
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL_PATH = os.path.expanduser(os.environ.get("CUBOID_MODEL", "~/models/cuboid_v1/best.pt"))
OUT_DIR = os.path.expanduser(os.environ.get("CUBOID_OUT", "~/cube_tracker/pose_out"))
CONF = float(os.environ.get("CUBOID_CONF", "0.4"))
SAT_MIN = int(os.environ.get("SAT_MIN", "70"))
ROI_PAD = int(os.environ.get("ROI_PAD", "12"))
CAM_TOPIC = os.environ.get("CUBOID_TOPIC", "/camera/color/image_raw")
SHOW = os.environ.get("SHOW", "1") == "1"
WIN = "cuboid pose (pre-grasp)"
os.makedirs(OUT_DIR, exist_ok=True)


def long_axis_angle(box_pts):
    edges = []
    for i in range(4):
        p, q = box_pts[i], box_pts[(i + 1) % 4]
        edges.append((np.hypot(*(q - p)), p, q))
    _, p, q = max(edges, key=lambda e: e[0])
    return np.degrees(np.arctan2(q[1] - p[1], q[0] - p[0])) % 180.0


def analyze(frame, model):
    H, W = frame.shape[:2]
    icx, icy = W / 2.0, H / 2.0
    ann = frame.copy()
    cv2.drawMarker(ann, (int(icx), int(icy)), (255, 0, 0), cv2.MARKER_CROSS, 24, 2)

    res = model.predict(frame, conf=CONF, device=0, verbose=False)[0]
    if len(res.boxes) == 0:
        rx1, ry1, rx2, ry2 = 0, 0, W, H
        conf, yolo_ok = 0.0, False
    else:
        b = max(res.boxes, key=lambda bb: float(bb.conf))
        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
        conf, yolo_ok = float(b.conf), True
        rx1, ry1 = max(0, x1 - ROI_PAD), max(0, y1 - ROI_PAD)
        rx2, ry2 = min(W, x2 + ROI_PAD), min(H, y2 + ROI_PAD)
    roi = frame[ry1:ry2, rx1:rx2]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = (hsv[:, :, 1] >= SAT_MIN).astype(np.uint8) * 255
    method = "sat"
    if cv2.countNonZero(mask) < 200:
        V = hsv[:, :, 2].astype(np.int16); med = int(np.median(V))
        mask = (np.abs(V - med) > 35).astype(np.uint8) * 255
        method = "val"
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    cv2.rectangle(ann, (rx1, ry1), (rx2, ry2), (0, 200, 200), 1)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= 200]
    if not cnts:
        cv2.putText(ann, "no cube", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return ann, None

    cnt = max(cnts, key=cv2.contourArea) + np.array([[rx1, ry1]])
    rect = cv2.minAreaRect(cnt)
    (cx, cy), (rw, rh), _ = rect
    pts = cv2.boxPoints(rect)
    ang = long_axis_angle(pts)
    L, S = max(rw, rh), min(rw, rh)
    ox, oy = cx - icx, cy - icy

    cv2.drawContours(ann, [pts.astype(int)], 0, (0, 255, 0), 2)
    cv2.drawMarker(ann, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.line(ann, (int(icx), int(icy)), (int(cx), int(cy)), (0, 165, 255), 1)
    txt = f"ang={ang:.1f}  off=({ox:+.0f},{oy:+.0f})  L/S={L:.0f}/{S:.0f}  conf={conf:.2f} yolo={int(yolo_ok)} {method}"
    cv2.rectangle(ann, (0, 0), (W, 26), (0, 0, 0), -1)
    cv2.putText(ann, txt, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1, cv2.LINE_AA)
    return ann, dict(cx=cx, cy=cy, ang=ang, off=(ox, oy), conf=conf, yolo=yolo_ok)


class LiveNode(Node):
    def __init__(self):
        super().__init__("detect_pose_live")
        self.br = CvBridge()
        self.get_logger().info(f"loading {MODEL_PATH} ...")
        self.model = YOLO(MODEL_PATH)
        self.pub = self.create_publisher(Image, "/detect_pose/image", 1)
        self.create_subscription(Image, CAM_TOPIC, self.cb, 1)
        self.n = 0
        if SHOW:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, 960, 720)
        self.get_logger().info("READY - live pose detection")

    def cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        ann, r = analyze(frame, self.model)
        self.pub.publish(self.br.cv2_to_imgmsg(ann, "bgr8"))
        self.n += 1
        if self.n % 5 == 0:
            cv2.imwrite(os.path.join(OUT_DIR, "live.jpg"), ann)
        if r and self.n % 15 == 0:
            self.get_logger().info(f"ang={r['ang']:.1f} off={r['off']} conf={r['conf']:.2f}")
        if SHOW:
            cv2.imshow(WIN, ann)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                rclpy.shutdown()


def main():
    rclpy.init()
    node = LiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if SHOW:
            cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
