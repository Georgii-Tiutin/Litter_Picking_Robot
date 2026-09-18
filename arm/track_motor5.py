#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Live motor-5 tracker: roll the wrist to match the cuboid's orientation.

Runs the cuboid pose detector every frame (shown on the robot's monitor via
cv2.imshow) and continuously commands motor 5 so the jaws stay perpendicular
to the cube's long axis (i.e. lined up to grab the 3 cm width). Only joint 5
is commanded, so the arm stays at the observation pose and just rolls the
wrist -- and rolling motor 5 does not move the (arm-fixed) camera, so the
detection is unaffected.

Mapping:  R = M5_HOME + SIGN*(theta - THETA_HOME),  clamped to [0,180].
Anchored on the known point theta=90 <-> R=90. SIGN is +/-1 (a wrist degree
~= a cube degree); if the wrist tracks BACKWARDS, run with M5_SIGN=-1.

Throttling: motor 5 is re-commanded only when the target moves more than
DEADBAND degrees and at most every MIN_INTERVAL seconds (no servo jitter).
Angle smoothed over the last few frames.

Keys: q/ESC quit. Env: M5_SIGN(+1), M5_HOME(90), THETA_HOME(90),
DEADBAND(3), MIN_INTERVAL(0.4), MOVE_TIME(400), DRY(0), plus the detector
envs (CUBOID_MODEL, CUBOID_CONF, SAT_MIN, ROI_PAD, SHOW).
"""
import os
import time
from collections import deque

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from arm_msgs.msg import ArmJoint

MODEL_PATH = os.path.expanduser(os.environ.get("CUBOID_MODEL", "~/models/cuboid_v1/best.pt"))
OUT_DIR = os.path.expanduser(os.environ.get("CUBOID_OUT", "~/cube_tracker/pose_out"))
CONF = float(os.environ.get("CUBOID_CONF", "0.4"))
SAT_MIN = int(os.environ.get("SAT_MIN", "70"))
ROI_PAD = int(os.environ.get("ROI_PAD", "12"))
CAM_TOPIC = os.environ.get("CUBOID_TOPIC", "/camera/color/image_raw")
SHOW = os.environ.get("SHOW", "1") == "1"

M5_SIGN = float(os.environ.get("M5_SIGN", "1"))       # flip to -1 if it tracks backwards
M5_HOME = float(os.environ.get("M5_HOME", "90"))
THETA_HOME = float(os.environ.get("THETA_HOME", "90"))
DEADBAND = float(os.environ.get("DEADBAND", "3"))     # deg; don't re-command for smaller moves
MIN_INTERVAL = float(os.environ.get("MIN_INTERVAL", "0.4"))  # s between motor commands
MOVE_TIME = int(os.environ.get("MOVE_TIME", "400"))   # servo move time (ms)
DRY = os.environ.get("DRY", "0") == "1"               # 1 = detect+display, don't move motor
WIN = "motor5 tracker"
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
        rx1, ry1, rx2, ry2, conf, yolo_ok = 0, 0, W, H, 0.0, False
    else:
        b = max(res.boxes, key=lambda bb: float(bb.conf))
        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
        conf, yolo_ok = float(b.conf), True
        rx1, ry1 = max(0, x1 - ROI_PAD), max(0, y1 - ROI_PAD)
        rx2, ry2 = min(W, x2 + ROI_PAD), min(H, y2 + ROI_PAD)
    roi = frame[ry1:ry2, rx1:rx2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = (hsv[:, :, 1] >= SAT_MIN).astype(np.uint8) * 255
    if cv2.countNonZero(mask) < 200:
        V = hsv[:, :, 2].astype(np.int16); med = int(np.median(V))
        mask = (np.abs(V - med) > 35).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    cv2.rectangle(ann, (rx1, ry1), (rx2, ry2), (0, 200, 200), 1)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= 200]
    if not cnts:
        return ann, None
    cnt = max(cnts, key=cv2.contourArea) + np.array([[rx1, ry1]])
    rect = cv2.minAreaRect(cnt)
    (cx, cy), (rw, rh), _ = rect
    pts = cv2.boxPoints(rect)
    ang = long_axis_angle(pts)
    cv2.drawContours(ann, [pts.astype(int)], 0, (0, 255, 0), 2)
    cv2.drawMarker(ann, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    return ann, dict(ang=ang, conf=conf, yolo=yolo_ok, cx=cx, cy=cy)


def target_R(theta):
    R = M5_HOME + M5_SIGN * (theta - THETA_HOME)
    # cube orientation is mod 180; fold R into the motor's [0,180] range
    while R < 0:
        R += 180
    while R > 180:
        R -= 180
    return R


class Tracker(Node):
    def __init__(self):
        super().__init__("track_motor5")
        self.br = CvBridge()
        self.get_logger().info("loading %s ..." % MODEL_PATH)
        self.model = YOLO(MODEL_PATH)
        self.pub_img = self.create_publisher(Image, "/track_motor5/image", 1)
        self.pub_joint = self.create_publisher(ArmJoint, "/arm_joint", 10)
        self.create_subscription(Image, CAM_TOPIC, self.cb, 1)
        self.hist = deque(maxlen=5)
        self.sent_R = None
        self.t_last = 0.0
        self.n = 0
        if SHOW:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, 960, 720)
        self.get_logger().info("READY tracker SIGN=%+d HOME=%.0f DRY=%s" % (M5_SIGN, M5_HOME, DRY))

    def cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        ann, r = analyze(frame, self.model)
        H, W = ann.shape[:2]
        line2 = ""
        if r is not None:
            # smooth theta with circular-aware median (mod 180)
            self.hist.append(r["ang"])
            th = float(np.median(list(self.hist)))
            R = target_R(th)
            now = time.time()
            do = (self.sent_R is None or abs(R - self.sent_R) > DEADBAND) and (now - self.t_last) > MIN_INTERVAL
            if do and not DRY:
                m = ArmJoint(); m.id = 5; m.joint = int(round(R)); m.time = MOVE_TIME
                self.pub_joint.publish(m)
                self.sent_R = R; self.t_last = now
            elif do and DRY:
                self.sent_R = R; self.t_last = now
            head = f"ang={th:.1f}  ->  motor5 R={R:.0f}  (sent={int(self.sent_R) if self.sent_R is not None else '-'})"
            line2 = f"SIGN={int(M5_SIGN):+d} conf={r['conf']:.2f}{'  DRY' if DRY else ''}"
            cv2.rectangle(ann, (0, 0), (W, 48), (0, 0, 0), -1)
            cv2.putText(ann, head, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(ann, line2, (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        else:
            cv2.rectangle(ann, (0, 0), (W, 26), (0, 0, 0), -1)
            cv2.putText(ann, "no cube", (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        self.pub_img.publish(self.br.cv2_to_imgmsg(ann, "bgr8"))
        self.n += 1
        if self.n % 5 == 0:
            cv2.imwrite(os.path.join(OUT_DIR, "track.jpg"), ann)
        if r and self.n % 15 == 0:
            self.get_logger().info(line2 + f"  ang={np.median(list(self.hist)):.1f} R={self.sent_R}")
        if SHOW:
            cv2.imshow(WIN, ann)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                rclpy.shutdown()


def main():
    rclpy.init()
    node = Tracker()
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
