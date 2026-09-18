#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Live motor-5 tracker + grab. Copy of track_motor5.py with a grab action.

Tracks motor 5 to the cube's orientation (shown on the robot's monitor), then
on command LOWERS DOWN keeping that same motor-5 roll and grabs the cube:
  descend [90,0,90,0,R,0] (open) -> close j6=145 -> lift [90,45,45,0,R,145].
R is the wrist roll the tracker settled on, so the jaws come down already
aligned to the 3 cm width.

Keys (focus the OpenCV window on the robot screen):
  g  grab now, using the current tracked motor-5 value
  r  release (open) and return to the observation pose, resume tracking
  q / ESC  quit

Mapping/throttle envs same as track_motor5.py (M5_SIGN, M5_HOME, THETA_HOME,
DEADBAND, MIN_INTERVAL, MOVE_TIME). Grab envs: GRIP_CLOSE(145), GRIP_OPEN(0).
"""
import os
import time
import threading
from collections import deque

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from arm_msgs.msg import ArmJoint, ArmJoints

MODEL_PATH = os.path.expanduser(os.environ.get("CUBOID_MODEL", "~/models/cuboid_v1/best.pt"))
OUT_DIR = os.path.expanduser(os.environ.get("CUBOID_OUT", "~/cube_tracker/pose_out"))
CONF = float(os.environ.get("CUBOID_CONF", "0.4"))
SAT_MIN = int(os.environ.get("SAT_MIN", "70"))
ROI_PAD = int(os.environ.get("ROI_PAD", "12"))
CAM_TOPIC = os.environ.get("CUBOID_TOPIC", "/camera/color/image_raw")
SHOW = os.environ.get("SHOW", "1") == "1"

M5_SIGN = float(os.environ.get("M5_SIGN", "1"))
M5_HOME = float(os.environ.get("M5_HOME", "90"))
THETA_HOME = float(os.environ.get("THETA_HOME", "90"))
DEADBAND = float(os.environ.get("DEADBAND", "3"))
MIN_INTERVAL = float(os.environ.get("MIN_INTERVAL", "0.4"))
MOVE_TIME = int(os.environ.get("MOVE_TIME", "400"))

OBS = [90, 45, 45, 0, 90, 0]
DESC_J3 = int(os.environ.get("DESC_J3", "76"))   # elbow at descent (lower = tip drops)
DESC_J4 = int(os.environ.get("DESC_J4", "0"))    # wrist pitch at descent
GRIP_OPEN = int(os.environ.get("GRIP_OPEN", "0"))
GRIP_CLOSE = int(os.environ.get("GRIP_CLOSE", "145"))
WIN = "track + grab"
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
        rx1, ry1, rx2, ry2, conf = 0, 0, W, H, 0.0
    else:
        b = max(res.boxes, key=lambda bb: float(bb.conf))
        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
        conf = float(b.conf)
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
    return ann, dict(ang=ang, conf=conf, cx=cx, cy=cy)


def target_R(theta):
    R = M5_HOME + M5_SIGN * (theta - THETA_HOME)
    while R < 0:
        R += 180
    while R > 180:
        R -= 180
    return R


class TrackGrab(Node):
    def __init__(self):
        super().__init__("track_and_grab")
        self.br = CvBridge()
        self.get_logger().info("loading %s ..." % MODEL_PATH)
        self.model = YOLO(MODEL_PATH)
        self.pub_img = self.create_publisher(Image, "/track_and_grab/image", 1)
        self.pub_joint = self.create_publisher(ArmJoint, "/arm_joint", 10)
        self.pub_joints = self.create_publisher(ArmJoints, "/arm6_joints", 10)
        self.create_subscription(Image, CAM_TOPIC, self.cb, 1)
        self.hist = deque(maxlen=5)
        self.sent_R = 90.0
        self.t_last = 0.0
        self.n = 0
        self.state = "TRACKING"   # TRACKING | GRABBING | HOLDING
        if SHOW:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WIN, 960, 720)
        self.get_logger().info("READY track+grab SIGN=%+d  keys: g=grab r=release q=quit" % M5_SIGN)

    # ---- arm helpers ----
    def _pose(self, j, t=2000, reps=3):
        m = ArmJoints()
        m.joint1, m.joint2, m.joint3, m.joint4, m.joint5, m.joint6 = [int(v) for v in j]
        m.time = int(t)
        for _ in range(reps):
            self.pub_joints.publish(m); time.sleep(0.15)

    def _grip(self, val, t=800, reps=3):
        m = ArmJoint(); m.id = 6; m.joint = int(val); m.time = int(t)
        for _ in range(reps):
            self.pub_joint.publish(m); time.sleep(0.15)

    def _grab_thread(self):
        R = int(round(self.sent_R))
        self.get_logger().info("GRAB: descend keeping motor5=%d (j3=%d)" % (R, DESC_J3))
        self._pose([90, 0, DESC_J3, DESC_J4, R, GRIP_OPEN], 2000); time.sleep(3.0)
        self.get_logger().info("GRAB: close")
        self._grip(GRIP_CLOSE, 800); time.sleep(2.0)
        self.get_logger().info("GRAB: lift (holding)")
        self._pose([90, 45, 45, 0, R, GRIP_CLOSE], 2000); time.sleep(3.0)
        self.state = "HOLDING"
        self.get_logger().info("GRAB: done, holding cube")

    def _release_thread(self):
        self.get_logger().info("RELEASE + reset")
        self._grip(GRIP_OPEN, 800); time.sleep(1.5)
        self._pose(OBS, 2000); time.sleep(3.0)
        self.hist.clear()
        self.state = "TRACKING"

    def start_grab(self):
        if self.state == "TRACKING":
            self.state = "GRABBING"
            threading.Thread(target=self._grab_thread, daemon=True).start()

    def start_release(self):
        if self.state == "HOLDING":
            self.state = "GRABBING"
            threading.Thread(target=self._release_thread, daemon=True).start()

    # ---- perception loop ----
    def cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        ann, r = analyze(frame, self.model)
        H, W = ann.shape[:2]
        if r is not None and self.state == "TRACKING":
            self.hist.append(r["ang"])
            th = float(np.median(list(self.hist)))
            R = target_R(th)
            now = time.time()
            if abs(R - self.sent_R) > DEADBAND and (now - self.t_last) > MIN_INTERVAL:
                m = ArmJoint(); m.id = 5; m.joint = int(round(R)); m.time = MOVE_TIME
                self.pub_joint.publish(m)
                self.sent_R = R; self.t_last = now
            head = f"ang={th:.1f} -> motor5 R={R:.0f} (sent={int(self.sent_R)})"
        elif r is not None:
            head = f"ang={r['ang']:.1f}  motor5 locked={int(self.sent_R)}"
        else:
            head = "no cube"
        col = {"TRACKING": (0, 255, 0), "GRABBING": (0, 165, 255), "HOLDING": (255, 0, 255)}[self.state]
        cv2.rectangle(ann, (0, 0), (W, 48), (0, 0, 0), -1)
        cv2.putText(ann, head, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(ann, f"[{self.state}]  SIGN={int(M5_SIGN):+d}   g=grab  r=release  q=quit",
                    (8, 41), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)

        self.pub_img.publish(self.br.cv2_to_imgmsg(ann, "bgr8"))
        self.n += 1
        if self.n % 5 == 0:
            cv2.imwrite(os.path.join(OUT_DIR, "track.jpg"), ann)
        # remote file triggers (touch these over SSH)
        gt = os.path.join(OUT_DIR, "grab.trigger")
        rt = os.path.join(OUT_DIR, "release.trigger")
        if os.path.exists(gt):
            os.remove(gt); self.start_grab()
        if os.path.exists(rt):
            os.remove(rt); self.start_release()
        if SHOW:
            cv2.imshow(WIN, ann)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                rclpy.shutdown()
            elif key == ord('g'):
                self.start_grab()
            elif key == ord('r'):
                self.start_release()


def main():
    rclpy.init()
    node = TrackGrab()
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
