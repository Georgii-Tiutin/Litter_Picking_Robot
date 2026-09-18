#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fixed-observation-pose drive-up.

Holds the arm at the grasp observation pose [90,45,45,0,90,0] the whole time
(so "the arm is at the grasp observation position" throughout) and drives the
BASE to bring the cube to the grasp framing:
  - rotate (angular.z) to center the cube horizontally (u -> TARGET_U)
  - drive forward (linear.x) to bring it to the target distance (v -> TARGET_V)
Continuous proportional visual servo (published every frame, so the base's
accel ramp is handled naturally -- no timed pulses). Robust saturation-based
detection (shadow-immune) with YOLO ROI + full-frame fallback.

Stops (vx=wz=0, ARRIVED) when the cube is within tolerance of the target, on
loss of detection, on odom distance cap, or on timeout. Shows the live view on
the robot monitor. Then you snap to grasp / descend.

Env: TARGET_U(320) TARGET_V(300) TOL_U(22) TOL_V(22) ROT_SIGN(1)
     KX(0.10) KZ(0.6) MAX_VX(0.07) MAX_WZ(0.25) MAX_DIST(0.40) MAX_TIME(25)
     plus detector envs. Keys: q/ESC quit (stops base).
"""
import os, time, math, threading
from collections import deque
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from ultralytics import YOLO
from arm_msgs.msg import ArmJoints

MODEL_PATH = os.path.expanduser(os.environ.get("CUBOID_MODEL", "~/models/cuboid_v1/best.pt"))
OUT_DIR = os.path.expanduser(os.environ.get("CUBOID_OUT", "~/cube_tracker/pose_out"))
CONF = float(os.environ.get("CUBOID_CONF", "0.4"))
SAT_MIN = int(os.environ.get("SAT_MIN", "70"))
ROI_PAD = int(os.environ.get("ROI_PAD", "12"))
CAM_TOPIC = os.environ.get("CUBOID_TOPIC", "/camera/color/image_raw")
SHOW = os.environ.get("SHOW", "1") == "1"

OBS = [90, 45, 45, 0, 90, 0]
TARGET_U = float(os.environ.get("TARGET_U", "320"))
TARGET_V = float(os.environ.get("TARGET_V", "300"))
TOL_U = float(os.environ.get("TOL_U", "22"))
TOL_V = float(os.environ.get("TOL_V", "22"))
ROT_SIGN = float(os.environ.get("ROT_SIGN", "1"))   # flip if it turns the wrong way
KX = float(os.environ.get("KX", "0.10"))            # m/s per normalized v-error
KZ = float(os.environ.get("KZ", "0.6"))             # rad/s per normalized u-error
MAX_VX = float(os.environ.get("MAX_VX", "0.07"))
MAX_WZ = float(os.environ.get("MAX_WZ", "0.25"))
MAX_DIST = float(os.environ.get("MAX_DIST", "0.40"))  # m odom safety cap
MAX_TIME = float(os.environ.get("MAX_TIME", "25"))    # s safety cap
WIN = "drive-up (fixed obs pose)"
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
    ann = frame.copy()
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
        Vc = hsv[:, :, 2].astype(np.int16); med = int(np.median(Vc))
        mask = (np.abs(Vc - med) > 35).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= 200]
    cv2.drawMarker(ann, (int(TARGET_U), int(TARGET_V)), (255, 0, 0), cv2.MARKER_CROSS, 26, 2)
    if not cnts:
        return ann, None
    cnt = max(cnts, key=cv2.contourArea) + np.array([[rx1, ry1]])
    rect = cv2.minAreaRect(cnt)
    (cx, cy), _, _ = rect
    pts = cv2.boxPoints(rect)
    ang = long_axis_angle(pts)
    cv2.drawContours(ann, [pts.astype(int)], 0, (0, 255, 0), 2)
    cv2.drawMarker(ann, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    return ann, dict(u=cx, v=cy, ang=ang, conf=conf)


class DriveUp(Node):
    def __init__(self):
        super().__init__("drive_up")
        self.br = CvBridge()
        self.get_logger().info("loading %s ..." % MODEL_PATH)
        self.model = YOLO(MODEL_PATH)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_joints = self.create_publisher(ArmJoints, "/arm6_joints", 10)
        self.create_subscription(Image, CAM_TOPIC, self.cb, 1)
        self.create_subscription(Odometry, "/odom_raw", self.odcb, 10)
        self.pub_img = self.create_publisher(Image, "/drive_up/image", 1)
        self.x0 = None; self.od = (0.0, 0.0)
        self.t0 = time.time()
        self.state = "APPROACH"
        self.hist = deque(maxlen=5)
        time.sleep(1.0)
        self.set_obs()
        if SHOW:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL); cv2.resizeWindow(WIN, 960, 720)
        self.get_logger().info("READY drive-up  target=(%.0f,%.0f) ROT_SIGN=%+d" % (TARGET_U, TARGET_V, ROT_SIGN))

    def set_obs(self):
        m = ArmJoints()
        m.joint1, m.joint2, m.joint3, m.joint4, m.joint5, m.joint6 = OBS
        m.time = 1500
        for _ in range(3):
            self.pub_joints.publish(m); time.sleep(0.15)

    def odcb(self, m):
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        if self.x0 is None:
            self.x0 = (x, y)
        self.od = (x - self.x0[0], y - self.x0[1])

    def stop(self):
        z = Twist()
        for _ in range(3):
            self.pub_cmd.publish(z)

    def cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        ann, r = analyze(frame, self.model)
        H, W = ann.shape[:2]
        dist = math.hypot(*self.od)
        vx = wz = 0.0
        note = ""
        if self.state == "APPROACH":
            if dist > MAX_DIST:
                self.state = "STOP"; note = "dist cap"
            elif (time.time() - self.t0) > MAX_TIME:
                self.state = "STOP"; note = "time cap"
            elif r is None:
                vx = wz = 0.0; note = "no cube -> stop base"
            else:
                eu = r["u"] - TARGET_U
                ev = r["v"] - TARGET_V
                u_ok = abs(eu) <= TOL_U
                v_ok = abs(ev) <= TOL_V
                if u_ok and v_ok:
                    self.state = "ARRIVED"; note = "arrived"
                else:
                    if not u_ok:
                        wz = max(-MAX_WZ, min(MAX_WZ, -ROT_SIGN * KZ * (eu / (W / 2.0))))
                    if not v_ok and ev < 0:   # cube above target (far) -> forward
                        vx = max(0.0, min(MAX_VX, -KX * (ev / (H / 2.0))))
                    # cube below target (too close): don't back up, just stop fwd
                    note = "eu=%+.0f ev=%+.0f" % (eu, ev)
        if self.state in ("STOP", "ARRIVED"):
            vx = wz = 0.0
        self.pub_cmd.publish(self._twist(vx, wz))

        # overlay
        col = (0, 255, 0) if self.state == "APPROACH" else ((0, 165, 255) if self.state == "ARRIVED" else (0, 0, 255))
        cv2.rectangle(ann, (0, 0), (W, 46), (0, 0, 0), -1)
        head = "[%s] %s  vx=%.2f wz=%.2f dist=%.2f" % (self.state, note, vx, wz, dist)
        cv2.putText(ann, head, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        if r:
            cv2.putText(ann, "cube u=%.0f v=%.0f ang=%.0f conf=%.2f" % (r["u"], r["v"], r["ang"], r["conf"]),
                        (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        self.pub_img.publish(self.br.cv2_to_imgmsg(ann, "bgr8"))
        cv2.imwrite(os.path.join(OUT_DIR, "driveup.jpg"), ann)
        if SHOW:
            cv2.imshow(WIN, ann)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                self.stop(); rclpy.shutdown()
        if self.state == "ARRIVED":
            self.get_logger().info("ARRIVED cube u=%.0f v=%.0f ang=%.0f dist=%.2f" %
                                   (r["u"], r["v"], r["ang"], dist))

    def _twist(self, vx, wz):
        t = Twist(); t.linear.x = float(vx); t.angular.z = float(wz); return t


def main():
    rclpy.init()
    node = DriveUp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if SHOW:
            cv2.destroyAllWindows()
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()
