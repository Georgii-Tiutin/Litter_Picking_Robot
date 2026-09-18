#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Eye-in-hand ARM centering: keep screen-center on the cube center.

Moves ONLY the arm (joint1 = horizontal pan, joint2 = vertical tilt) with a
proportional visual servo so the detected cube stays centered in the camera.
No base motion. Robust saturation detection (shadow-immune). Shows the live
view on the robot monitor with the center cross and the cube center.

Starts from the observation pose and adjusts j1/j2 around it. j3/j4/j5 held.

Env: SIGN1(1) SIGNV(-1) KP1(4) KPV(2.5) DB(0.05) STEP(1.2)
     J1_MIN(30) J1_MAX(150) J2_MIN(30) J2_MAX(120) J3(45) J5(90)
     plus detector envs. Keys: q/ESC quit.
"""
import os, time
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
CONF = float(os.environ.get("CUBOID_CONF", "0.35"))
SAT_MIN = int(os.environ.get("SAT_MIN", "70"))
ROI_PAD = int(os.environ.get("ROI_PAD", "12"))
CAM_TOPIC = os.environ.get("CUBOID_TOPIC", "/camera/color/image_raw")
SHOW = os.environ.get("SHOW", "1") == "1"

SIGN1 = float(os.environ.get("SIGN1", "1"))     # joint1 pan sign
SIGNV = float(os.environ.get("SIGNV", "-1"))    # joint2 tilt sign
KP1 = float(os.environ.get("KP1", "4"))
KPV = float(os.environ.get("KPV", "2.5"))
DB = float(os.environ.get("DB", "0.05"))        # normalized deadband
STEP = float(os.environ.get("STEP", "1.2"))     # max deg per tick
J1_MIN, J1_MAX = float(os.environ.get("J1_MIN","30")), float(os.environ.get("J1_MAX","150"))
J2_MIN, J2_MAX = float(os.environ.get("J2_MIN","30")), float(os.environ.get("J2_MAX","120"))
J3 = int(os.environ.get("J3", "45"))
J4 = int(os.environ.get("J4", "0"))
J5 = int(os.environ.get("J5", "90"))
CTRL_PERIOD = 0.10
WIN = "arm centering"
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
    cv2.drawMarker(ann, (W // 2, H // 2), (255, 0, 0), cv2.MARKER_CROSS, 26, 2)
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


class CenterArm(Node):
    def __init__(self):
        super().__init__("center_arm")
        self.br = CvBridge()
        self.get_logger().info("loading %s ..." % MODEL_PATH)
        self.model = YOLO(MODEL_PATH)
        self.pub_joint = self.create_publisher(ArmJoint, "/arm_joint", 10)
        self.pub_joints = self.create_publisher(ArmJoints, "/arm6_joints", 10)
        self.create_subscription(Image, CAM_TOPIC, self.cb, 1)
        self.pub_img = self.create_publisher(Image, "/center_arm/image", 1)
        self.j1 = 90.0; self.j2 = 45.0
        self.last = 0.0; self.n = 0
        time.sleep(1.0)
        self._pose_full()   # go to start observation-ish pose
        if SHOW:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL); cv2.resizeWindow(WIN, 960, 720)
        self.get_logger().info("READY center-arm SIGN1=%+d SIGNV=%+d" % (SIGN1, SIGNV))

    def _pose_full(self):
        m = ArmJoints()
        m.joint1, m.joint2, m.joint3, m.joint4, m.joint5, m.joint6 = int(self.j1), int(self.j2), J3, J4, J5, 0
        m.time = 1200
        for _ in range(3):
            self.pub_joints.publish(m); time.sleep(0.15)

    def _send(self, jid, val):
        m = ArmJoint(); m.id = int(jid); m.joint = int(round(val)); m.time = 150
        self.pub_joint.publish(m)

    def cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        ann, r = analyze(frame, self.model)
        H, W = ann.shape[:2]
        now = time.time()
        note = "no cube"
        if r is not None:
            eu = (r["u"] - W / 2.0) / (W / 2.0)
            ev = (r["v"] - H / 2.0) / (H / 2.0)
            if now - self.last >= CTRL_PERIOD:
                if abs(eu) > DB:
                    s = max(-STEP, min(STEP, KP1 * eu))
                    self.j1 = max(J1_MIN, min(J1_MAX, self.j1 + SIGN1 * s))
                    self._send(1, self.j1)
                if abs(ev) > DB:
                    s = max(-STEP, min(STEP, KPV * ev))
                    self.j2 = max(J2_MIN, min(J2_MAX, self.j2 + SIGNV * s))
                    self._send(2, self.j2)
                self.last = now
            centered = abs(eu) < DB and abs(ev) < DB
            note = "%s eu=%+.2f ev=%+.2f j1=%.0f j2=%.0f ang=%.0f" % (
                "CENTERED" if centered else "centering", eu, ev, self.j1, self.j2, r["ang"])
        col = (0, 255, 0)
        cv2.rectangle(ann, (0, 0), (W, 24), (0, 0, 0), -1)
        cv2.putText(ann, note, (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        self.pub_img.publish(self.br.cv2_to_imgmsg(ann, "bgr8"))
        self.n += 1
        if self.n % 5 == 0:
            cv2.imwrite(os.path.join(OUT_DIR, "center.jpg"), ann)
        if SHOW:
            cv2.imshow(WIN, ann)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                rclpy.shutdown()


def main():
    rclpy.init()
    node = CenterArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if SHOW:
            cv2.destroyAllWindows()
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()
