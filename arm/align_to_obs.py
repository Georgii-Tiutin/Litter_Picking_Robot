#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Center the cube with the ARM, then drive the BASE to unwind the arm back to
the observation pose -> end at [90,45,45,0,90,0] with the cube centered.

Inner loop (every frame): arm visual servo keeps the cube centered
  joint1 (pan)  <- horizontal pixel error
  joint2 (tilt) <- vertical pixel error
Outer loop (base): rotate to bring j1 -> 90 (face), drive fwd/back to bring
  j2 -> 45 (approach), so the arm unwinds toward the observation pose while the
  inner loop keeps the cube centered. ARRIVED when |j1-90|<DB_J1 and
  |j2-45|<DB_J2 and cube centered -> base stops; arm is at the observation pose,
  cube centered, ready to grasp.

Min base speeds beat the friction deadband; odom distance cap + timeout for
safety; stops base on loss of detection. Live view on the robot monitor.

Env: SIGN1(1) SIGNV(-1) SIGN_BASE(-1) J1T(90) J2T(45) DB_J1(5) DB_J2(4)
     KROT(0.010) MINWZ(0.12) MAXWZ(0.30) KFWD(0.006) MINVX(0.09) MAXVX(0.13)
     MAX_DIST(0.5) MAX_TIME(40). Keys: q/ESC quit (stops base).
"""
import os, time, math
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
from arm_msgs.msg import ArmJoint, ArmJoints

MODEL_PATH = os.path.expanduser(os.environ.get("CUBOID_MODEL", "~/models/cuboid_v1/best.pt"))
OUT_DIR = os.path.expanduser(os.environ.get("CUBOID_OUT", "~/cube_tracker/pose_out"))
CONF = float(os.environ.get("CUBOID_CONF", "0.35"))
SAT_MIN = int(os.environ.get("SAT_MIN", "70"))
ROI_PAD = int(os.environ.get("ROI_PAD", "12"))
CAM_TOPIC = os.environ.get("CUBOID_TOPIC", "/camera/color/image_raw")
SHOW = os.environ.get("SHOW", "1") == "1"

SIGN1 = float(os.environ.get("SIGN1", "1"))
SIGNV = float(os.environ.get("SIGNV", "-1"))
SIGN_BASE = float(os.environ.get("SIGN_BASE", "-1"))   # verified in cube_arm_tracker
KP1 = 4.0; KPV = 2.5; DB = 0.05; STEP = 1.2
J1_MIN, J1_MAX = 30.0, 150.0
J2_MIN, J2_MAX = 30.0, 120.0
J3 = int(os.environ.get("J3", "45")); J4 = int(os.environ.get("J4", "0")); J5 = int(os.environ.get("J5", "90"))
J1T = float(os.environ.get("J1T", "90")); J2T = float(os.environ.get("J2T", "45"))
DB_J1 = float(os.environ.get("DB_J1", "5")); DB_J2 = float(os.environ.get("DB_J2", "4"))
KROT = float(os.environ.get("KROT", "0.010")); MINWZ = float(os.environ.get("MINWZ", "0.12")); MAXWZ = float(os.environ.get("MAXWZ", "0.30"))
KFWD = float(os.environ.get("KFWD", "0.006")); MINVX = float(os.environ.get("MINVX", "0.09")); MAXVX = float(os.environ.get("MAXVX", "0.13"))
MAX_DIST = float(os.environ.get("MAX_DIST", "0.5")); MAX_TIME = float(os.environ.get("MAX_TIME", "40"))
CTRL_PERIOD = 0.10
WIN = "align to obs pose"
os.makedirs(OUT_DIR, exist_ok=True)


def long_axis_angle(box_pts):
    e = []
    for i in range(4):
        p, q = box_pts[i], box_pts[(i + 1) % 4]
        e.append((np.hypot(*(q - p)), p, q))
    _, p, q = max(e, key=lambda t: t[0])
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


def with_min(val, mn, mx):
    s = 1.0 if val >= 0 else -1.0
    return s * min(mx, max(mn, abs(val)))


class Align(Node):
    def __init__(self):
        super().__init__("align_to_obs")
        self.br = CvBridge()
        self.get_logger().info("loading %s ..." % MODEL_PATH)
        self.model = YOLO(MODEL_PATH)
        self.pub_joint = self.create_publisher(ArmJoint, "/arm_joint", 10)
        self.pub_joints = self.create_publisher(ArmJoints, "/arm6_joints", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_img = self.create_publisher(Image, "/align_to_obs/image", 1)
        self.create_subscription(Image, CAM_TOPIC, self.cb, 1)
        self.create_subscription(Odometry, "/odom_raw", self.odcb, 10)
        self.j1 = 90.0; self.j2 = 45.0
        self.x0 = None; self.od = (0.0, 0.0)
        self.last = 0.0; self.n = 0; self.t0 = time.time()
        self.state = "ALIGN"
        time.sleep(1.0)
        self._pose_full()
        if SHOW:
            cv2.namedWindow(WIN, cv2.WINDOW_NORMAL); cv2.resizeWindow(WIN, 960, 720)
        self.get_logger().info("READY align SIGN_BASE=%+d target j1=%.0f j2=%.0f" % (SIGN_BASE, J1T, J2T))

    def _pose_full(self):
        m = ArmJoints()
        m.joint1, m.joint2, m.joint3, m.joint4, m.joint5, m.joint6 = int(self.j1), int(self.j2), J3, J4, J5, 0
        m.time = 1200
        for _ in range(3):
            self.pub_joints.publish(m); time.sleep(0.15)

    def _send(self, jid, val):
        m = ArmJoint(); m.id = int(jid); m.joint = int(round(val)); m.time = 150
        self.pub_joint.publish(m)

    def odcb(self, m):
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        if self.x0 is None:
            self.x0 = (x, y)
        self.od = (x - self.x0[0], y - self.x0[1])

    def stop_base(self):
        z = Twist()
        for _ in range(2):
            self.pub_cmd.publish(z)

    def cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        ann, r = analyze(frame, self.model)
        H, W = ann.shape[:2]
        now = time.time()
        dist = math.hypot(*self.od)
        vx = wz = 0.0
        note = "no cube -> base stop"
        if r is not None and self.state == "ALIGN":
            eu = (r["u"] - W / 2.0) / (W / 2.0)
            ev = (r["v"] - H / 2.0) / (H / 2.0)
            # inner: arm centering
            if now - self.last >= CTRL_PERIOD:
                if abs(eu) > DB:
                    self.j1 = max(J1_MIN, min(J1_MAX, self.j1 + SIGN1 * max(-STEP, min(STEP, KP1 * eu))))
                    self._send(1, self.j1)
                if abs(ev) > DB:
                    self.j2 = max(J2_MIN, min(J2_MAX, self.j2 + SIGNV * max(-STEP, min(STEP, KPV * ev))))
                    self._send(2, self.j2)
                self.last = now
            centered = abs(eu) < DB + 0.02 and abs(ev) < DB + 0.02
            er = self.j1 - J1T      # +: arm panned right -> cube to the right
            ef = self.j2 - J2T      # +: arm tilted up -> cube far -> drive forward
            j1_ok = abs(er) <= DB_J1
            j2_ok = abs(ef) <= DB_J2
            # safety
            if dist > MAX_DIST or (now - self.t0) > MAX_TIME:
                self.state = "STOP"; note = "safety cap"
            elif j1_ok and j2_ok and centered:
                self.state = "ARRIVED"; note = "arrived at obs pose"
            else:
                # outer: base. face first (rotate j1->90), then approach (j2->45)
                if not j1_ok:
                    wz = with_min(SIGN_BASE * KROT * er, MINWZ, MAXWZ)
                elif not j2_ok:
                    vx = with_min(KFWD * ef, MINVX, MAXVX)
                note = "er=%+.0f ef=%+.0f j1=%.0f j2=%.0f%s%s" % (
                    er, ef, self.j1, self.j2, " j1ok" if j1_ok else "", " j2ok" if j2_ok else "")
        if self.state in ("STOP", "ARRIVED"):
            vx = wz = 0.0
        t = Twist(); t.linear.x = float(vx); t.angular.z = float(wz)
        self.pub_cmd.publish(t)

        col = (0, 255, 0) if self.state == "ALIGN" else ((0, 165, 255) if self.state == "ARRIVED" else (0, 0, 255))
        cv2.rectangle(ann, (0, 0), (W, 24), (0, 0, 0), -1)
        cv2.putText(ann, "[%s] %s vx=%.2f wz=%.2f d=%.2f" % (self.state, note, vx, wz, dist),
                    (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, 1, cv2.LINE_AA)
        self.pub_img.publish(self.br.cv2_to_imgmsg(ann, "bgr8"))
        self.n += 1
        if self.n % 5 == 0:
            cv2.imwrite(os.path.join(OUT_DIR, "align.jpg"), ann)
        if self.state == "ARRIVED" and self.n % 10 == 0:
            self.get_logger().info("ARRIVED j1=%.0f j2=%.0f ang=%.0f" % (self.j1, self.j2, r["ang"] if r else -1))
        if SHOW:
            cv2.imshow(WIN, ann)
            if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                self.stop_base(); rclpy.shutdown()


def main():
    rclpy.init()
    node = Align()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_base()
        if SHOW:
            cv2.destroyAllWindows()
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()
