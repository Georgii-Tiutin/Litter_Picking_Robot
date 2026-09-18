#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compute a top-down grasp pose from a live detection via the floor
homography, and optionally execute the grasp.

Pipeline: best.pt detects the cube at the fixed observation pose -> take the
bbox bottom-centre pixel (floor contact) -> apply the calibrated homography H
-> floor (X, Y) in the arm base frame. With --grasp, command the arm through
the pick sequence via /get_kinemarics IK + /arm6_joints.

Usage:
  python3 grasp_pose.py            # DRY RUN: detect -> (X,Y), print, no motion
  python3 grasp_pose.py --grasp    # also execute the pick (moves the arm!)

Grasp geometry knobs (override via env): CUBE_H, GRIP_Z, HOVER_DZ.
"""
import os
import sys
import time

import numpy as np
import cv2
import yaml
from ultralytics import YOLO

import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from arm_msgs.msg import ArmJoints, ArmJoint
from arm_interface.srv import ArmKinemarics

MODEL = "/home/jetson/models/cuboid_v1/best.pt"
HYAML = "/home/jetson/project0/calibration/floor_homography/floor_homography.yaml"
OBS_POSE = [90, 100, 0, 0, 90, 0]          # observation pose = home
CONF = 0.25

CUBE_H = float(os.environ.get("CUBE_H", "0.03"))    # cube height (m)
GRIP_Z = float(os.environ.get("GRIP_Z", "0.02"))    # fingertip height above floor at close (m)
HOVER_DZ = float(os.environ.get("HOVER_DZ", "0.05"))  # clearance above cube top for hover (m)
GRIP_OPEN = 30
GRIP_CLOSE = 130
DO_GRASP = "--grasp" in sys.argv
DO_HOVER = "--hover" in sys.argv


def load_H():
    d = yaml.safe_load(open(HYAML))
    return np.array(d["H"], dtype=np.float64)


def pixel_to_floor(H, u, v):
    p = np.array([[[u, v]]], dtype=np.float64)
    xy = cv2.perspectiveTransform(p, H).reshape(2)
    return float(xy[0]), float(xy[1])


class GraspNode:
    def __init__(self):
        self.node = rclpy.create_node("grasp_pose")
        self.bridge = CvBridge()
        self.frame = None
        self.node.create_subscription(Image, "/camera/color/image_raw", self._cb, 10)
        self.pub6 = self.node.create_publisher(ArmJoints, "arm6_joints", 10)
        self.pub1 = self.node.create_publisher(ArmJoint, "arm_joint", 10)
        self.ik = self.node.create_client(ArmKinemarics, "get_kinemarics")

    def _cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def wait_frame(self, t=5):
        t0 = time.time()
        while self.frame is None and time.time() - t0 < t:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return self.frame

    def pub_six(self, joints, runtime=2500):
        m = ArmJoints()
        # driver expects joint1 = 180 - kinematic_joint1 (see arm-control notes)
        m.joint1 = int(round(180 - joints[0]))
        m.joint2 = int(round(joints[1]))
        m.joint3 = int(round(joints[2]))
        m.joint4 = int(round(joints[3]))
        m.joint5 = int(round(joints[4]))
        m.joint6 = int(round(joints[5]))
        m.time = runtime
        self.pub6.publish(m)

    def pub_grip(self, val, runtime=1200):
        m = ArmJoint(); m.id = 6; m.joint = int(val); m.time = runtime
        self.pub1.publish(m)

    def solve_ik(self, x, y, z, pitch=1.5708):
        req = ArmKinemarics.Request()
        req.tar_x, req.tar_y, req.tar_z = float(x), float(y), float(z)
        req.roll, req.pitch, req.yaw = 0.0, float(pitch), 0.0
        req.cur_joint1, req.cur_joint2, req.cur_joint3 = 90.0, 100.0, 0.0
        req.cur_joint4, req.cur_joint5, req.cur_joint6 = 0.0, 90.0, 0.0
        req.kin_name = "ik"
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=6)
        r = fut.result()
        if r is None:
            return None
        return [r.joint1, r.joint2, r.joint3, r.joint4, r.joint5, 0.0]


def main():
    H = load_H()
    model = YOLO(MODEL)
    rclpy.init()
    g = GraspNode()
    frame = g.wait_frame()
    if frame is None:
        print("NO_FRAME"); return

    r = model.predict(frame, conf=CONF, device=0, verbose=False)[0]
    if len(r.boxes) == 0:
        print("NO_DETECTION"); return
    best = max(r.boxes, key=lambda b: float(b.conf))
    x1, y1, x2, y2 = [float(v) for v in best.xyxy[0]]
    u = (x1 + x2) / 2.0
    v = y2
    conf = float(best.conf)
    X, Y = pixel_to_floor(H, u, v)
    print(f"DETECT conf={conf:.2f} pixel=({u:.1f},{v:.1f}) -> FLOOR X={X:.4f} Y={Y:.4f} m")

    if not (DO_GRASP or DO_HOVER):
        print("DRY RUN (no motion). Re-run with --hover or --grasp.")
        return

    # --- execute ---
    if not g.ik.wait_for_service(timeout_sec=5):
        print("IK service unavailable"); return
    z_hover = CUBE_H + HOVER_DZ

    # hover above the cube (open gripper), then stop or continue
    j = g.solve_ik(X, Y, z_hover)
    if j is None or any(vv < 0 or vv > 180 for vv in [180 - j[0]] + j[1:5]):
        print(f"IK FAIL/unreachable at hover: {j}"); return
    print(f"hover: IK joints={[round(vv,1) for vv in j[:5]]}")
    g.pub_grip(GRIP_OPEN); time.sleep(0.5)
    g.pub_six(j); time.sleep(3.0)
    print(f"HOVER reached: {z_hover*100:.0f} cm above floor at X={X:.3f} Y={Y:.3f}")
    if DO_HOVER:
        print("HOVER-ONLY mode. Check gripper alignment, then run --grasp.")
        return

    # descend to grip height
    j = g.solve_ik(X, Y, GRIP_Z)
    if j is None or any(vv < 0 or vv > 180 for vv in [180 - j[0]] + j[1:5]):
        print(f"IK FAIL/unreachable at descend: {j}"); return
    print(f"descend: IK joints={[round(vv,1) for vv in j[:5]]}")
    g.pub_six(j); time.sleep(3.0)
    print("close gripper"); g.pub_grip(GRIP_CLOSE); time.sleep(2.0)
    # lift
    j = g.solve_ik(X, Y, CUBE_H + 0.08)
    if j is not None:
        print("lift"); g.pub_six(j); time.sleep(3.0)
    # return to observation pose, keep holding
    print("return to obs pose"); g.pub_six(OBS_POSE); time.sleep(3.0)
    print("GRASP SEQUENCE DONE")


if __name__ == "__main__":
    main()
