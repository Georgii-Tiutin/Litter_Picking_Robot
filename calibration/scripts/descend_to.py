#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Controlled descent over the known cube position for grasp bring-up.

Usage:
  python3 descend_to.py Z            # IK to (X,Y,Z) top-down, move, gripper OPEN
  python3 descend_to.py Z --close    # after moving, close gripper to hold
  python3 descend_to.py Z --lift     # after (optional close), lift to Z+0.08

X,Y default to the last detected cube (0.185, -0.002); override via env CUBE_X/CUBE_Y.
"""
import os
import sys
import time

import rclpy
from arm_msgs.msg import ArmJoints, ArmJoint
from arm_interface.srv import ArmKinemarics

X = float(os.environ.get("CUBE_X", "0.185"))
Y = float(os.environ.get("CUBE_Y", "-0.002"))
Z = float(sys.argv[1])
DO_CLOSE = "--close" in sys.argv
DO_LIFT = "--lift" in sys.argv
GRIP_OPEN = 30
GRIP_CLOSE = int(os.environ.get("GRIP_CLOSE", "120"))  # demo's proven hold value


def solve_ik(node, ik, x, y, z):
    req = ArmKinemarics.Request()
    req.tar_x, req.tar_y, req.tar_z = float(x), float(y), float(z)
    req.roll, req.pitch, req.yaw = 0.0, 1.5708, 0.0
    req.cur_joint1, req.cur_joint2, req.cur_joint3 = 90.0, 100.0, 0.0
    req.cur_joint4, req.cur_joint5, req.cur_joint6 = 0.0, 90.0, 0.0
    req.kin_name = "ik"
    fut = ik.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=6)
    r = fut.result()
    return None if r is None else [r.joint1, r.joint2, r.joint3, r.joint4, r.joint5]


def move(pub6, j, grip):
    cmd1 = 180 - j[0]
    if any(v < 0 or v > 180 for v in [cmd1, j[1], j[2], j[3], j[4]]):
        print(f"UNREACHABLE joints={[round(v,1) for v in j]} (cmd_j1={cmd1:.1f})")
        return False
    m = ArmJoints()
    m.joint1 = int(round(cmd1)); m.joint2 = int(round(j[1])); m.joint3 = int(round(j[2]))
    m.joint4 = int(round(j[3])); m.joint5 = int(round(j[4])); m.joint6 = int(grip)
    m.time = 2500
    pub6.publish(m)
    return True


def main():
    rclpy.init()
    node = rclpy.create_node("descend_to")
    pub6 = node.create_publisher(ArmJoints, "arm6_joints", 10)
    pub1 = node.create_publisher(ArmJoint, "arm_joint", 10)
    ik = node.create_client(ArmKinemarics, "get_kinemarics")
    if not ik.wait_for_service(timeout_sec=5):
        print("IK service unavailable"); return

    j = solve_ik(node, ik, X, Y, Z)
    if j is None:
        print("IK returned None"); return
    print(f"IK z={Z:.3f}: joints={[round(v,1) for v in j]}")
    if not move(pub6, j, GRIP_OPEN):
        return
    print(f"moving to X={X:.3f} Y={Y:.3f} Z={Z:.3f} (gripper open)...")
    time.sleep(3.5)

    if DO_CLOSE:
        g = ArmJoint(); g.id = 6; g.joint = GRIP_CLOSE; g.time = 1500
        pub1.publish(g); print(f"closing gripper to {GRIP_CLOSE}..."); time.sleep(2.5)

    if DO_LIFT:
        jl = solve_ik(node, ik, X, Y, Z + 0.08)
        if jl is not None:
            move(pub6, jl, GRIP_CLOSE if DO_CLOSE else GRIP_OPEN)
            print("lifting..."); time.sleep(3.0)

    print("DONE")


if __name__ == "__main__":
    main()
