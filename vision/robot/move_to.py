#!/usr/bin/env python3
"""Move the arm end-effector to a target pose, safely.

Applies every guard the course taught us:
  - IK -> FK round trip, reject if position error > tolerance (section 1)
  - clamp all joints to 0..180 (section 2)
  - mirror joint1: servo j1 = 180 - ik_j1 (section 2)
  - joint6 held explicitly, IK always returns 0 for it (section 1)

usage: move_to.py X Y Z [PITCH_RAD] [JOINT6] [TIME_MS]
"""
import sys, time, rclpy
from rclpy.node import Node
from arm_interface.srv import ArmKinemarics
from arm_msgs.msg import ArmJoints

TOL_MM = 5.0

class Mover(Node):
    def __init__(self):
        super().__init__("calib_mover")
        self.cli = self.create_client(ArmKinemarics, "get_kinemarics")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("waiting for /get_kinemarics ...")
        self.pub = self.create_publisher(ArmJoints, "arm6_joints", 10)

    def call(self, req):
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def ik(self, x, y, z, pitch):
        r = ArmKinemarics.Request()
        r.tar_x, r.tar_y, r.tar_z = float(x), float(y), float(z)
        r.roll, r.pitch, r.yaw = 0.0, float(pitch), 0.0
        r.kin_name = "ik"
        return self.call(r)

    def fk(self, j):
        r = ArmKinemarics.Request()
        r.cur_joint1, r.cur_joint2, r.cur_joint3 = float(j[0]), float(j[1]), float(j[2])
        r.cur_joint4, r.cur_joint5, r.cur_joint6 = float(j[3]), float(j[4]), 0.0
        r.kin_name = "fk"
        return self.call(r)


def wait_for_subscriber(node, pub, timeout=5.0):
    """ROS2 discovery is not instant: publishing before the subscriber is
    matched silently drops the message. Wait for the match first."""
    t0 = time.time()
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 > timeout:
            print("WARN: no subscriber on arm6_joints after %.1fs" % timeout)
            return False
    return True

def main():
    x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    pitch  = float(sys.argv[4]) if len(sys.argv) > 4 else 1.5708
    j6     = int(sys.argv[5])   if len(sys.argv) > 5 else 135
    tms    = int(sys.argv[6])   if len(sys.argv) > 6 else 2000

    rclpy.init(); m = Mover()
    sol = m.ik(x, y, z, pitch)
    if sol is None:
        print("FAIL ik: no response"); return 2
    j = [sol.joint1, sol.joint2, sol.joint3, sol.joint4, sol.joint5]

    chk = m.fk(j)
    if chk is None:
        print("FAIL fk: no response"); return 2
    err = (((chk.x-x)**2 + (chk.y-y)**2 + (chk.z-z)**2) ** 0.5) * 1000
    print(f"ik joints raw: {[round(v,2) for v in j]}")
    print(f"fk check: ({chk.x:.4f},{chk.y:.4f},{chk.z:.4f})  err {err:.2f} mm")
    if err > TOL_MM:
        print(f"REJECT: unreachable (err {err:.1f} mm > {TOL_MM} mm)"); return 1

    out = [max(0, min(180, int(round(v)))) for v in j]
    if out != [int(round(v)) for v in j]:
        print(f"note: clamped {[round(v,1) for v in j]} -> {out}")
        # re-validate: clamping changes the pose, so the earlier check is void
        chk2 = m.fk(out)
        if chk2 is None:
            print("FAIL fk after clamp"); return 2
        err2 = (((chk2.x-x)**2 + (chk2.y-y)**2 + (chk2.z-z)**2) ** 0.5) * 1000
        print(f"fk after clamp: ({chk2.x:.4f},{chk2.y:.4f},{chk2.z:.4f})  err {err2:.2f} mm")
        if err2 > TOL_MM:
            print(f"REJECT: clamped pose misses target by {err2:.1f} mm - not sending")
            return 1

    msg = ArmJoints()
    msg.joint1 = 180 - out[0]          # servo mirror
    msg.joint2, msg.joint3 = out[1], out[2]
    msg.joint4, msg.joint5 = out[3], out[4]
    msg.joint6 = j6
    msg.time = tms
    wait_for_subscriber(m, m.pub)
    for _ in range(3):
        m.pub.publish(msg); rclpy.spin_once(m, timeout_sec=0.05); time.sleep(0.1)
    print(f"SENT j1..j6 = {msg.joint1},{msg.joint2},{msg.joint3},{msg.joint4},{msg.joint5},{msg.joint6}")
    time.sleep(tms/1000.0 + 0.5)
    return 0

if __name__ == "__main__":
    sys.exit(main())
