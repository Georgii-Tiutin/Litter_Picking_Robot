#!/usr/bin/env python3
"""Turn toward the nearest floor cuboid and drive until it is at grasp range."""
import sys, math, time, numpy as np, rclpy
sys.path.insert(0, "/home/jetson/calib")
from detect_cuboids import Det
from geometry_msgs.msg import Twist
from arm_msgs.msg import ArmJoints

TARGET_RANGE = float(sys.argv[1]) if len(sys.argv) > 1 else 0.45   # m, camera to object
U_TOL   = 18          # px, horizontal centring tolerance
KP_W    = 0.0022      # rad/s per px
KP_V    = 0.35        # m/s per m
W_MAX, V_MAX = 0.45, 0.10

class Chase(Det):
    def __init__(self):
        super().__init__()
        self.vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.arm = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.j2  = 170
    def set_pitch(self, j2):
        """Tilt the camera. Lower joint2 looks further down / closer in."""
        if j2 == self.j2: return
        m = ArmJoints()
        m.joint1, m.joint2, m.joint3 = 90, int(j2), 0
        m.joint4, m.joint5, m.joint6 = 0, 90, 30
        m.time = 700
        for _ in range(3):
            self.arm.publish(m); rclpy.spin_once(self, timeout_sec=0.03); time.sleep(0.05)
        self.j2 = int(j2); self.refresh(0.9)
    def pitch_for(self, z):
        if z > 1.20: return 170
        if z > 0.85: return 155
        if z > 0.60: return 142
        return 128
    def stop(self):
        for _ in range(4): self.vel.publish(Twist()); time.sleep(0.04)
    def drive(self, v, w):
        t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.vel.publish(t)
    def refresh(self, secs=0.35):
        """Spin so image callbacks fire - without this the loop re-uses a stale frame."""
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)
    def best(self):
        self.refresh()
        fp, res = self.detect()
        acc = [r for r in res if r[5] == "CUBOID"]
        if not acc: return None
        return min(acc, key=lambda r: r[3])          # nearest by depth

def main():
    rclpy.init(); c = Chase()
    t0 = time.time()
    while time.time() - t0 < 15 and (c.rgb is None or c.depth is None or c.K is None):
        rclpy.spin_once(c, timeout_sec=0.3)
    print("chase: target range %.2f m" % TARGET_RANGE)
    misses, t_start = 0, time.time()
    while time.time() - t_start < 90:
        b = c.best()
        if b is None:
            misses += 1
            print("  no cuboid (%d)" % misses)
            c.stop()
            if misses in (2, 4):
                nxt = max(120, c.j2 - 14)
                print("  lost - tilting camera down to %d" % nxt); c.set_pitch(nxt)
            if misses > 6: print("LOST"); break
            c.refresh(0.4); continue
        misses = 0
        conf, u, v, z, h, _ = b
        c.set_pitch(c.pitch_for(z))
        du = u - 320
        if abs(du) <= U_TOL and abs(z - TARGET_RANGE) < 0.04:
            c.stop(); print("ARRIVED  range %.2f m  du %+d px  conf %.2f" % (z, du, conf)); break
        w = max(-W_MAX, min(W_MAX, -KP_W * du))
        v_cmd = 0.0
        if abs(du) <= U_TOL * 2:
            v_cmd = max(-V_MAX, min(V_MAX, KP_V * (z - TARGET_RANGE)))
        print("  z %.2f du %+4d -> v %+.3f w %+.3f  conf %.2f" % (z, du, v_cmd, w, conf))
        c.drive(v_cmd, w)
        c.refresh(0.30)
    c.stop()

if __name__ == "__main__": main()
