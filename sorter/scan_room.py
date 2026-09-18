#!/usr/bin/env python3
"""Rotate in place, detect floor cuboids at each heading."""
import sys, math, time, numpy as np, rclpy
sys.path.insert(0, "/home/jetson/calib")
from detect_cuboids import Det
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

STEPS = int(sys.argv[1]) if len(sys.argv) > 1 else 8
WZ, STEP_S = 0.6, 1.35

class Scan(Det):
    def __init__(self):
        super().__init__()
        self.vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.amcl = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.cb_amcl, 10)
    def cb_amcl(self, m): self.amcl = m.pose.pose
    def rotate(self, secs):
        t = Twist(); t.angular.z = WZ; t0 = time.time()
        while time.time() - t0 < secs:
            self.vel.publish(t); rclpy.spin_once(self, timeout_sec=0.05); time.sleep(0.05)
        for _ in range(5):
            self.vel.publish(Twist()); time.sleep(0.05)
    def settle(self, secs=1.4):
        t0 = time.time()
        while time.time() - t0 < secs: rclpy.spin_once(self, timeout_sec=0.1)
    def yaw_deg(self):
        if self.amcl is None: return float("nan")
        q = self.amcl.orientation
        return math.degrees(2 * math.atan2(q.z, q.w))

def main():
    rclpy.init(); s = Scan()
    t0 = time.time()
    while time.time() - t0 < 15 and (s.rgb is None or s.depth is None or s.K is None):
        rclpy.spin_once(s, timeout_sec=0.3)
    print("sensors ready; scanning")
    found = []
    for i in range(STEPS):
        s.settle()
        fp, res = s.detect()
        yaw = s.yaw_deg()
        acc = [r for r in res if r[5] == "CUBOID"]
        fpi = ("%.2f" % fp[2]) if fp else "none"
        print("  step %d yaw %6.0f  floor %s  raw %d  accepted %d"
              % (i, yaw, fpi, len(res), len(acc)))
        for conf, u, v, z, h, verd in res:
            zs = ("%.2fm" % z) if z else "  -  "
            hs = ("%+.3f" % h) if h is not None else "   -  "
            mark = "  <== CUBOID" if verd == "CUBOID" else ""
            print("      conf %.2f px(%3d,%3d) z %s h %s %s%s" % (conf, u, v, zs, hs, verd, mark))
        for a in acc: found.append((yaw, a))
        s.rotate(STEP_S)
    print("\ntotal accepted cuboid sightings: %d" % len(found))
    for yaw, a in found:
        print("   yaw %6.0f  conf %.2f  range %.2f m  height %+.3f m" % (yaw, a[0], a[3], a[4]))

if __name__ == "__main__": main()
