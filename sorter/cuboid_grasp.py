#!/usr/bin/env python3
"""Approach a detected cuboid to the vendor's 220 mm grasp distance and hand it
to grasp_hold via PosInfo.

Uses the vendor's calibrated hand-eye transform verbatim (EndToCamMat, camera K,
offset_value.yaml) so we inherit their calibration instead of re-deriving it.
The transform is only valid with the arm at init_joints = [90,120,0,0,90,90].
"""
import sys, math, time, yaml, numpy as np, cv2, rclpy
import transforms3d as tfs
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Bool
from arm_msgs.msg import ArmJoints
from arm_interface.msg import AprilTagInfo, CurJoints
from arm_interface.srv import ArmKinemarics
from ultralytics import YOLO

OFFS = yaml.safe_load(open("/home/jetson/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"))
INIT_JOINTS = [90, 120, 0, 0, 90, 90]
GRASP_DIST  = 175.0     # closer than the vendor 220 mm: the arm cannot reach low AND far
TOL         = 6.0
CONF        = 0.25
H_MIN, H_MAX = 0.005, 0.12

class CuboidGrasp(Node):
    def __init__(self):
        super().__init__("cuboid_grasp")
        self.b = CvBridge(); self.rgb = None; self.depth = None
        self.CurEndPos = [0.1279009179959246, 0.00023254956548456117, 0.1674898062979958,
                          0.00036263794618046863, 1.3962632350758744, 0.0003332603981328959]
        self.K = [477.57421875, 0.0, 319.3820495605469, 0.0,
                  477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
        self.EndToCamMat = np.array([[0,0,1,-0.101],[-1,0,0,0.002],
                                     [0,-1,0,4.82e-02],[0,0,0,1]])
        self.create_subscription(Image, "/camera/color/image_raw", self.cb_rgb, 1)
        self.create_subscription(Image, "/camera/depth/image_raw", self.cb_d, 1)
        self.vel  = self.create_publisher(Twist, "/cmd_vel", 1)
        self.arm  = self.create_publisher(ArmJoints, "arm6_joints", 10)
        self.pos  = self.create_publisher(AprilTagInfo, "PosInfo", 1)
        self.curj = self.create_publisher(CurJoints, "Curjoints", 1)
        self.j6   = self.create_publisher(Int16, "set_joint6", 10)
        self.kin = self.create_client(ArmKinemarics, "get_kinemarics")
        self.model = YOLO("/home/jetson/cuboid_best_baseline.pt")

    def refresh_end_pose(self):
        """CurEndPos MUST come from FK of the actual pose - the value hardcoded in the
        vendor source is stale and produces a ~77 mm error."""
        if not self.kin.wait_for_service(timeout_sec=8.0):
            print("WARN: kinematics service missing, using stale CurEndPos"); return False
        r = ArmKinemarics.Request()
        r.cur_joint1, r.cur_joint2, r.cur_joint3 = float(INIT_JOINTS[0]), float(INIT_JOINTS[1]), float(INIT_JOINTS[2])
        r.cur_joint4, r.cur_joint5, r.cur_joint6 = float(INIT_JOINTS[3]), float(INIT_JOINTS[4]), 0.0
        r.kin_name = "fk"
        f = self.kin.call_async(r); rclpy.spin_until_future_complete(self, f, timeout_sec=6.0)
        s = f.result()
        if s is None: return False
        self.CurEndPos = [s.x, s.y, s.z, s.roll, s.pitch, s.yaw]
        print("CurEndPos from FK: [%.5f, %.5f, %.5f, pitch %.5f]" % (s.x, s.y, s.z, s.pitch))
        return True
    def cb_rgb(self, m): self.rgb = self.b.imgmsg_to_cv2(m, "bgr8")
    def cb_d(self, m):   self.depth = self.b.imgmsg_to_cv2(m, "32FC1").astype(np.float32)
    def refresh(self, s=0.35):
        t0 = time.time()
        while time.time() - t0 < s: rclpy.spin_once(self, timeout_sec=0.05)

    # ---- vendor transform, copied verbatim ----
    def pixel_to_camera_depth(self, px, depth):
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        return np.array([(px[0]-cx)*depth/fx, (px[1]-cy)*depth/fy, depth])
    def xyz_euler_to_mat(self, xyz, euler):
        return tfs.affines.compose(np.squeeze(np.asarray(xyz)),
                                   tfs.euler.euler2mat(*euler), [1,1,1])
    def get_end_point_mat(self):
        q = tfs.euler.euler2quat(self.CurEndPos[3], self.CurEndPos[4], self.CurEndPos[5])
        return tfs.affines.compose(np.squeeze(np.asarray(self.CurEndPos[0:3])),
                                   tfs.quaternions.quat2mat(q), [1,1,1])
    def compute_pose(self, u, v, z):
        cam = self.pixel_to_camera_depth((u,v), z)
        world = np.matmul(self.get_end_point_mat(),
                          np.matmul(self.EndToCamMat, self.xyz_euler_to_mat(cam,(0,0,0))))
        T = tfs.affines.decompose(world)[0]
        return np.array([T[0]+OFFS["x_offset"], T[1]+OFFS["y_offset"], T[2]+OFFS["z_offset"]])

    # ---- perception ----
    def floor_normal(self):
        d = self.depth; h, w = d.shape
        vs, us = np.mgrid[h//2:h:6, 0:w:6]
        zs = d[vs,us]/1000.0
        ok = (zs>0.12)&(zs<2.0)
        if ok.sum() < 150: return None
        P = np.stack([self.pixel_to_camera_depth((u,v),z) for u,v,z in zip(us[ok],vs[ok],zs[ok])])
        c = P.mean(axis=0); n = np.linalg.svd(P-c)[2][-1]
        if n@np.array([0,1,0]) < 0: n = -n
        return n, c
    def best_cuboid(self):
        if self.rgb is None or self.depth is None: return None
        fp = self.floor_normal()
        r = self.model.predict(self.rgb, imgsz=640, conf=CONF, verbose=False)[0]
        cands = []
        for bx in r.boxes:
            x1,y1,x2,y2 = [float(t) for t in bx.xyxy[0]]
            u,v = int((x1+x2)/2), int((y1+y2)/2)
            patch = self.depth[max(0,v-4):v+5, max(0,u-4):u+5]
            g = patch[(patch>0)&np.isfinite(patch)]
            if g.size < 5: continue
            z = float(np.median(g))/1000.0
            if not (0.12 < z < 2.0): continue
            if fp is not None:
                n,c = fp
                hgt = float(-(self.pixel_to_camera_depth((u,v),z)-c)@n)
                if not (H_MIN < hgt < H_MAX): continue
            pose = self.compute_pose(u,v,z)
            dist = math.hypot(pose[0], pose[1])*1000
            cands.append((dist, u, v, z, float(bx.conf[0]), pose))
        return min(cands, key=lambda c: c[0]) if cands else None

def main():
    rclpy.init(); n = CuboidGrasp()
    t0 = time.time()
    while time.time()-t0 < 15 and (n.rgb is None or n.depth is None):
        rclpy.spin_once(n, timeout_sec=0.3)
    # arm to the pose the vendor transform assumes
    m = ArmJoints(); m.joint1,m.joint2,m.joint3,m.joint4,m.joint5,m.joint6 = INIT_JOINTS; m.time=2000
    t0=time.time()
    while n.arm.get_subscription_count()==0 and time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.1)
    for _ in range(4): n.arm.publish(m); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.1)
    time.sleep(2.5); n.refresh(1.0)
    n.refresh_end_pose()
    g = Int16(); g.data = 142
    for _ in range(3): n.j6.publish(g); time.sleep(0.1)

    t_start = time.time(); misses = 0
    while time.time()-t_start < 75:
        n.refresh(0.3)
        b = n.best_cuboid()
        if b is None:
            misses += 1; print("  no cuboid (%d)" % misses)
            for _ in range(3): n.vel.publish(Twist()); time.sleep(0.05)
            if misses > 8: print("LOST"); return
            continue
        misses = 0
        dist,u,v,z,conf,pose = b
        err = dist - GRASP_DIST
        print("  dist %6.1f mm  err %+6.1f  px(%d,%d) z %.2f conf %.2f" % (dist,err,u,v,z,conf))
        if abs(err) <= TOL:
            for _ in range(4): n.vel.publish(Twist()); time.sleep(0.05)
            cj = CurJoints(); cj.joints = INIT_JOINTS
            for _ in range(3): n.curj.publish(cj); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.15)
            time.sleep(0.6)
            p = AprilTagInfo(); p.id=1; p.x=float(u); p.y=float(v); p.z=float(z)
            for _ in range(3): n.pos.publish(p); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.15)
            print("HANDED OFF to grasp_hold: px(%d,%d) z %.3f  base dist %.1f mm" % (u,v,z,dist))
            return
        t = Twist(); t.linear.x = max(-0.06, min(0.06, 0.0009*err)); n.vel.publish(t)
    for _ in range(4): n.vel.publish(Twist()); time.sleep(0.05)
    print("TIMEOUT")

if __name__ == "__main__": main()
