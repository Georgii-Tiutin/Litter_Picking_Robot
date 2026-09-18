#!/usr/bin/env python3
"""Publish base -> camera_depth_optical_frame so the depth cloud can feed the costmap.

WHY THIS IS NEEDED: the DCW2 depth camera is a fixed child of `arm4` in the URDF, i.e.
FOUR variable revolute joints from the base - and nothing on this robot publishes real
arm joint angles (there is no /joint_states at all). So TF cannot resolve the chain.

Instead we park the arm at a FIXED pose and compute the transform the same way the grasp
code does, which is verified accurate (0.2-1.4 mm, journal 2026-09-11):

    base -> end-effector      : FK via /get_kinemarics at the parked angles
    end-effector -> cam optical : EndToCamMat (vendor calibration)

HARD CONSTRAINT: while this is publishing, the arm MUST stay at NAV_POSE. If the arm
moves, every depth point is transformed wrongly and the costmap fills with garbage.
Stop this node before doing anything that moves the arm.
"""
import os, sys, math, time, numpy as np, rclpy
import transforms3d as tfs
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from arm_msgs.msg import ArmJoints
from arm_interface.srv import ArmKinemarics
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener

NAV_J2   = int(os.environ.get("NAV_J2","150"))
NAV_POSE = [90, NAV_J2, 0, 0, 90, 30]          # arm parked, claw open, clear of the chassis
PARENT   = os.environ.get("PARENT","base_footprint")
# Attach the camera to the robot at camera_link, the ROOT of the driver's own TF island.
#
# The earlier version published base_footprint -> camera_*_optical_frame directly. That
# was a real bug: the orbbec driver ALSO publishes camera_*_frame -> camera_*_optical_frame
# as static, so each optical frame had TWO parents. tf2 keys static transforms by CHILD
# frame, so the second publisher silently overwrites the first and which one is in force
# depends on publish order - different consumers can then disagree about where the camera
# points. Measured on 2026-09-12:
#     camera_depth_optical_frame  parents: base_footprint, camera_depth_frame
#     camera_color_optical_frame  parents: base_footprint, camera_color_frame
# Publishing to camera_link instead leaves every frame with exactly one parent, and lets
# the driver's own depth<->colour extrinsics apply (previously both optical frames were
# given the SAME transform, ignoring the ~2-3 cm baseline).
CHILD    = "camera_link"
LINK2OPT = "camera_depth_optical_frame"   # driver-internal frame END2CAM actually describes
END2CAM  = np.array([[0,0,1,-0.101],[-1,0,0,0.002],[0,-1,0,4.82e-02],[0,0,0,1]])

class NavCamTF(Node):
    def __init__(self):
        super().__init__("nav_camera_tf")
        self.arm=self.create_publisher(ArmJoints,"arm6_joints",10)
        self.kin=self.create_client(ArmKinemarics,"get_kinemarics")
        self.br=StaticTransformBroadcaster(self)
        self.buf=Buffer(); TransformListener(self.buf,self)

    def link_to_optical(self,timeout=15.0):
        """Read the driver's own camera_link -> optical extrinsic, so we can attach the
        camera at camera_link rather than fighting the driver for the optical frame."""
        t0=time.time()
        while time.time()-t0<timeout:
            rclpy.spin_once(self,timeout_sec=0.1)
            try:
                tr=self.buf.lookup_transform(CHILD,LINK2OPT,rclpy.time.Time()).transform
            except Exception:
                continue
            q=tr.rotation
            return tfs.affines.compose(
                [tr.translation.x,tr.translation.y,tr.translation.z],
                tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z]),[1,1,1])
        return None
    def spin(self,s):
        t0=time.time()
        while time.time()-t0<s: rclpy.spin_once(self,timeout_sec=0.05)
    def park(self):
        m=ArmJoints()
        m.joint1,m.joint2,m.joint3,m.joint4,m.joint5,m.joint6=[int(v) for v in NAV_POSE]
        m.time=2500
        t0=time.time()
        while self.arm.get_subscription_count()==0 and time.time()-t0<6:
            rclpy.spin_once(self,timeout_sec=0.05)
        if self.arm.get_subscription_count()==0:
            print("WARN: nothing subscribed to arm6_joints - is YB_Node running?")
            return False
        for _ in range(4):
            self.arm.publish(m); rclpy.spin_once(self,timeout_sec=0.03); time.sleep(0.1)
        time.sleep(3.0)
        return True
    def fk(self,j):
        if not self.kin.wait_for_service(timeout_sec=10.0):
            print("FAIL: /get_kinemarics not available"); return None
        r=ArmKinemarics.Request()
        r.cur_joint1,r.cur_joint2,r.cur_joint3=float(j[0]),float(j[1]),float(j[2])
        r.cur_joint4,r.cur_joint5,r.cur_joint6=float(j[3]),float(j[4]),0.0
        r.kin_name="fk"
        f=self.kin.call_async(r); rclpy.spin_until_future_complete(self,f,timeout_sec=8.0)
        return f.result()

def main():
    rclpy.init(); n=NavCamTF(); n.spin(1.0)
    print("parking arm at %s"%NAV_POSE)
    if not n.park(): return 2
    s=n.fk(NAV_POSE[:5])
    if s is None: return 2
    print("FK end-effector: (%.4f, %.4f, %.4f)  rpy (%.4f, %.4f, %.4f)"
          %(s.x,s.y,s.z,s.roll,s.pitch,s.yaw))

    q=tfs.euler.euler2quat(s.roll,s.pitch,s.yaw)
    T_base_end=tfs.affines.compose([s.x,s.y,s.z],tfs.quaternions.quat2mat(q),[1,1,1])
    T_base_opt=np.matmul(T_base_end,END2CAM)          # base -> depth optical frame

    T_link_opt=n.link_to_optical()
    if T_link_opt is None:
        print("FAIL: could not read %s -> %s from the driver. Is the camera node up?"%(CHILD,LINK2OPT))
        return 2
    # base -> camera_link = (base -> optical) * (camera_link -> optical)^-1
    T_base_link=np.matmul(T_base_opt,np.linalg.inv(T_link_opt))
    t,R,_,_=tfs.affines.decompose(T_base_link)
    qw,qx,qy,qz=tfs.quaternions.mat2quat(R)
    to,_,_,_=tfs.affines.decompose(T_base_opt)
    print("%s -> %s  (single parent; driver supplies the rest of the chain)"%(PARENT,CHILD))
    print("   translation (%.4f, %.4f, %.4f) m"%(t[0],t[1],t[2]))
    print("   quaternion  (%.4f, %.4f, %.4f, %.4f) xyzw"%(qx,qy,qz,qw))
    print("   implied %s at (%.4f, %.4f, %.4f)"%(LINK2OPT,to[0],to[1],to[2]))

    msg=TransformStamped()
    msg.header.stamp=n.get_clock().now().to_msg()
    msg.header.frame_id=PARENT; msg.child_frame_id=CHILD
    msg.transform.translation.x=float(t[0])
    msg.transform.translation.y=float(t[1])
    msg.transform.translation.z=float(t[2])
    msg.transform.rotation.x=float(qx); msg.transform.rotation.y=float(qy)
    msg.transform.rotation.z=float(qz); msg.transform.rotation.w=float(qw)
    msgs=[msg]
    n.br.sendTransform(msgs)
    print("\nstatic transform published. KEEP THIS RUNNING and do NOT move the arm.")
    # Re-send periodically: a listener that subscribes late can miss a one-shot latched
    # transform if QoS handshakes race, which showed up as intermittent "no camera TF".
    def resend():
        for m in msgs: m.header.stamp=n.get_clock().now().to_msg()
        n.br.sendTransform(msgs)
    n.create_timer(2.0, resend)
    rclpy.spin(n)

if __name__=="__main__": sys.exit(main())
