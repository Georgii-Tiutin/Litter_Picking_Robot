#!/usr/bin/env python3
"""Place a held cube gently on the floor in front of the robot, then return to NAV_POSE.

PLACE, not drop: the arm lowers until the cube is a few mm above the floor, opens the jaws,
and only then retreats. Dropping from the carry height bounces the cube out of the zone.

Reuses the guards proven in stationary_pickup.py: IK -> FK round trip, clamp joints to
0..180, then RE-VALIDATE because clamping changes the pose, joint1 mirror (servo = 180-ik),
joint4 capped at 90, and waiting for subscriber discovery before publishing.

Ends at NAV_POSE with the claw open, which is the pose nav_camera_tf.py's transform assumes.
"""
import os, sys, math, time, numpy as np, rclpy
import transforms3d as tfs
from rclpy.node import Node
from arm_msgs.msg import ArmJoints
from arm_interface.srv import ArmKinemarics

PLACE_R   = float(os.environ.get("PLACE_R","0.20"))   # radial distance in front of the base
PLACE_Y   = float(os.environ.get("PLACE_Y","0.0"))
PLACE_Z   = float(os.environ.get("PLACE_Z","0.035"))  # cube centre height when released
LIFT      = 0.06                                      # approach/retreat height above that
GRIP_OPEN = 30
GRIP_HOLD = int(os.environ.get("GRIP","142"))
NAV_J2    = int(os.environ.get("NAV_J2","150"))   # navigation pose
NAV_POSE  = [90, NAV_J2, 0, 0, 90]
PITCHES   = [1.5708, 1.45, 1.30, 1.15]
TOL_MM    = 6.0

class Placer(Node):
    def __init__(self):
        super().__init__("place_cube")
        self.pub=self.create_publisher(ArmJoints,"arm6_joints",10)
        self.kin=self.create_client(ArmKinemarics,"get_kinemarics")
    def spin(self,s=0.4):
        t=time.time()
        while time.time()-t<s: rclpy.spin_once(self,timeout_sec=0.05)
    def wait_sub(self,timeout=6.0):
        t=time.time()
        while self.pub.get_subscription_count()==0 and time.time()-t<timeout:
            rclpy.spin_once(self,timeout_sec=0.05)
        return self.pub.get_subscription_count()>0
    def send(self,j,grip,ms=1800):
        m=ArmJoints()
        m.joint1,m.joint2,m.joint3,m.joint4,m.joint5=[int(v) for v in j[:5]]
        m.joint6=int(grip); m.time=int(ms)
        for _ in range(3):
            self.pub.publish(m); rclpy.spin_once(self,timeout_sec=0.03); time.sleep(0.05)
        time.sleep(ms/1000.0+0.35)
    def _call(self,r):
        if not self.kin.wait_for_service(timeout_sec=10.0): return None
        f=self.kin.call_async(r); rclpy.spin_until_future_complete(self,f,timeout_sec=8.0)
        return f.result()
    def fk(self,j):
        r=ArmKinemarics.Request()
        r.cur_joint1,r.cur_joint2,r.cur_joint3=float(j[0]),float(j[1]),float(j[2])
        r.cur_joint4,r.cur_joint5,r.cur_joint6=float(j[3]),float(j[4]),0.0
        r.kin_name="fk"; return self._call(r)
    def ik(self,x,y,z,pitch):
        r=ArmKinemarics.Request()
        r.tar_x,r.tar_y,r.tar_z=float(x),float(y),float(z)
        r.roll,r.pitch,r.yaw=0.0,float(pitch),0.0
        r.kin_name="ik"; return self._call(r)
    def solve(self,x,y,z,pitch):
        """IK -> clamp -> RE-VALIDATE -> mirror joint1 LAST.

        Copied from stationary_pickup.py rather than rewritten: the order matters. Clamping
        changes the pose, so FK must be re-run on the CLAMPED joints; and the joint1 servo
        mirror must be applied only after validation, because FK expects raw IK angles.
        My first version mirrored before validating and was simply wrong."""
        s=self.ik(x,y,z,pitch)
        if s is None: return None
        raw=[s.joint1,s.joint2,s.joint3,s.joint4,s.joint5]
        chk=self.fk(raw)
        if chk is None: return None
        err=math.sqrt((chk.x-x)**2+(chk.y-y)**2+(chk.z-z)**2)*1000
        out=[max(0,min(180,int(round(v)))) for v in raw]
        if out!=[int(round(v)) for v in raw]:
            chk2=self.fk(out)
            if chk2 is None: return None
            err=math.sqrt((chk2.x-x)**2+(chk2.y-y)**2+(chk2.z-z)**2)*1000
        if err>TOL_MM: return None
        servo=[180-out[0],out[1],out[2],out[3],out[4]]
        return servo,err

def main():
    rclpy.init(); p=Placer(); p.spin(1.0)
    if not p.wait_sub():
        print("FAIL: nothing subscribed to arm6_joints"); return 2
    tgt=(PLACE_R,PLACE_Y,PLACE_Z)
    print("placing at base (%.3f, %.3f, %.3f)"%tgt)
    chosen=None
    for pitch in PITCHES:
        a=p.solve(PLACE_R,PLACE_Y,PLACE_Z+LIFT,pitch)
        b=p.solve(PLACE_R,PLACE_Y,PLACE_Z,pitch)
        if a and b:
            chosen=(pitch,a[0],b[0],a[1],b[1]); break
        print("   pitch %.4f unreachable (above=%s grip=%s)"%(pitch,bool(a),bool(b)))
    if chosen is None:
        print("NO REACHABLE SOLUTION for the place pose"); 
        p.send(NAV_POSE,GRIP_HOLD,1800)
        return 1
    pitch,j_above,j_down,ea,ed=chosen
    print("   pitch %.4f  above=%s (%.1f mm)  down=%s (%.1f mm)"%(pitch,j_above,ea,j_down,ed))
    p.send(j_above,GRIP_HOLD,1800)      # over the spot, still holding
    p.send(j_down ,GRIP_HOLD,1500)      # lower until the cube is on the floor
    time.sleep(0.4)
    p.send(j_down ,GRIP_OPEN,900)       # release GENTLY, cube already resting
    time.sleep(0.5)
    p.send(j_above,GRIP_OPEN,1400)      # retreat straight up, do not drag it
    p.send(NAV_POSE,GRIP_OPEN,1800)     # back to the pose the camera TF assumes
    print("PLACED and returned to NAV_POSE")
    rclpy.shutdown(); return 0

if __name__=="__main__": sys.exit(main())
