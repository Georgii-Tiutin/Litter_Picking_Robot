#!/usr/bin/env python3
"""Tag-free, colour-free grasp: YOLO + depth only.

The depth camera sees the TOP of a cube. Aiming there closes the jaws above the
object, which is why the vendor grasp misses small cubes. We measure the object's
height above the fitted floor plane and aim at its MID-height instead.
All motion goes through move_to's guards (IK->FK check, clamp, re-validate).
"""
import sys, math, time, subprocess, yaml, numpy as np, rclpy
import transforms3d as tfs
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from arm_msgs.msg import ArmJoints
from geometry_msgs.msg import Twist
from arm_interface.srv import ArmKinemarics
from ultralytics import YOLO

OFFS = yaml.safe_load(open("/home/jetson/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"))
import os as _os
OBS_J2 = int(_os.environ.get('OBS_J2','105'))   # steeper pose sees closer in
INIT_JOINTS = [90, OBS_J2, 0, 0, 90, 90]
CONF = 0.25
GRIP_CLOSE, GRIP_OPEN = 142, 30
APPROACH_UP = 0.055           # m above the cube top for the approach
PITCH = 1.5708                # vertical gripper
import os
X_CORR = float(os.environ.get("X_CORR", "0.0"))   # systematic reach correction (m)
Y_CORR = float(os.environ.get("Y_CORR", "0.0"))

class DepthGrasp(Node):
    def __init__(self):
        super().__init__("grasp_depth")
        self.b=CvBridge(); self.rgb=None; self.depth=None
        self.K=[477.57421875,0.0,319.3820495605469,0.0,477.55718994140625,238.64108276367188,0.0,0.0,1.0]
        self.EndToCamMat=np.array([[0,0,1,-0.101],[-1,0,0,0.002],[0,-1,0,4.82e-02],[0,0,0,1]])
        self.CurEndPos=None
        self.create_subscription(Image,"/camera/color/image_raw",self.cb_rgb,1)
        self.create_subscription(Image,"/camera/depth/image_raw",self.cb_d,1)
        self.arm=self.create_publisher(ArmJoints,"arm6_joints",10)
        self.vel=self.create_publisher(Twist,"/cmd_vel",1)
        self.kin=self.create_client(ArmKinemarics,"get_kinemarics")
        self.model=YOLO("/home/jetson/cuboid_best_baseline.pt")
    def cb_rgb(self,m): self.rgb=self.b.imgmsg_to_cv2(m,"bgr8")
    def cb_d(self,m):   self.depth=self.b.imgmsg_to_cv2(m,"32FC1").astype(np.float32)
    def refresh(self,s=0.4):
        t0=time.time()
        while time.time()-t0<s: rclpy.spin_once(self,timeout_sec=0.05)
    def fk_end_pose(self):
        self.kin.wait_for_service(timeout_sec=8.0)
        r=ArmKinemarics.Request()
        r.cur_joint1,r.cur_joint2,r.cur_joint3=float(INIT_JOINTS[0]),float(INIT_JOINTS[1]),float(INIT_JOINTS[2])
        r.cur_joint4,r.cur_joint5,r.cur_joint6=float(INIT_JOINTS[3]),float(INIT_JOINTS[4]),0.0
        r.kin_name="fk"
        f=self.kin.call_async(r); rclpy.spin_until_future_complete(self,f,timeout_sec=6.0)
        s=f.result(); self.CurEndPos=[s.x,s.y,s.z,s.roll,s.pitch,s.yaw]
        print("CurEndPos (FK): [%.5f %.5f %.5f] pitch %.4f"%(s.x,s.y,s.z,s.pitch))
    def cam3d(self,u,v,z):
        fx,fy,cx,cy=self.K[0],self.K[4],self.K[2],self.K[5]
        return np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])
    def base_pose(self,u,v,z):
        cam=self.cam3d(u,v,z)
        q=tfs.euler.euler2quat(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
        end=tfs.affines.compose(np.asarray(self.CurEndPos[0:3]),tfs.quaternions.quat2mat(q),[1,1,1])
        cammat=tfs.affines.compose(np.squeeze(cam),tfs.euler.euler2mat(0,0,0),[1,1,1])
        T=tfs.affines.decompose(np.matmul(end,np.matmul(self.EndToCamMat,cammat)))[0]
        return np.array([T[0]+OFFS["x_offset"],T[1]+OFFS["y_offset"],T[2]+OFFS["z_offset"]])
    def floor(self):
        d=self.depth; h,w=d.shape
        vs,us=np.mgrid[h//2:h:6,0:w:6]; zs=d[vs,us]/1000.0
        ok=(zs>0.12)&(zs<2.0)
        if ok.sum()<150: return None
        P=np.stack([self.cam3d(u,v,z) for u,v,z in zip(us[ok],vs[ok],zs[ok])])
        c=P.mean(axis=0); n=np.linalg.svd(P-c)[2][-1]
        if n@np.array([0,1,0])<0: n=-n
        return n,c
    def find(self):
        fp=self.floor()
        r=self.model.predict(self.rgb,imgsz=640,conf=CONF,verbose=False)[0]
        best=None
        for bx in r.boxes:
            x1,y1,x2,y2=[float(t) for t in bx.xyxy[0]]
            u,v=int((x1+x2)/2),int((y1+y2)/2)
            patch=self.depth[max(0,v-4):v+5,max(0,u-4):u+5]
            g=patch[(patch>0)&np.isfinite(patch)]
            if g.size<5: continue
            z=float(np.median(g))/1000.0
            if not(0.12<z<1.0): continue
            hgt=None
            if fp is not None:
                n,c=fp; hgt=float(-(self.cam3d(u,v,z)-c)@n)
                if not(0.005<hgt<0.12): continue
            pose=self.base_pose(u,v,z)
            dist=math.hypot(pose[0],pose[1])
            # physical width of the box at this range
            wmm=(x2-x1)*z/self.K[0]*1000
            if best is None or dist<best[0]:
                best=(dist,pose,hgt,wmm,float(bx.conf[0]),u,v)
        return best

def move_to(x,y,z,j6,t=2000):
    cmd=("export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; "
         "source ~/yahboomcar_ws/install/setup.bash; "
         "python3 ~/calib/move_to.py %.4f %.4f %.4f %.4f %d %d"%(x,y,z,PITCH,j6,t))
    r=subprocess.run(["bash","-lc",cmd],capture_output=True,text=True,timeout=60)
    ok="SENT" in r.stdout
    print("   move (%.3f,%.3f,%.3f) j6=%d -> %s"%(x,y,z,j6,"ok" if ok else "REJECTED"))
    if not ok:
        for ln in r.stdout.strip().splitlines()[-2:]: print("     "+ln)
    return ok

def main():
    rclpy.init(); n=DepthGrasp()
    t0=time.time()
    while time.time()-t0<15 and (n.rgb is None or n.depth is None): rclpy.spin_once(n,timeout_sec=0.3)
    m=ArmJoints(); m.joint1,m.joint2,m.joint3,m.joint4,m.joint5,m.joint6=INIT_JOINTS; m.time=2000
    t0=time.time()
    while n.arm.get_subscription_count()==0 and time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.1)
    for _ in range(4): n.arm.publish(m); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.1)
    time.sleep(2.5); n.refresh(1.0); n.fk_end_pose()

    # final approach at the observation pose: close until inside the reach band
    TARGET_D = 0.165   # avoids the x=0.20 dead column; reachable at both approach and grip height
    t_end=time.time()+50
    while time.time()<t_end:
        n.refresh(0.35)
        b=n.find()
        if b is None:
            print("   approach: lost target"); 
            for _ in range(3): n.vel.publish(Twist()); time.sleep(0.05)
            break
        err=b[0]-TARGET_D
        if abs(err)<=0.012:
            for _ in range(4): n.vel.publish(Twist()); time.sleep(0.05)
            print("   approach done at %.0f mm"%(b[0]*1000)); break
        t=Twist(); t.linear.x=max(-0.05,min(0.05,0.30*err)); n.vel.publish(t)
        print("   approach: dist %.0f mm  err %+.0f  v %+.3f"%(b[0]*1000,err*1000,t.linear.x))
    for _ in range(4): n.vel.publish(Twist()); time.sleep(0.05)
    n.refresh(0.6)

    b=n.find()
    if b is None: print("no cuboid visible"); return
    dist,pose,hgt,wmm,conf,u,v=b
    print("target: base (%.3f, %.3f, top z %.3f)  height %.3f m  width %.0f mm  conf %.2f  dist %.0f mm"
          %(pose[0],pose[1],pose[2],hgt or -1,wmm,conf,dist*1000))
    if hgt is None: print("no floor plane - abort"); return
    z_grip = max(0.012, hgt*0.5)          # aim at MID height, not the top
    gx, gy = pose[0]+X_CORR, pose[1]+Y_CORR
    print("grip height %.3f m (half of %.3f);  correction x%+.3f y%+.3f -> grasp at (%.3f, %.3f)"
          %(z_grip,hgt,X_CORR,Y_CORR,gx,gy))
    if not (0.12 <= dist <= 0.26):
        print("out of the arm's floor reach band (0.14-0.25 m) - drive closer first"); return
    if not move_to(gx,gy,pose[2]+APPROACH_UP,GRIP_OPEN,1800): return
    time.sleep(0.6)
    if not move_to(gx,gy,z_grip,GRIP_OPEN,1500): return
    time.sleep(0.8)
    move_to(gx,gy,z_grip,GRIP_CLOSE,1200)
    time.sleep(1.5)
    print("GRASP SEQUENCE DONE")

if __name__=="__main__": main()
