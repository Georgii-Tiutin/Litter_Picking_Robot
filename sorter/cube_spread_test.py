#!/usr/bin/env python3
"""Measure WHERE the cube-position error comes from, before trying to filter it away.

Place ONE cube on the floor. This records every RAW estimate - no merging, no averaging -
from several robot positions, then separates the three candidate causes:

  * stationary spread, robot and arm still           -> pure depth/detector noise
  * spread BETWEEN viewpoint means                   -> calibration or TF-chain error
  * estimates that shift while the robot is turning  -> timestamp / TF synchronisation

The distinction matters because the fixes are completely different, and because a tracker
tuned to absorb a 20 cm error is hiding a bug rather than solving one.

Usage:  python3 cube_spread_test.py [n_viewpoints]
"""
import sys, math, time, json, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import tf2_ros, transforms3d as tfs
from ultralytics import YOLO

MODEL = "/home/jetson/cuboid_best_baseline.pt"
K     = [477.57421875,0.0,319.3820495605469,0.0,477.55718994140625,238.64108276367188,0.0,0.0,1.0]
CONF  = 0.25
OUT   = "/home/jetson/maps/cube_spread.json"
STATIONARY_FRAMES = 25
TURN_SPEED = 0.4

class Spread(Node):
    def __init__(self):
        super().__init__("cube_spread")
        self.b=CvBridge(); self.rgb=None; self.depth=None; self.rgb_stamp=None
        self.create_subscription(Image,"/camera/color/image_raw",self.cb_rgb,1)
        self.create_subscription(Image,"/camera/depth/image_raw",self.cb_d,1)
        self.cmd=self.create_publisher(Twist,"/cmd_vel",10)
        self.buf=tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf,self)
        self.model=YOLO(MODEL)
    def cb_rgb(self,m):
        self.rgb=self.b.imgmsg_to_cv2(m,"bgr8"); self.rgb_stamp=m.header.stamp
    def cb_d(self,m):
        self.depth=self.b.imgmsg_to_cv2(m,"32FC1").astype(np.float32)
    def spin(self,s):
        t=self.get_clock().now()
        while (self.get_clock().now()-t).nanoseconds<s*1e9: rclpy.spin_once(self,timeout_sec=0.05)
    def pose(self):
        try:
            t=self.buf.lookup_transform("map","base_footprint",rclpy.time.Time()).transform
            q=t.rotation
            return (t.translation.x,t.translation.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2)))
        except Exception: return None

    def observe(self, use_capture_time=True):
        """One raw estimate. Returns a dict, or None. NOTHING is merged or averaged."""
        if self.rgb is None or self.depth is None: return None
        rgb=self.rgb.copy(); dep=self.depth.copy(); stamp=self.rgb_stamp
        res=self.model.predict(rgb,imgsz=640,conf=CONF,verbose=False)[0]
        if len(res.boxes)==0: return None
        bx=max(res.boxes,key=lambda b:float(b.conf[0]))
        x1,y1,x2,y2=[int(v) for v in bx.xyxy[0]]
        mx1,my1=x1+(x2-x1)//4, y1+(y2-y1)//4
        mx2,my2=x2-(x2-x1)//4, y2-(y2-y1)//4
        patch=dep[max(0,my1):my2, max(0,mx1):mx2]
        patch=patch[np.isfinite(patch)&(patch>0)]
        if patch.size<10: return None
        z=float(np.median(patch))*0.001
        spread=float(np.std(patch))*0.001
        try:
            t_q=rclpy.time.Time.from_msg(stamp) if (use_capture_time and stamp) else rclpy.time.Time()
            tr=self.buf.lookup_transform("map","camera_color_optical_frame",t_q).transform
        except Exception:
            return None
        q=tr.rotation
        T=tfs.affines.compose([tr.translation.x,tr.translation.y,tr.translation.z],
                              tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z]),[1,1,1])
        fx,fy,cx0,cy0=K[0],K[4],K[2],K[5]
        u=(x1+x2)/2.0; vb=float(y2)
        d_cam=np.array([(u-cx0)/fx,(vb-cy0)/fy,1.0,0.0])
        Cw=(T@np.array([0.0,0.0,0.0,1.0]))[:3]; dw=(T@d_cam)[:3]
        if abs(dw[2])<1e-6: return None
        tt=-Cw[2]/dw[2]
        if tt<=0: return None
        F=Cw+tt*dw
        p=self.pose()
        return dict(x=float(F[0]), y=float(F[1]), z=float(F[2]), conf=float(bx.conf[0]),
                    rng=z, n_px=int(patch.size), depth_spread=spread,
                    robot=(p[0],p[1],p[2]) if p else None,
                    bearing=(math.degrees(math.atan2(F[1]-p[1],F[0]-p[0])) if p else None),
                    capture_time=bool(use_capture_time))

    def turn(self, rad):
        p0=self.pose()
        if p0 is None: return
        t=Twist(); t.angular.z=math.copysign(TURN_SPEED,rad); t0=time.time()
        while time.time()-t0<abs(rad)/TURN_SPEED+2.0:
            p=self.pose()
            if p and abs((p[2]-p0[2]+math.pi)%(2*math.pi)-math.pi)>=abs(rad): break
            self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)
        t=Twist()
        for _ in range(10): self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)
        self.spin(0.8)

def stats(obs):
    if len(obs)<2: return None
    xs=np.array([o["x"] for o in obs]); ys=np.array([o["y"] for o in obs])
    cx,cy=xs.mean(),ys.mean()
    r=np.hypot(xs-cx,ys-cy)
    return dict(n=len(obs), cx=float(cx), cy=float(cy),
                rms=float(np.sqrt((r**2).mean())), max=float(r.max()),
                sx=float(xs.std()), sy=float(ys.std()))

def main():
    n_views=int(sys.argv[1]) if len(sys.argv)>1 else 5
    rclpy.init(); e=Spread()
    print("waiting for camera and TF ...")
    t0=time.time()
    while time.time()-t0<40 and (e.rgb is None or e.depth is None or e.pose() is None):
        rclpy.spin_once(e,timeout_sec=0.2)
    if e.pose() is None: print("ABORT: no map transform - localise first"); return 2

    record={"stationary":[], "views":[], "latest_tf":[]}

    print("\n=== A. STATIONARY: %d frames, nothing moving ==="%STATIONARY_FRAMES)
    print("    any spread here is pure depth/detector noise")
    for i in range(STATIONARY_FRAMES):
        o=e.observe(use_capture_time=True)
        if o: record["stationary"].append(o)
        e.spin(0.25)
    s=stats(record["stationary"])
    if s: print("    %d obs, centre (%+.3f,%+.3f), rms %.3f m, worst %.3f m"
                %(s["n"],s["cx"],s["cy"],s["rms"],s["max"]))
    else: print("    no cube detected - place one in front of the robot and rerun")

    print("\n=== B. SAME SPOT, but transformed with the LATEST TF instead of capture time ===")
    print("    a difference here means timestamp handling matters even while stationary")
    for i in range(STATIONARY_FRAMES):
        o=e.observe(use_capture_time=False)
        if o: record["latest_tf"].append(o)
        e.spin(0.25)
    s2=stats(record["latest_tf"])
    if s2: print("    %d obs, centre (%+.3f,%+.3f), rms %.3f m"%(s2["n"],s2["cx"],s2["cy"],s2["rms"]))

    print("\n=== C. %d VIEWPOINTS: turning between each ==="%n_views)
    print("    spread BETWEEN viewpoint centres indicates calibration / TF-chain error")
    for v in range(n_views):
        if v: e.turn(2*math.pi/max(3,n_views+2))
        got=[]
        for i in range(10):
            o=e.observe(use_capture_time=True)
            if o: got.append(o)
            e.spin(0.2)
        st=stats(got)
        if st:
            hd=got[0]["robot"][2] if got[0]["robot"] else 0.0
            print("    view %d: %2d obs, centre (%+.3f,%+.3f), rms %.3f m, heading %+.0f deg"
                  %(v+1,st["n"],st["cx"],st["cy"],st["rms"],math.degrees(hd)))
            record["views"].append(dict(view=v+1, heading=hd, stats=st, obs=got))
        else:
            print("    view %d: cube not visible"%(v+1))

    json.dump(record, open(OUT,"w"), indent=1)
    print("\nraw observations written to %s"%OUT)

    print("\n=== VERDICT ===")
    if s:
        print("  depth/detector noise floor (stationary rms): %.3f m"%s["rms"])
    if s and s2:
        d=math.hypot(s2["cx"]-s["cx"], s2["cy"]-s["cy"])
        print("  capture-time vs latest-TF centre differs by : %.3f m %s"
              %(d,"(significant while stationary!)" if d>0.02 else "(negligible when still)"))
    if len(record["views"])>=2:
        cs=[(v["stats"]["cx"],v["stats"]["cy"]) for v in record["views"]]
        xs=np.array([c[0] for c in cs]); ys=np.array([c[1] for c in cs])
        between=float(np.sqrt(((xs-xs.mean())**2+(ys-ys.mean())**2).mean()))
        print("  spread BETWEEN viewpoint centres              : %.3f m"%between)
        if s and between > 2.5*max(s["rms"],1e-3):
            print("  -> viewpoint-dependent: points at CALIBRATION / TF-chain error,")
            print("     not at depth noise. Averaging will not fix this.")
        elif s:
            print("  -> consistent across viewpoints: the residual is mostly depth noise,")
            print("     which averaging and weighting genuinely do reduce.")
    rclpy.shutdown(); return 0

if __name__=="__main__": sys.exit(main())
