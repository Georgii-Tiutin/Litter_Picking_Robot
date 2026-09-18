#!/usr/bin/env python3
"""Drive onto a cube by VISUAL SERVOING into the arm's reachable zone, then grasp it.

The idea (user's): the reachable zone that vision_view.py paints green - floor whose
base-frame radial distance lies in the arm's reach annulus and is laterally centred - is
exactly the grasp precondition. So instead of guessing a stand-off distance and hoping, drive
until best.pt's cube actually lands inside that zone, then hand over to the grasp script.

Why this is better than the fixed GRASP_R it replaces:
  * the stand-off was bracketed between two invisible limits (arm reach 0.15-0.245 m, and the
    grasp camera which cannot see nearer than 0.16 m) and had to be tuned from crash reports
  * it was open-loop: ask for 0.145 m, get 0.131 m, discover the failure only after committing
  * the zone test is the SAME geometry the grasp itself uses, so "inside the zone" means
    "the arm can reach it", by construction

And it needs NO map and NO AMCL: every quantity is in the base frame, derived from FK of the
parked arm plus the vendor camera extrinsic. The transform dropouts that have broken every
map-based run cannot touch this loop.
"""
import os, sys, math, time, subprocess, yaml, numpy as np, cv2, rclpy
import transforms3d as tfs
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from arm_msgs.msg import ArmJoints
from arm_interface.srv import ArmKinemarics
from ultralytics import YOLO

MODEL   = "/home/jetson/cuboid_best_baseline.pt"
CONF    = float(os.environ.get("CONF","0.25"))
OBS_J2  = int(os.environ.get("OBS_J2","120"))
OBS     = [90, OBS_J2, 0, 0, 90, 30]
END2CAM = np.array([[0,0,1,-0.101],[-1,0,0,0.002],[0,-1,0,4.82e-02],[0,0,0,1]])

# the green zone, exactly as vision_view.py defines it
GRASP_MIN, GRASP_MAX = 0.150, 0.245
LAT_TOL   = 0.10          # |y| must be inside this for the wrist to line up
TARGET_R  = 0.1975        # middle of the annulus - aim here, tolerate the whole band
# A ROTATED claw does not reach as far as a straight one - the jaws swing out of the plane
# of the forearm, so the usable annulus shrinks. A cube that a straight grasp handles
# comfortably near the outer edge will be merely nudged by a rotated one (observed as
# "cube still at the same place (10-15 mm away)"). So when the wrist is going to be turned,
# require the WHOLE cube inside a tightened zone rather than just its centre.
# CENTRE the cube in the zone, do not merely get it inside. Measured 2026-09-15 over 11
# rounds: every failure sat at r = 0.218-0.226 m, the OUTER part of the 0.150-0.245 band,
# while successes clustered lower. "Inside" is too weak a test - at the outer edge the arm
# cannot reach low and far at once, and a rotated claw reaches less still.
CENTRE_R      = 0.1975           # middle of the annulus
CENTRE_TOL    = 0.025            # straight wrist: accept +/- this around the middle
CENTRE_ACCEPT = 0.012            # how close to the exact centre a grasp is accepted. Cannot
                                 # be tighter than the smallest move the base can make (~12 mm),
                                 # or the servo would chase a target it cannot reach.
CENTRE_TOL_ROT= 0.012            # +/- this around the annulus middle. A 45 deg grasp failed
                                 # on 2026-09-15 with the cube merely "inside" a 41 mm band -
                                 # the user's point: inside is not centred.
LAT_CENTRE    = 0.040            # and laterally centred, not just inside |y| < 0.10
LAT_CENTRE_ROT= 0.015
ROT_THRESH  = math.radians(20)   # beyond this much wrist rotation, treat it as "rotated"
ROT_MARGIN  = 0.020              # m shaved off the outer radius for a rotated grasp.
                                 # With CUBE_HALF this gives a 0.175-0.200 m band; 0.030
                                 # shrank it to 15 mm, tighter than the servo's own 8 mm
                                 # stopping tolerance can reliably hit.
CUBE_HALF   = 0.025              # m; half a cube, so the whole body clears the boundary
SQUARE_RATIO= 0.85

V_FWD     = 0.06          # m/s, deliberately slow: the last 10 cm decide the grasp
V_STRAFE      = 0.06      # commanded lateral speed (mecanum base)
V_STRAFE_EFF  = 0.021     # MEASURED result: 1.5 s at 0.06 produced 0.032 m, so the base
                          # realises about a third of the command. Duration is computed from
                          # the measured figure; the loop re-measures and iterates anyway.
# MEASURED base response (2026-09-15), commanded vs achieved:
#     0.025 m/s for 0.6 s -> 0.000 m   (2%)    <- slower is NOT finer, it is inert
#     0.025 m/s for 1.2 s -> 0.007 m  (23%)
#     0.060 m/s for 0.6 s -> 0.012 m  (34%)
#     0.060 m/s for 1.2 s -> 0.041 m  (57%)
#     0.060 m/s for 2.0 s -> 0.101 m  (85%)
# There is a deadband plus a ramp, so efficiency rises with DURATION, not with lower speed -
# the earlier "use a slower speed for fine moves" idea made small corrections impossible.
# Always drive at V_FWD and pick the duration from this curve. Smallest reliable move ~12 mm.
FWD_CURVE = [(0.012,0.6),(0.041,1.2),(0.101,2.0)]
MIN_MOVE  = 0.012         # anything smaller than this simply will not happen
W_TURN    = 0.45
W_MIN     = 0.35          # below this the base does not actually rotate - a commanded turn
                          # of 0.22 rad/s produced no movement at all and the loop stalled
STEP_MAX  = 0.12          # never move more than this between looks
YAW_TOL   = math.radians(4)
SETTLE    = 0.7
MAX_TRIES = 30
MAX_SEARCH_CREEPS = 3     # at 0.06 m each, at most ~0.18 m of blind forward search
Z_MIN,Z_MAX = 0.12, 2.0
H_MIN,H_MAX = -0.03, 0.15

class Servo(Node):
    def __init__(self):
        super().__init__("servo_grasp")
        self.b=CvBridge(); self.rgb=None; self.depth=None
        self.K=[477.57421875,0.0,319.3820495605469,0.0,477.55718994140625,238.64108276367188,0.0,0.0,1.0]
        self.create_subscription(Image,"/camera/color/image_raw",
                                 lambda m:setattr(self,"rgb",self.b.imgmsg_to_cv2(m,"bgr8")),1)
        self.create_subscription(Image,"/camera/depth/image_raw",
                                 lambda m:setattr(self,"depth",self.b.imgmsg_to_cv2(m,"32FC1").astype(np.float32)),1)
        self.scan=None
        self.create_subscription(LaserScan,"/scan",lambda m:setattr(self,"scan",m),
                                 rclpy.qos.qos_profile_sensor_data)
        self.cmd=self.create_publisher(Twist,"/cmd_vel",10)
        self.arm=self.create_publisher(ArmJoints,"arm6_joints",10)
        self.kin=self.create_client(ArmKinemarics,"get_kinemarics")
        self.model=YOLO(MODEL)
        self.T_base_cam=None

    def front_clear(self,dist):
        """Lidar check before any blind forward move."""
        s=self.scan
        if s is None: return True
        r=np.array(s.ranges); ang=s.angle_min+np.arange(len(r))*s.angle_increment
        ok=np.isfinite(r)&(r>s.range_min)&(r<s.range_max)&(np.abs(ang)<math.radians(25))
        if not ok.any(): return True
        return float(r[ok].min()) > dist+0.20

    def spin(self,s=0.4):
        t=time.time()
        while time.time()-t<s: rclpy.spin_once(self,timeout_sec=0.05)
    def stop(self):
        t=Twist()
        for _ in range(10): self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)

    def park(self):
        m=ArmJoints()
        m.joint1,m.joint2,m.joint3,m.joint4,m.joint5,m.joint6=[int(v) for v in OBS]
        m.time=2000
        t=time.time()
        while self.arm.get_subscription_count()==0 and time.time()-t<6:
            rclpy.spin_once(self,timeout_sec=0.05)
        for _ in range(3): self.arm.publish(m); rclpy.spin_once(self,timeout_sec=0.03); time.sleep(0.05)
        time.sleep(2.4)

    def fk_camera(self):
        """base_footprint -> camera, from FK of the PARKED arm. No TF, no localisation."""
        if not self.kin.wait_for_service(timeout_sec=10.0): return False
        r=ArmKinemarics.Request()
        r.cur_joint1,r.cur_joint2,r.cur_joint3=float(OBS[0]),float(OBS[1]),float(OBS[2])
        r.cur_joint4,r.cur_joint5,r.cur_joint6=float(OBS[3]),float(OBS[4]),0.0
        r.kin_name="fk"
        f=self.kin.call_async(r); rclpy.spin_until_future_complete(self,f,timeout_sec=8.0)
        s=f.result()
        if s is None: return False
        q=tfs.euler.euler2quat(s.roll,s.pitch,s.yaw)
        T_be=tfs.affines.compose([s.x,s.y,s.z],tfs.quaternions.quat2mat(q),[1,1,1])
        self.T_base_cam=np.matmul(T_be,END2CAM)
        print("   FK end-effector (%.4f, %.4f, %.4f) pitch %.3f rad"%(s.x,s.y,s.z,s.pitch))
        return True

    def to_base(self,u,v,z):
        fx,fy,cx,cy=self.K[0],self.K[4],self.K[2],self.K[5]
        P=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1.0])
        return (self.T_base_cam@P)[:3]

    def see_cube(self, frames=3):
        """Best cube in BASE coordinates, median over a few frames. None if not visible."""
        got=[]
        for _ in range(frames):
            self.spin(0.25)
            if self.rgb is None or self.depth is None: continue
            rgb=self.rgb.copy(); dep=self.depth.copy()
            res=self.model.predict(rgb,imgsz=640,conf=CONF,verbose=False)[0]
            best=None
            for bx in res.boxes:
                x1,y1,x2,y2=[int(t) for t in bx.xyxy[0]]
                mx1,my1=x1+(x2-x1)//4, y1+(y2-y1)//4
                mx2,my2=x2-(x2-x1)//4, y2-(y2-y1)//4
                patch=dep[max(0,my1):my2, max(0,mx1):mx2]
                patch=patch[np.isfinite(patch)&(patch>0)]
                if patch.size<10: continue
                z=float(np.median(patch))/1000.0
                if not (Z_MIN<z<Z_MAX): continue
                W=self.to_base((x1+x2)/2.0,(y1+y2)/2.0,z)
                if not (H_MIN<W[2]<H_MAX): continue
                c=float(bx.conf[0])
                ang,ratio=self.footprint(dep,x1,y1,x2,y2,z)
                if best is None or c>best[3]: best=(W[0],W[1],W[2],c,ang,ratio)
            if best: got.append(best)
        if not got: return None
        angs=[g[4] for g in got if g[4] is not None]
        rats=[g[5] for g in got if g[5] is not None]
        return (float(np.median([g[0] for g in got])),
                float(np.median([g[1] for g in got])),
                float(np.median([g[2] for g in got])),
                float(max(g[3] for g in got)),
                float(np.median(angs)) if angs else None,
                float(np.median(rats)) if rats else 1.0)

    def footprint(self,dep,x1,y1,x2,y2,z):
        """Short-axis angle and aspect of the cube's footprint, in the BASE frame.

        Mirrors what stationary_pickup measures, so the wrist angle it will choose can be
        predicted here - before the robot commits to a stand-off.
        """
        try:
            sub=dep[max(0,int(y1)):int(y2), max(0,int(x1)):int(x2)]
            if sub.size<40: return None,1.0
            m=np.isfinite(sub)&(sub>0)&(np.abs(sub/1000.0-z)<0.035)
            if m.sum()<30: return None,1.0
            vs,us=np.nonzero(m)
            us=us+int(x1); vs=vs+int(y1)
            pts=[]
            for u,v in zip(us[::3],vs[::3]):
                P=self.to_base(float(u),float(v),z)
                pts.append([P[0]*1000.0,P[1]*1000.0])
            if len(pts)<12: return None,1.0
            rect=cv2.minAreaRect(np.array(pts,dtype=np.float32))
            (rw,rh),rang=rect[1],rect[2]
            if rw<=0 or rh<=0: return None,1.0
            if rw<=rh: short,long_,ang = rw,rh,rang
            else:      short,long_,ang = rh,rw,rang+90.0
            return float(ang), float(short/max(long_,1e-6))
        except Exception:
            return None,1.0

    def predict_j5(self,x,y,ang,ratio):
        """The wrist angle stationary_pickup will choose, including its square preference."""
        if ang is None: return 90.0
        theta=math.degrees(math.atan2(y,x))
        j5=(90.0+ang-theta)%180.0
        if ratio>SQUARE_RATIO:
            alt=(j5+90.0)%180.0
            if abs(alt-90.0)<abs(j5-90.0): j5=alt
        return j5

    def zone_bounds(self,j5=None):
        """One acceptance band for EVERY grasp: the middle of the zone.

        The earlier version predicted the wrist angle here and relaxed the band when it
        looked straight. That prediction was simply wrong - measured 2026-09-15, the servo
        predicted j5~99 ("straight") on grasps the arm then performed at j5 117 and 130
        (clearly rotated), so the loose band was applied exactly where the tight one was
        needed. Two implementations of the same geometry disagreed, and the duplicate lost.

        So: no prediction. Always require the cube CENTRED, which is what a rotated claw
        needs and what a straight one tolerates anyway. The reported j5 below is for
        information only and no longer changes any decision.
        """
        lo,hi = CENTRE_R-CENTRE_TOL_ROT, CENTRE_R+CENTRE_TOL_ROT
        return lo,hi,LAT_CENTRE_ROT,True

    def in_zone(self,x,y,j5=None):
        """CENTRED, not merely inside.

        The previous version accepted the first look that fell anywhere in the band, so the
        loop stopped at whatever radius it happened to reach - measured 2026-09-15, that was
        r = 0.207-0.209 every round, the very top of a 0.185-0.210 band. A 45 deg grasp then
        failed for exactly the reason the user predicted: in the zone, but not in the middle
        of it. Accept only near the centre, and let the servo keep creeping until it is.
        """
        _,_,lat,_=self.zone_bounds(j5)
        return abs(math.hypot(x,y)-CENTRE_R) <= CENTRE_ACCEPT and abs(y) <= lat

    @staticmethod
    def fwd_duration(d):
        """Seconds of V_FWD needed to actually move d metres, from the measured curve."""
        d=abs(d)
        if d<=FWD_CURVE[0][0]: return FWD_CURVE[0][1]
        for (d0,t0),(d1,t1) in zip(FWD_CURVE, FWD_CURVE[1:]):
            if d<=d1: return t0+(t1-t0)*(d-d0)/(d1-d0)
        (dl,tl)=FWD_CURVE[-1]
        return min(4.0, tl+(d-dl)/0.075)      # extrapolate at the steady-state rate

    def drive(self,forward=0.0,turn=0.0,strafe=0.0):
        if abs(strafe)>1e-3:
            # Correct lateral offset by STRAFING, not turning. Turning to fix |y| needs a
            # bearing accuracy the base cannot hold at range - on 2026-09-15 it issued the
            # same -6.8 deg turn 29 times with no effect, because a 0.34 s pulse is shorter
            # than the base responds to. Strafing moves y directly and leaves r almost
            # untouched (measured drift: 1 mm over a 32 mm strafe).
            dur=min(2.5, abs(strafe)/V_STRAFE_EFF)
            t=Twist(); t.linear.y=math.copysign(V_STRAFE,strafe)
            t0=time.time()
            while time.time()-t0<dur: self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)
            self.stop(); self.spin(0.3)
        if abs(turn)>1e-3:
            w=max(W_MIN, min(W_TURN, abs(turn)*1.5))   # never below the base's dead-band
            t=Twist(); t.angular.z=math.copysign(w,turn)
            dur=abs(turn)/w
            t0=time.time()
            while time.time()-t0<dur: self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)
            self.stop(); self.spin(0.3)
        if abs(forward)>1e-3:
            f=math.copysign(min(STEP_MAX,max(MIN_MOVE,abs(forward))),forward)
            t=Twist(); t.linear.x=math.copysign(V_FWD,f)
            dur=self.fwd_duration(f)
            t0=time.time()
            while time.time()-t0<dur: self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)
            self.stop(); self.spin(0.3)

def sh(cmd,timeout=300):
    return subprocess.run(cmd,shell=True,capture_output=True,text=True,timeout=timeout)

def main():
    rounds=int(sys.argv[1]) if len(sys.argv)>1 else 0     # 0 = keep going
    rclpy.init(); e=Servo(); e.spin(1.0)
    print("parking the arm at the observation pose %s"%OBS)
    e.park()
    if not e.fk_camera():
        print("FAIL: no kinematics service"); return 2

    n=0
    while rounds==0 or n<rounds:
        n+=1
        print("\n=== round %d: servo the cube into the reachable zone ==="%n)
        print("   zone: %.3f < r < %.3f m and |y| < %.2f m"%(GRASP_MIN,GRASP_MAX,LAT_TOL))
        ok=False; misses=0
        for attempt in range(MAX_TRIES):
            s=e.see_cube()
            if s is None:
                # Creep forward to search - but STRICTLY bounded. Unbounded, this drove the
                # robot 1.8 m across the room on 2026-09-16 hunting a cube that was not there,
                # ending 0.18 m from an obstacle. A search fallback must not become a blind
                # traverse.
                misses+=1
                if misses>MAX_SEARCH_CREEPS:
                    print("   [%2d] no cube in view after %d creeps - giving up rather than"
                          " driving further"%(attempt+1,misses))
                    break
                if not e.front_clear(0.10):
                    print("   [%2d] no cube in view and the way ahead is not clear - stopping"
                          %(attempt+1))
                    break
                print("   [%2d] no cube in view (search creep %d/%d)"
                      %(attempt+1,misses,MAX_SEARCH_CREEPS))
                e.drive(forward=0.06)
                continue
            x,y,z,conf,ang,ratio = s
            r=math.hypot(x,y); yaw=math.atan2(y,x)
            j5=e.predict_j5(x,y,ang,ratio)
            lo,hi,lat,rot=e.zone_bounds(j5)
            inside=e.in_zone(x,y,j5)
            print("   [%2d] cube base (%+.3f,%+.3f) r=%.3f m  lat %+.3f  conf %.2f  "
                  "j5~%d(est)  want r %.3f-%.3f |y|<%.3f %s"
                  %(attempt+1,x,y,r,y,conf,int(round(j5)),
                    lo,hi,lat,"CENTRED" if inside else ""))
            if inside:
                ok=True; break
            # Only turn if the lateral offset actually threatens the zone's |y| < LAT_TOL.
            # The first version demanded centring to within YAW_TOL before it would drive
            # forward at all, and with |y| already 0.047 (well inside the 0.10 tolerance) it
            # spent 25 looks issuing turns too small for the base to execute, never advancing.
            # The zone is the specification - do not impose a stricter one.
            if abs(y) > LAT_TOL*0.7 and abs(yaw) > YAW_TOL:
                e.drive(turn=yaw); continue
            # Steer on BEARING, not on absolute lateral offset.
            # Gating the turn on |y| < 0.015 m demanded a 1.8 deg bearing accuracy while the
            # cube was still 0.49 m away - unachievable, so on 2026-09-15 the robot turned
            # 29 times and never drove forward at all. Lateral offset is y = r*sin(bearing),
            # so pointing at the cube and closing in shrinks it for free; only the final
            # acceptance needs to check |y|.
            if abs(y) > lat:
                # cube sits to one side: slide across to it rather than pivoting
                e.drive(strafe=y); continue
            step=r-((lo+hi)/2.0)
            if abs(step)<MIN_MOVE*0.8:   # no point commanding a move the base cannot make
                ok=e.in_zone(x,y,j5); break
            before=r
            e.drive(forward=step)
            s2=e.see_cube(frames=1)
            if s2 is not None and abs(math.hypot(s2[0],s2[1])-before)<0.01 and abs(step)>0.03:
                print("        range did not change (%.3f -> %.3f m) - the base may not be"
                      " moving; nudging harder"%(before,math.hypot(s2[0],s2[1])))
                e.drive(forward=math.copysign(max(0.06,abs(step)),step))
        e.stop()
        if not ok:
            print("   could not bring the cube into the zone; stopping best.pt anyway")
            return 1

        print("   cube is in the reachable zone - handing over to the grasp")
        # stationary_pickup runs in its OWN process with its own rclpy context, so this node
        # simply stays quiet while it works. (Tearing rclpy down and re-initialising it here
        # was fragile and bought nothing.)
        g=sh("cd /home/jetson/calib && ROS_DOMAIN_ID=30 python3 stationary_pickup.py")
        lines=[l for l in g.stdout.splitlines() if l.strip()]
        # Dump the COMPLETE grasp output. Diagnosing through a filter has now cost several
        # rounds: the lines that mattered (footprint aspect, height above floor, grip at)
        # were being dropped before they reached the log.
        try:
            with open("/tmp/grasp_full.log","a") as fh:
                fh.write("\n===== attempt %s rc=%d =====\n"%(time.strftime("%H:%M:%S"),g.returncode))
                fh.write(g.stdout)
        except Exception: pass
        # On failure, show the REASON rather than just the last few lines: the grasp aborts
        # for specific, diagnosable causes (span guard, unreachable, no cube) and truncating
        # to the tail hid exactly those on round 1.
        if g.returncode!=0:
            reasons=[l for l in lines if any(k in l for k in
                     ("ABORT","NO REACHABLE","NO CUBE","FAILED","still at the same place",
                      "short_ang","jaw axis","footprint aspect","square footprint",
                      "height above floor","grip at","consistency check"))]
            for l in (reasons or lines[-6:]): print("      | %s"%l.strip())
        if True:
            for l in lines:
                if "jaw axis" in l or "short_ang" in l: print("      > %s"%l.strip())
        else:
            # Log the SAME diagnostic lines on success as on failure. Previously success
            # printed only the last three lines, which omit the wrist/span measurements - so
            # failures could be characterised but never compared against successes, and any
            # hypothesis about what distinguishes them was untestable.
            keep=[l for l in lines if any(k in l for k in
                  ("jaw axis","short_ang","footprint aspect","height above floor",
                    "grip at","consistency check","using pitch"))]
            for l in keep[-4:]: print("      | %s"%l.strip())
            for l in lines[-2:]: print("      | %s"%l.strip())
        print("   grasp %s"%("SUCCEEDED" if g.returncode==0 else "FAILED"))
        if rounds and n>=rounds:
            e.stop(); rclpy.shutdown()
            return 0 if g.returncode==0 else 1

        # put it back down so the cycle can repeat on the same cube
        p=sh("cd /home/jetson/calib && ROS_DOMAIN_ID=30 python3 place_cube.py",timeout=200)
        print("   placed back down for the next round" if p.returncode==0 else "   place failed")
        e.park(); e.fk_camera()      # place_cube leaves the arm at NAV_POSE; go back to OBS
        e.drive(forward=-0.10)       # back off so the next approach has somewhere to go
    e.stop(); rclpy.shutdown()
    return 0

if __name__=="__main__": sys.exit(main())
