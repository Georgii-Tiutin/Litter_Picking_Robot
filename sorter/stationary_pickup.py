#!/usr/bin/env python3
"""stationary_pickup - pick up a cuboid in front of the robot without moving the base.

    observe -> FK -> detect (best.pt + depth) -> transform -> IK -> reach -> grasp -> verify

Tag-free and colour-free: works from the YOLO11n cuboid detector plus depth only,
which is the only option when demo blocks carry no AprilTags and unknown colours.

KEY FACTS, all measured on this robot (2026-09-11)
--------------------------------------------------
Coordinates are accurate.  The base-frame height and the depth floor-plane height
agree to 0.2-1.4 mm across every run, and measured footprints match the real blocks
(29-31 x 59-63 mm for a 3x6 cuboid).  Grasp failures are NOT a coordinate problem.

Wrist rule (derived, then confirmed):
        j5 = 90 + short_ang - theta          (mod 180)
    where short_ang is the block's short-axis angle in the BASE frame and
    theta = atan2(y, x) is the arm's bearing to it.  The jaw axis that results is
        jaw = theta + (j5 - 90)
    and the jaws must close ALONG the short axis.

    The vendor formula (grasp_desktop.py) CANNOT do this.  It wraps joint5 into
    [45,135] - a 90-degree window - but jaw orientation is 180-degree symmetric, so
    the wrap discards exactly the information that distinguishes a 30 mm face from a
    60 mm one.  Fine for their square AprilTag cubes, unusable for cuboids.  On the
    confirming run it said 108 where the correct answer was 161.

Span guard: the jaws must span  short*|cos(mis)| + long*|sin(mis)|  where mis is the
    misalignment.  For a 31 x 60 block that is 31 mm at 0 deg but 57 mm at 30 deg, so
    tolerance is only about +/-15 deg.  Anything over CLAW_MAX_MM aborts BEFORE moving
    - a bad angle then costs nothing instead of shoving the block out of reach.

Reliable grasp width is 45 mm, not the 60 mm physical opening (15 mm redundancy).

Reach: at grip height the arm reaches roughly 150-245 mm radial.  It cannot reach low
    AND far at once, so pitch is chosen automatically from PITCHES; the vertical
    gripper (1.5708) is almost always rejected.  Blocks outside the band must be
    approached by the chassis first.

Guards inherited from the course: IK -> FK round trip, clamp to 0..180, then
    RE-VALIDATE (clamping changes the pose), joint1 mirror (servo = 180 - ik), joint4
    capped at 90, and waiting for ROS2 subscriber discovery before publishing.

Environment overrides: OBS_J2, CONF, GRIP, X_CORR, Y_CORR, Z_CORR, J5_FORCE
"""
import os, sys, math, time, yaml, numpy as np, cv2, rclpy
import transforms3d as tfs
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from arm_msgs.msg import ArmJoints
from arm_interface.srv import ArmKinemarics
from ultralytics import YOLO

OFFS   = yaml.safe_load(open("/home/jetson/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"))
OBS_J2 = int(os.environ.get("OBS_J2", "120"))
OBS    = [90, OBS_J2, 0, 0, 90, 30]          # observation pose, claw OPEN
CONF   = float(os.environ.get("CONF", "0.25"))
GRIP_CLOSE = int(os.environ.get("GRIP", "142"))
GRIP_OPEN  = 30
X_CORR = float(os.environ.get("X_CORR", "0.0"))
Y_CORR = float(os.environ.get("Y_CORR", "0.0"))
Z_CORR = float(os.environ.get("Z_CORR", "0.0"))
H_MIN, H_MAX = 0.005, 0.12                    # valid object height above the floor plane
J5_SIGN   = float(os.environ.get("J5_SIGN", "1"))    # flip if the wrist turns the wrong way
J5_OFFSET = float(os.environ.get("J5_OFFSET", "0"))  # neutral-jaw reference, degrees
J5_FORCE  = os.environ.get("J5_FORCE")               # override for testing
CLAW_MAX_MM = 45.0   # RELIABLE grasp width; physical opening is 60 mm, 15 mm kept as redundancy
SQUARE_RATIO = 0.85   # above this the footprint is square and its angle is meaningless
CLEAR  = 0.055                                # approach height above the cube top
TOL_MM = 5.0
PITCHES = [1.5708, 1.45, 1.30, 1.15]          # try vertical first, then tilted

class Grab(Node):
    def __init__(self):
        super().__init__("grab_in_place")
        self.b=CvBridge(); self.rgb=None; self.depth=None; self.CurEndPos=None
        self.K=[477.57421875,0.0,319.3820495605469,0.0,477.55718994140625,238.64108276367188,0.0,0.0,1.0]
        self.EndToCamMat=np.array([[0,0,1,-0.101],[-1,0,0,0.002],[0,-1,0,4.82e-02],[0,0,0,1]])
        self.create_subscription(Image,"/camera/color/image_raw",self.cb_rgb,1)
        self.create_subscription(Image,"/camera/depth/image_raw",self.cb_d,1)
        self.arm=self.create_publisher(ArmJoints,"arm6_joints",10)
        self.kin=self.create_client(ArmKinemarics,"get_kinemarics")
        self.model=YOLO("/home/jetson/cuboid_best_baseline.pt")
    def cb_rgb(self,m): self.rgb=self.b.imgmsg_to_cv2(m,"bgr8")
    def cb_d(self,m):   self.depth=self.b.imgmsg_to_cv2(m,"32FC1").astype(np.float32)
    def spin(self,s=0.4):
        t0=time.time()
        while time.time()-t0<s: rclpy.spin_once(self,timeout_sec=0.05)

    # ---------- arm ----------
    def wait_sub(self,timeout=5.0):
        t0=time.time()
        while self.arm.get_subscription_count()==0 and time.time()-t0<timeout:
            rclpy.spin_once(self,timeout_sec=0.05)
    def send_joints(self,j,ms=2000):
        m=ArmJoints()
        m.joint1,m.joint2,m.joint3,m.joint4,m.joint5,m.joint6=[int(v) for v in j]
        m.time=int(ms)
        self.wait_sub()
        for _ in range(3):
            self.arm.publish(m); rclpy.spin_once(self,timeout_sec=0.03); time.sleep(0.08)
        time.sleep(ms/1000.0+0.4)

    # ---------- kinematics ----------
    def _call(self,req):
        f=self.kin.call_async(req); rclpy.spin_until_future_complete(self,f,timeout_sec=6.0)
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
        """IK -> clamp -> re-validate. Returns (servo_joints, err_mm) or (None, err)."""
        s=self.ik(x,y,z,pitch)
        if s is None: return None,9e9
        raw=[s.joint1,s.joint2,s.joint3,s.joint4,s.joint5]
        chk=self.fk(raw)
        if chk is None: return None,9e9
        err=math.dist((chk.x,chk.y,chk.z),(x,y,z))*1000
        out=[max(0,min(180,int(round(v)))) for v in raw]
        if out!=[int(round(v)) for v in raw]:
            chk2=self.fk(out)
            if chk2 is None: return None,9e9
            err=math.dist((chk2.x,chk2.y,chk2.z),(x,y,z))*1000
        if err>TOL_MM: return None,err
        servo=[180-out[0],out[1],out[2],out[3],out[4]]      # joint1 mirror
        return servo,err

    # ---------- perception ----------
    def cam3d(self,u,v,z):
        fx,fy,cx,cy=self.K[0],self.K[4],self.K[2],self.K[5]
        return np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])
    def to_base(self,u,v,z):
        cam=self.cam3d(u,v,z)
        q=tfs.euler.euler2quat(self.CurEndPos[3],self.CurEndPos[4],self.CurEndPos[5])
        end=tfs.affines.compose(np.asarray(self.CurEndPos[0:3]),tfs.quaternions.quat2mat(q),[1,1,1])
        cm=tfs.affines.compose(np.squeeze(cam),tfs.euler.euler2mat(0,0,0),[1,1,1])
        T=tfs.affines.decompose(np.matmul(end,np.matmul(self.EndToCamMat,cm)))[0]
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
    def measure(self, bx_xyxy, fp):
        """Estimate the cube from the depth pixels that actually sit on it.

        The YOLO box is axis-aligned and often includes shadow, so its centre is
        biased. Segment the cube inside the box by height above the floor plane,
        then use that mask's centroid and median depth."""
        x1,y1,x2,y2=[int(t) for t in bx_xyxy]
        H,W=self.depth.shape
        x1,y1=max(0,x1),max(0,y1); x2,y2=min(W-1,x2),min(H-1,y2)
        sub=self.depth[y1:y2+1, x1:x2+1]/1000.0
        vs,us=np.mgrid[y1:y2+1, x1:x2+1]
        valid=(sub>0.12)&(sub<1.2)&np.isfinite(sub)
        if valid.sum()<20 or fp is None: return None
        n,c=fp
        uu,vv,zz=us[valid],vs[valid],sub[valid]
        pts=np.stack([self.cam3d(u,v,z) for u,v,z in zip(uu,vv,zz)])
        hs=-(pts-c)@n
        on=(hs>H_MIN)&(hs<H_MAX)
        if on.sum()<15: return None
        u_c=float(uu[on].mean()); v_c=float(vv[on].mean())
        z_c=float(np.median(zz[on])); h_c=float(np.median(hs[on]))
        # footprint in the BASE frame: image angles are perspective-distorted, base ones are not
        idx=np.arange(on.sum())
        if len(idx)>320: idx=np.random.default_rng(0).choice(idx,320,replace=False)
        uo,vo,zo=uu[on][idx],vs[valid][on][idx] if False else vv[on][idx],zz[on][idx]
        pts_b=np.array([self.to_base(u,v,z)[:2] for u,v,z in zip(uo,vo,zo)],dtype=np.float32)
        # image-space rect too: the vendor derives joint5 from pixel corners
        img_pts=np.stack([uu[on],vv[on]],axis=1).astype(np.float32)
        irect=cv2.minAreaRect(img_pts)
        ibox=cv2.boxPoints(irect)
        ivx=float(ibox[0][0]-ibox[1][0]); ivy=float(ibox[0][1]-ibox[1][1])
        rect=cv2.minAreaRect((pts_b*1000.0).astype(np.float32))   # work in mm
        (rx,ry),(rw,rh),rang=rect
        if rw<=rh: short_mm,long_mm,short_ang = rw,rh,rang
        else:      short_mm,long_mm,short_ang = rh,rw,rang+90.0
        return dict(u=u_c,v=v_c,z=z_c,h=h_c,npx=int(on.sum()),ivx=ivx,ivy=ivy,
                    rect_x=rx/1000.0, rect_y=ry/1000.0,
                    short_mm=float(short_mm), long_mm=float(long_mm),
                    short_ang=float(short_ang))

    def detect_once(self):
        """One frame -> best on-floor cube, measured two ways for comparison."""
        fp=self.floor()
        r=self.model.predict(self.rgb,imgsz=640,conf=CONF,verbose=False)[0]
        out=[]
        for bx in r.boxes:
            x1,y1,x2,y2=[float(t) for t in bx.xyxy[0]]
            # (a) naive: box centre + fixed patch
            u0,v0=int((x1+x2)/2),int((y1+y2)/2)
            patch=self.depth[max(0,v0-4):v0+5,max(0,u0-4):u0+5]
            gpx=patch[(patch>0)&np.isfinite(patch)]
            if gpx.size<5: continue
            z0=float(np.median(gpx))/1000.0
            if not(0.12<z0<1.2): continue
            base0=self.to_base(u0,v0,z0)
            # (b) refined: depth-segmented centroid
            m=self.measure((x1,y1,x2,y2),fp)
            if m is None: continue
            u1,v1,z1,h1,npx=m["u"],m["v"],m["z"],m["h"],m["npx"]
            base1=self.to_base(u1,v1,z1)
            out.append(dict(conf=float(bx.conf[0]),
                            naive=dict(u=u0,v=v0,z=z0,base=base0),
                            fine =dict(u=u1,v=v1,z=z1,base=base1,h=h1,npx=npx,
                                       short_mm=m["short_mm"],long_mm=m["long_mm"],
                                       short_ang=m["short_ang"],ivx=m["ivx"],ivy=m["ivy"]),
                            w_mm=(x2-x1)*z1/self.K[0]*1000,
                            h_mm=(y2-y1)*z1/self.K[0]*1000))
        if not out: return None
        return max(out,key=lambda o:o["conf"])

    def detect(self, frames=7):
        """Average several frames: the estimate is noisy per-frame, stable in median."""
        obs=[]
        for _ in range(frames):
            self.spin(0.25)
            d=self.detect_once()
            if d: obs.append(d)
        if not obs: return None
        F=np.array([o["fine"]["base"] for o in obs])
        N=np.array([o["naive"]["base"] for o in obs])
        Hh=np.array([o["fine"]["h"] for o in obs])
        best=max(obs,key=lambda o:o["conf"])
        best["n_frames"]=len(obs)
        best["fine"]["base"]=np.median(F,axis=0)
        best["naive"]["base"]=np.median(N,axis=0)
        best["fine"]["h"]=float(np.median(Hh))
        best["spread_mm"]=float(np.max(np.linalg.norm(F-np.median(F,axis=0),axis=1))*1000)
        best["fine"]["short_mm"]=float(np.median([o["fine"]["short_mm"] for o in obs]))
        best["fine"]["long_mm"] =float(np.median([o["fine"]["long_mm"]  for o in obs]))
        angs=np.array([o["fine"]["short_ang"] for o in obs])%90.0     # 90-deg symmetry
        a=np.deg2rad(angs*4.0)
        best["fine"]["short_ang"]=float((np.rad2deg(np.arctan2(np.sin(a).mean(),np.cos(a).mean()))/4.0)%90.0)
        best["fine"]["ivx"]=float(np.median([o["fine"]["ivx"] for o in obs]))
        best["fine"]["ivy"]=float(np.median([o["fine"]["ivy"] for o in obs]))
        return best

def main():
    rclpy.init(); g=Grab()
    t0=time.time()
    while time.time()-t0<15 and (g.rgb is None or g.depth is None):
        rclpy.spin_once(g,timeout_sec=0.3)
    if g.rgb is None: print("no camera"); return 2
    if not g.kin.wait_for_service(timeout_sec=10.0): print("no kinematics service"); return 2

    print("=== 1. observation pose %s ==="%OBS)
    g.send_joints(OBS,2000); g.spin(1.0)

    print("=== 2. FK of the observation pose ===")
    s=g.fk(OBS[:5]); g.CurEndPos=[s.x,s.y,s.z,s.roll,s.pitch,s.yaw]
    print("    end-effector at (%.4f, %.4f, %.4f)  pitch %.4f rad (%.1f deg)"
          %(s.x,s.y,s.z,s.pitch,math.degrees(s.pitch)))

    print("=== 3. detect cube (best.pt, depth-segmented, multi-frame) ===")
    det=g.detect()
    if det is None: print("NO CUBE FOUND"); return 1
    f,nv=det["fine"],det["naive"]
    print("    conf %.2f   frames used %d   cube pixels %d   frame-to-frame spread %.1f mm"
          %(det["conf"],det["n_frames"],f["npx"],det["spread_mm"]))
    print("    box centre     : pixel (%d,%d)  depth %.3f m"%(nv["u"],nv["v"],nv["z"]))
    print("    segmented      : pixel (%.1f,%.1f)  depth %.3f m"%(f["u"],f["v"],f["z"]))
    print("    apparent size  : %.0f x %.0f mm (bounding box)"%(det["w_mm"],det["h_mm"]))
    print("    footprint       : %.0f x %.0f mm, short axis at %.1f deg (base frame)"
          %(f["short_mm"],f["long_mm"],f["short_ang"]))

    print("=== 4. transform pixel+depth -> base frame ===")
    b=f["base"]; bn=nv["base"]
    print("    box-centre estimate    : (%.4f, %.4f, %.4f) m"%(bn[0],bn[1],bn[2]))
    print("    segmented estimate     : (%.4f, %.4f, %.4f) m   <- used"%(b[0],b[1],b[2]))
    print("    refinement moved it by : %.1f mm"%(np.linalg.norm(b-bn)*1000))
    print("    radial distance        : %.1f mm"%(math.hypot(b[0],b[1])*1000))
    print("    height above floor     : %.4f m"%f["h"])
    print("    consistency check      : base z %.4f vs floor-plane height %.4f -> differ %.1f mm"
          %(b[2],f["h"],abs(b[2]-f["h"])*1000))

    cube_h = f["h"]
    gx,gy = b[0]+X_CORR, b[1]+Y_CORR
    gz    = max(0.010, cube_h*0.5) + Z_CORR
    above = cube_h + CLEAR
    print("=== 5. grasp target ===")
    print("    grip at (%.4f, %.4f, %.4f)   [mid-height of a %.0f mm cube]"
          %(gx,gy,gz,cube_h*1000))
    if X_CORR or Y_CORR or Z_CORR:
        print("    (corrections applied: x%+.3f y%+.3f z%+.3f)"%(X_CORR,Y_CORR,Z_CORR))

    def vendor_joint5(gripper_joint, servo_j1):
        """Vendor formula, verbatim from grasp_desktop.py lines 133-155."""
        j5=None
        if gripper_joint<0:
            j5=abs(gripper_joint)
            if abs(gripper_joint)<90: j5=180-servo_j1+j5-90
            else:                     j5=j5-servo_j1+90
        elif gripper_joint>0:
            j5=180-abs(gripper_joint)
            if gripper_joint<90: j5=j5-servo_j1
            else:                j5=j5-(servo_j1-90)
        else:
            return None
        while j5>135: j5-=90
        while j5<45:  j5+=90
        return int(round(j5))

    print("=== 5b. wrist alignment (vendor convention) ===")
    ratio=f["short_mm"]/max(f["long_mm"],1e-6)
    gripper_joint=math.degrees(math.atan2(f["ivx"],f["ivy"]))   # compute_joint5(vx,vy)
    print("    footprint aspect       : %.2f (short %.0f / long %.0f mm)"%(ratio,f["short_mm"],f["long_mm"]))
    print("    image corner vector    : vx %+.1f  vy %+.1f  -> gripper_joint %.1f deg"
          %(f["ivx"],f["ivy"],gripper_joint))
    if f["short_mm"]>CLAW_MAX_MM:
        print("    ABORT: short side %.0f mm exceeds the %.0f mm reliable grasp width"
              %(f["short_mm"],CLAW_MAX_MM))
        return 1

    print("=== 6. IK ===")
    chosen=None
    for p in PITCHES:
        j_above,e1=g.solve(gx,gy,above,p)
        j_grip ,e2=g.solve(gx,gy,gz   ,p)
        ok = j_above is not None and j_grip is not None
        print("    pitch %.4f (%4.1f deg): above %s  grip %s"
              %(p,math.degrees(p),
                ("ok %.2fmm"%e1) if j_above is not None else ("REJECT %.1fmm"%e1),
                ("ok %.2fmm"%e2) if j_grip  is not None else ("REJECT %.1fmm"%e2)))
        if ok and chosen is None: chosen=(p,j_above,j_grip)
    if chosen is None:
        print("NO REACHABLE SOLUTION - cube is outside the arm's envelope"); return 1
    p,j_above,j_grip=chosen
    print("    using pitch %.4f  above=%s  grip=%s"%(p,j_above,j_grip))

    print("=== 7. reach and grasp ===")
    servo_j1=j_grip[0]
    theta=math.degrees(math.atan2(gy,gx))
    # jaw = theta + (j5 - 90);  want jaw == short_ang  ->  j5 = 90 + short_ang - theta
    j5=(90.0+f["short_ang"]-theta)%180.0
    # The jaws must close along a FACE. For a SQUARE footprint both short_ang and
    # short_ang+90 are faces, so j5 and j5+90 are equally valid - but they are not equally
    # forgiving: a heavily rotated wrist sweeps a wider arc and needs the cube well inside
    # the reachable zone, while a near-straight wrist tolerates it sitting slightly outside.
    # So for a square, keep the face alignment and take the representative nearer 90 deg.
    #
    # NOT the same as forcing j5=90, which was tried on 2026-09-15 and was WRONG: it ignores
    # orientation entirely, and a 45 deg misalignment on a 40 mm square spans
    # 40*(|cos45|+|sin45|) = 57 mm, tripping the 45 mm span guard. The guard caught it.
    if ratio > SQUARE_RATIO:
        alt=(j5+90.0)%180.0
        if abs(alt-90.0) < abs(j5-90.0):
            print("    square footprint (aspect %.2f): both %d and %d align with a face,"
                  " taking the straighter wrist"%(ratio,int(round(j5)),int(round(alt))))
            j5=alt
    if j5>180: j5-=180
    j5=int(round(j5))
    if J5_FORCE is not None:
        j5=int(J5_FORCE); print("    joint5 FORCED to %d for this test"%j5)
    # what will the jaws actually have to span at this angle?
    jaw=(theta+j5-90.0)%180.0
    mis=(jaw-f["short_ang"]+90.0)%180.0-90.0
    span=f["short_mm"]*abs(math.cos(math.radians(mis)))+f["long_mm"]*abs(math.sin(math.radians(mis)))
    print("    bearing %.1f deg  short_ang %.1f deg  -> j5 %d  (vendor would say %d)"
          %(theta,f["short_ang"],j5,vendor_joint5(gripper_joint,servo_j1)))
    print("    jaw axis %.1f deg, misaligned %.1f deg -> jaws must span %.0f mm"%(jaw,mis,span))
    if span>CLAW_MAX_MM:
        print("    ABORT: %.0f mm span exceeds the %.0f mm reliable width - would shove the block, not grip it"
              %(span,CLAW_MAX_MM))
        g.send_joints(G.OBS if False else OBS,1800)
        return 1
    if j_grip[3]>90:
        print("    joint4 %d capped to 90 (vendor guard)"%j_grip[3])
        j_above[3]=min(j_above[3],90); j_grip[3]=min(j_grip[3],90)
    if j5 is not None:
        j_above=j_above[:4]+[j5]; j_grip=j_grip[:4]+[j5]
        print("    vendor joint5 = %d  (from gripper_joint %.1f, servo j1 %d)"%(j5,gripper_joint,servo_j1))
    else:
        print("    gripper_joint is 0 -> wrist neutral from IK")
    print("    above=%s  grip=%s"%(j_above,j_grip))
    g.send_joints(j_above+[GRIP_OPEN],1800)
    g.send_joints(j_grip +[GRIP_OPEN],1500)
    g.send_joints(j_grip +[GRIP_CLOSE],1200)
    time.sleep(0.8)
    g.send_joints(j_above+[GRIP_CLOSE],1500)

    print("=== 8. verify (return to observation pose and re-look) ===")
    g.send_joints([OBS[0],OBS[1],OBS[2],OBS[3],OBS[4],GRIP_CLOSE],2000); g.spin(1.2)
    still=g.detect(frames=4)
    if still is None:
        print("    no cube on the floor -> GRASP SUCCEEDED"); return 0
    d=math.hypot(still["fine"]["base"][0]-b[0],still["fine"]["base"][1]-b[1])*1000
    if d<40:
        print("    cube still at the same place (%.0f mm away) -> GRASP FAILED"%d)
        print("    it was commanded to (%.4f, %.4f); the cube now reads (%.4f, %.4f)"
              %(gx,gy,still["fine"]["base"][0],still["fine"]["base"][1]))
    else:
        print("    a cube is visible %.0f mm from the original -> likely a different one; check manually"%d)
    return 1

if __name__=="__main__": sys.exit(main())
