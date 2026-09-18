#!/usr/bin/env python3
"""Patrol the mapped room, find cuboids with best.pt + depth, mark them in RViz.

Mission:
  1. build waypoints covering the free space of the RECORDED map
  2. before driving anywhere, ask the planner whether the route is safe;
     if there is no path, or the path squeezes through inscribed cost, ABANDON that
     waypoint rather than push through (a wedged robot corrupts everything downstream)
  3. at each waypoint stop and rotate through 360 deg in steps, running best.pt at each
     step - the camera FOV is only ~58 deg, so a standing scan is the only way to see a
     whole room
  4. back-project each detection with the depth image, lift it into the map frame, merge
     repeat sightings, and publish a MarkerArray for RViz

Obstacle avoidance is Nav2's, using the costmap fixed on 2026-09-12: obstacle_layer only
(voxel_layer disabled - it fabricated phantoms), fed by BOTH /scan and /camera/depth/points
with min_obstacle_height 0.02 so that even a 3 cm cuboid is an obstacle.

The arm MUST stay parked at NAV_POSE with nav_camera_tf.py running: every depth point and
every detection is transformed using that fixed camera pose.
"""
import sys, math, time, numpy as np, rclpy, cv2
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros, transforms3d as tfs
from ultralytics import YOLO
from cube_tracker import CubeTracker, Detection

MODEL      = "/home/jetson/cuboid_best_baseline.pt"
K          = [477.57421875, 0.0, 319.3820495605469,
              0.0, 477.55718994140625, 238.64108276367188, 0.0, 0.0, 1.0]
CONF       = 0.25
DEPTH_MM   = True          # this driver publishes 32FC1 in MILLIMETRES (see vision_view.py)

FLOOR_PROJECT = True          # locate a cube by intersecting the ray through the BOTTOM of
                              # its box with the floor plane z=0, instead of trusting the depth
                              # median. A cube sits on the floor, so that intersection is fully
                              # determined by the camera pose and the pixel - no depth noise,
                              # which was scattering repeat sightings of one cube over >0.4 m.
H_MIN,H_MAX   = -0.03, 0.20   # a cuboid on the floor. Generous on purpose: the box centre
                              # lands near the cube's BASE, not its top, so a real cube can
                              # compute to ~0.006 m (measured). The gate exists to reject
                              # detections up on walls and furniture, not to measure height.
R_MIN,R_MAX   = 0.30, 1.80    # depth is trustworthy here; beyond that bearing errors blow up
MERGE_M       = 0.30          # sightings closer than this are the same cube. 0.22 was too
                              # tight: the SAME cube seen from a new viewpoint lands 15-25 cm
                              # away (bearing error at 1.5 m + AMCL error), so it registered as
                              # a new cube and the count climbed past the 7 actually present.
# (CONSOLIDATE_M / MIN_SIGHTINGS are unused since the tracker replaced the greedy merge;
#  confirmation is now CONFIRM_OBS + CONFIRM_VIEWS inside cube_tracker.py)
CONSOLIDATE_M = 0.38
MIN_SIGHTINGS = 2             # a real cube is seen many times across 8-step scans at several
                              # waypoints; a single sighting is usually a false positive

SCAN_STEPS    = 8             # 8 x 45 deg = full turn
TURN_SPEED    = 0.5
SETTLE        = 1.2           # let the image and TF settle before inference

WP_SPACING    = 1.15          # m between patrol points
ROBOT_RADIUS  = 0.22
SAFE_MARGIN   = 0.12          # extra clearance required to STAND at a waypoint
FREE_MAX      = 55            # cartographer's uncertain band sits at 45-55
GOAL_TIMEOUT  = 150.0
STUCK_DIST    = 0.04
STUCK_YAW     = 0.12          # rad. Nav2 rotates IN PLACE to reach the goal yaw, making zero
                              # translation for longer than STUCK_WIN - which made a robot that
                              # had arrived correctly report STUCK, and skip its cube scan.
STUCK_WIN     = 16.0
BATT_ABORT    = 10.80         # higher than the mapper's 10.65: 10.73 V was not enough to
                              # shut down cleanly on 2026-09-12

class CubePatrol(Node):
    def __init__(self):
        super().__init__("cube_patrol")
        self.map=None; self.gcost=None; self.rgb=None; self.depth=None; self.batt=None
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,"/map",lambda m:setattr(self,"map",m),latched)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",
                                 lambda m:setattr(self,"gcost",m),latched)
        self.b=CvBridge()
        self.create_subscription(Image,"/camera/color/image_raw",self.cb_rgb,1)
        self.create_subscription(Image,"/camera/depth/image_raw",self.cb_d,1)
        self.create_subscription(Float32,"/battery",lambda m:setattr(self,"batt",float(m.data)),10)
        self.mk=self.create_publisher(MarkerArray,"/cube_markers",latched)
        self.cmd=self.create_publisher(Twist,"/cmd_vel",10)
        self.nav=ActionClient(self,NavigateToPose,"navigate_to_pose")
        self.plan=ActionClient(self,ComputePathToPose,"compute_path_to_pose")
        self.tfbuf=tf2_ros.Buffer(); tf2_ros.TransformListener(self.tfbuf,self)
        self.model=YOLO(MODEL)
        # Whole-frame data association instead of merging each detection into the nearest
        # cube. Two boxes in ONE image are two different physical objects; a greedy nearest
        # merge can fold both into one track and destroy that fact. See cube_tracker.py.
        self.tracker=CubeTracker(gate=MERGE_M)

    def cb_rgb(self,m): self._cb_rgb(m)
    def cb_d(self,m):   self.depth=self.b.imgmsg_to_cv2(m,"32FC1").astype(np.float32)

    def _cb_rgb(self,m):
        # Keep the CAPTURE TIME with the image. Transforming a detection with the LATEST
        # transform instead of the one for the shutter instant silently bakes every bit of
        # motion between capture and processing into the cube's position.
        self.rgb=self.b.imgmsg_to_cv2(m,"bgr8")
        self.rgb_stamp=m.header.stamp

    def spin(self,s):
        t=self.get_clock().now()
        while (self.get_clock().now()-t).nanoseconds<s*1e9: rclpy.spin_once(self,timeout_sec=0.05)

    def pose(self):
        try:
            t=self.tfbuf.lookup_transform("map","base_footprint",rclpy.time.Time()).transform
            q=t.rotation
            return (t.translation.x,t.translation.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2)))
        except Exception: return None

    def stop(self):
        t=Twist()
        for _ in range(12): self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)

    # ---------------- waypoints from the recorded map ----------------
    def waypoints(self):
        m=self.map
        g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
        res=m.info.resolution
        free=((g>=0)&(g<FREE_MAX)).astype(np.uint8)
        # a waypoint must be somewhere the robot can actually STAND
        r=max(1,int(round((ROBOT_RADIUS+SAFE_MARGIN)/res)))
        safe=cv2.erode(free,np.ones((2*r+1,2*r+1),np.uint8))
        step=max(1,int(round(WP_SPACING/res)))
        pts=[]
        for cy in range(0,m.info.height,step):
            row=[]
            for cx in range(0,m.info.width,step):
                if safe[cy,cx]:
                    row.append((m.info.origin.position.x+(cx+0.5)*res,
                                m.info.origin.position.y+(cy+0.5)*res))
            # serpentine: reverse alternate rows so the robot does not cross the room each time
            if (cy//step)%2: row.reverse()
            pts.extend(row)
        print("map %dx%d, %d free cells, %d safe-to-stand, %d waypoints at %.2f m spacing"
              %(m.info.width,m.info.height,int(free.sum()),int(safe.sum()),len(pts),WP_SPACING))
        return pts

    # ---------------- is this route safe to attempt? ----------------
    def route_ok(self,x,y):
        """Ask the planner first. Abandon rather than push through anything tight."""
        if not self.plan.wait_for_server(timeout_sec=8.0): return False,"no planner"
        g=ComputePathToPose.Goal(); g.use_start=False
        p=PoseStamped(); p.header.frame_id="map"
        p.pose.position.x=float(x); p.pose.position.y=float(y); p.pose.orientation.w=1.0
        g.goal=p
        f=self.plan.send_goal_async(g); rclpy.spin_until_future_complete(self,f,timeout_sec=10)
        h=f.result()
        if h is None or not h.accepted: return False,"planner rejected"
        rf=h.get_result_async(); rclpy.spin_until_future_complete(self,rf,timeout_sec=20)
        if rf.result() is None: return False,"planner timed out"
        poses=rf.result().result.path.poses
        if not poses: return False,"no path"
        c=self.gcost
        if c is None: return True,"path ok (no costmap to check)"
        a=np.array(c.data,dtype=np.int16).reshape(c.info.height,c.info.width)
        res=c.info.resolution; lethal=0; inscribed=0
        for q in poses:
            cx=int((q.pose.position.x-c.info.origin.position.x)/res)
            cy=int((q.pose.position.y-c.info.origin.position.y)/res)
            if not(0<=cx<c.info.width and 0<=cy<c.info.height): continue
            v=a[cy,cx]
            if v>=100: lethal+=1
            elif v>=99: inscribed+=1
        frac=inscribed/max(1,len(poses))
        if lethal: return False,"path crosses %d LETHAL cells"%lethal
        if frac>0.15: return False,"path is %.0f%% inscribed - too tight"%(100*frac)
        return True,"path ok (%d poses, %.0f%% inscribed)"%(len(poses),100*frac)

    # ---------------- drive ----------------
    def goto(self,x,y):
        if not self.nav.wait_for_server(timeout_sec=10.0): return "NO SERVER"
        g=NavigateToPose.Goal()
        g.pose.header.frame_id="map"; g.pose.header.stamp=self.get_clock().now().to_msg()
        g.pose.pose.position.x=float(x); g.pose.pose.position.y=float(y)
        g.pose.pose.orientation.w=1.0
        f=self.nav.send_goal_async(g); rclpy.spin_until_future_complete(self,f,timeout_sec=12)
        h=f.result()
        if h is None or not h.accepted: return "REJECTED"
        rf=h.get_result_async(); t0=time.time()
        last=self.pose(); last_t=time.time()
        while time.time()-t0<GOAL_TIMEOUT:
            rclpy.spin_once(self,timeout_sec=0.2)
            if rf.done(): break
            now=self.pose()
            if now and last:
                moved=math.hypot(now[0]-last[0],now[1]-last[1])
                turned=abs((now[2]-last[2]+math.pi)%(2*math.pi)-math.pi)
                if moved>STUCK_DIST or turned>STUCK_YAW: last,last_t=now,time.time()
                elif time.time()-last_t>STUCK_WIN:
                    h.cancel_goal_async(); self.stop(); return "STUCK"
        if not rf.done():
            h.cancel_goal_async(); self.stop(); return "TIMEOUT"
        return {4:"SUCCEEDED",5:"CANCELED",6:"ABORTED"}.get(rf.result().status,"?")

    # ---------------- perception ----------------
    def detect(self):
        """One inference on the current frame -> list of Detection objects in the MAP frame."""
        if self.rgb is None or self.depth is None: return []
        rgb=self.rgb.copy(); dep=self.depth.copy()
        try:
            stamp=getattr(self,"rgb_stamp",None)
            t_q=rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time()
            tr=self.tfbuf.lookup_transform("map","camera_color_optical_frame",t_q).transform
        except Exception:
            # capture-time transform unavailable; fall back to latest rather than dropping
            # the frame, but that bakes in any motion since the shutter fired
            try:
                tr=self.tfbuf.lookup_transform("map","camera_color_optical_frame",
                                               rclpy.time.Time()).transform
            except Exception: return []
        q=tr.rotation
        T=tfs.affines.compose([tr.translation.x,tr.translation.y,tr.translation.z],
                              tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z]),[1,1,1])
        res=self.model.predict(rgb,imgsz=640,conf=CONF,verbose=False)[0]
        out=[]
        fx,fy,cx0,cy0=K[0],K[4],K[2],K[5]
        for bx in res.boxes:
            x1,y1,x2,y2=[int(v) for v in bx.xyxy[0]]
            # median depth over the middle of the box: edges straddle the floor behind it
            mx1,my1=x1+(x2-x1)//4, y1+(y2-y1)//4
            mx2,my2=x2-(x2-x1)//4, y2-(y2-y1)//4
            patch=dep[max(0,my1):my2, max(0,mx1):mx2]
            patch=patch[np.isfinite(patch)&(patch>0)]
            if patch.size<10: continue
            scale=(0.001 if DEPTH_MM else 1.0)
            z=float(np.median(patch))*scale
            n_px=int(patch.size)
            depth_spread=float(np.std(patch))*scale
            if not (R_MIN<z<R_MAX): continue
            u=(x1+x2)/2.0; v=(y1+y2)/2.0
            P=np.array([(u-cx0)*z/fx,(v-cy0)*z/fy,z,1.0])
            W=(T@P)[:3]
            if not (H_MIN<W[2]<H_MAX): continue     # must be sitting on the floor
            if FLOOR_PROJECT:
                # ray through the BOTTOM-centre of the box, in camera coords, then to map
                vb=float(y2)
                dir_cam=np.array([(u-cx0)/fx,(vb-cy0)/fy,1.0,0.0])
                Cw=(T@np.array([0.0,0.0,0.0,1.0]))[:3]      # camera origin in map
                dw=(T@dir_cam)[:3]
                if abs(dw[2])>1e-6:
                    t=-Cw[2]/dw[2]
                    if t>0:
                        F=Cw+t*dw
                        # only trust it if it lands near where depth said, else keep depth
                        if math.hypot(F[0]-W[0],F[1]-W[1])<0.45:
                            W=np.array([F[0],F[1],0.02])
            out.append(Detection(float(W[0]),float(W[1]),float(W[2]),
                                 conf=float(bx.conf[0]), rng=z,
                                 bbox=(x1,y1,x2,y2),
                                 n_px=n_px, depth_spread=depth_spread))
        return out

    def add(self,dets):
        """Associate an ENTIRE frame at once, under a one-to-one constraint."""
        p=self.pose()
        before=len(self.tracker.tracks)
        self.tracker.update(dets, robot_xy=(p[0],p[1]) if p else None,
                            stamp=time.time())
        return max(0,len(self.tracker.tracks)-before)


    def consolidate(self):
        """No longer needed: one-to-one association keeps tracks distinct as they are
        created, instead of merging afterwards and hoping the radius was right."""
        return

    def publish(self):
        ma=MarkerArray()
        for i,t in enumerate(self.tracker.tracks):
            confirmed = (t.state=="confirmed")
            m=Marker()
            m.header.frame_id="map"; m.header.stamp=self.get_clock().now().to_msg()
            m.ns="cubes"; m.id=t.id; m.type=Marker.CUBE; m.action=Marker.ADD
            m.pose.position.x=t.x; m.pose.position.y=t.y; m.pose.position.z=max(t.z,0.02)
            m.pose.orientation.w=1.0
            m.scale.x=m.scale.y=m.scale.z=0.06
            if confirmed: m.color.r,m.color.g,m.color.b=0.1,1.0,0.2
            else:         m.color.r,m.color.g,m.color.b=1.0,0.72,0.0
            m.color.a=0.95
            ma.markers.append(m)
            lab=Marker(); lab.header=m.header; lab.ns="cube_labels"; lab.id=1000+t.id
            lab.type=Marker.TEXT_VIEW_FACING; lab.action=Marker.ADD
            lab.pose.position.x=t.x; lab.pose.position.y=t.y; lab.pose.position.z=0.22
            lab.pose.orientation.w=1.0; lab.scale.z=0.09
            lab.color.r=lab.color.g=lab.color.b=1.0; lab.color.a=1.0
            lab.text="#%d x%d/%dv %s"%(t.id,t.n,len(t.views),
                                       "" if confirmed else "?")
            ma.markers.append(lab)
        self.mk.publish(ma)

    def scan_here(self):
        """Rotate on the spot, running best.pt at each step. The FOV is ~58 deg, so without
        this the robot would only ever see the strip straight ahead of it."""
        found=0
        for k in range(SCAN_STEPS):
            self.spin(SETTLE)
            d=self.detect()
            found+=self.add(d)
            self.publish()
            if d: print("       step %d/%d: %d detection(s), %d track(s), %d confirmed"
                        %(k+1,SCAN_STEPS,len(d),len(self.tracker.tracks),
                          len(self.tracker.confirmed())))
            if k<SCAN_STEPS-1:
                turn=2*math.pi/SCAN_STEPS
                t=Twist(); t.angular.z=TURN_SPEED
                dur=turn/TURN_SPEED
                t0=time.time()
                while time.time()-t0<dur:
                    self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)
                self.stop(); self.spin(0.4)
        return found

def main():
    rclpy.init(); e=CubePatrol()
    print("waiting for map, camera and TF ...")
    t0=time.time()
    while time.time()-t0<40 and (e.map is None or e.rgb is None or e.depth is None or e.pose() is None):
        rclpy.spin_once(e,timeout_sec=0.2)
    for what,ok in (("map",e.map is not None),("colour",e.rgb is not None),
                    ("depth",e.depth is not None),("pose",e.pose() is not None)):
        print("   %-8s %s"%(what,"ok" if ok else "MISSING"))
    if e.map is None or e.rgb is None or e.depth is None or e.pose() is None:
        print("ABORT: prerequisites missing"); return 2

    wps=e.waypoints()
    if not wps: print("ABORT: no safe waypoints"); return 2
    p=e.pose()
    # start from the waypoint nearest the robot, keeping the serpentine order after it
    i0=min(range(len(wps)),key=lambda i:math.hypot(wps[i][0]-p[0],wps[i][1]-p[1]))
    wps=wps[i0:]+wps[:i0]
    budget=float(sys.argv[1]) if len(sys.argv)>1 else 1500.0
    t_start=time.time()

    print("\nscanning from the starting position first")
    e.scan_here()

    visited=abandoned=0
    for n,(x,y) in enumerate(wps):
        if time.time()-t_start>budget: print("\ntime budget reached"); break
        if e.batt is not None and e.batt<BATT_ABORT:
            print("\nBATTERY %.2f V - stopping"%e.batt); break
        pr=e.pose()
        if pr and math.hypot(x-pr[0],y-pr[1])<0.35: continue
        print("\n[%d/%d] waypoint (%+.2f,%+.2f)  batt %s"
              %(n+1,len(wps),x,y,"%.2f V"%e.batt if e.batt else "?"))
        ok,why=e.route_ok(x,y)
        if not ok:
            print("       ABANDONED: %s"%why); abandoned+=1; continue
        print("       %s"%why)
        r=e.goto(x,y)
        print("       %s"%r)
        if r!="SUCCEEDED":
            abandoned+=1
            if r in ("STUCK","TIMEOUT"): e.stop()
            continue
        visited+=1
        e.scan_here()

    e.stop(); e.consolidate(); e.publish()
    print("\n=== mission over: %d waypoints reached, %d abandoned, %.0f s ==="
          %(visited,abandoned,time.time()-t_start))
    conf=e.tracker.confirmed()
    print("=== %d confirmed cube(s), %d tentative ==="
          %(len(conf),len(e.tracker.tracks)-len(conf)))
    for t in conf:
        print("   cube %d: map (%+.2f, %+.2f) height %.3f m, %d sighting(s), conf %.2f"
              %(t.id,t.x,t.y,t.z,t.n,t.conf))
    for t in e.tracker.tracks:
        if t.state!="confirmed":
            print("   (tentative) #%d (%+.2f,%+.2f) %d sighting(s) from %d viewpoint(s)"
                  %(t.id,t.x,t.y,t.n,len(t.views)))
    rclpy.shutdown(); return 0

if __name__=="__main__": sys.exit(main())
