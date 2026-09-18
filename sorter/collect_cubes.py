#!/usr/bin/env python3
"""Drive to each detected cube, pick it up, carry it to the drop zone and PLACE it.

Marks every spot on the map as it goes:
    BLUE  - cube collected and placed in the zone
    RED   - nothing there: the detection was a false positive
    AMBER - a cube is there but the grasp failed (out of reach, bad angle, dropped)
    GREEN - not attempted yet

THE CENTRAL CONSTRAINT
----------------------
The depth camera is bolted to `arm4`. Its transform is a STATIC one computed for the arm
parked at NAV_POSE, so the instant the arm moves to grasp, every depth point the costmap
receives is wrong. Navigation and grasping therefore must never overlap:

    navigate (arm parked, TF valid)
      -> stop, kill nav_camera_tf
      -> grasp (arm moves freely, costmap is garbage but the robot is stationary)
      -> return arm to NAV_POSE *with the claw closed on the cube*
      -> restart nav_camera_tf, CLEAR the costmaps, only then drive

Carrying at the NAV_POSE angles works because the gripper joint is beyond `arm5` while the
camera hangs off `arm4` - closing the claw does not move the camera one millimetre, so the
calibrated transform still holds with a cube in the jaws.
"""
import os, sys, math, time, subprocess, re, numpy as np, rclpy, cv2
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
from arm_msgs.msg import ArmJoints, ArmJoint
from cv_bridge import CvBridge
import tf2_ros, transforms3d as tfs
from ultralytics import YOLO

CUBES_FILE = os.environ.get("CUBES","/home/jetson/maps/cubes_confirmed.txt")
DROP       = (float(os.environ.get("DROP_X","-0.075")), float(os.environ.get("DROP_Y","-1.175")))
LIMIT      = int(os.environ.get("LIMIT","0"))          # 0 = all
MODEL      = "/home/jetson/cuboid_best_baseline.pt"
K          = [477.57421875,0.0,319.3820495605469,0.0,477.55718994140625,238.64108276367188,0.0,0.0,1.0]
CONF       = 0.25

APPROACH_R = 0.42        # stand this far from a cube to look at it (Nav2 goal)
GRASP_R    = 0.155       # Final stand-off. Two INDEPENDENT limits bracket this tightly:
                         #   arm envelope at grip height : 0.15 - 0.245 m
                         #   grasp camera at joint2=120  : sees floor from 0.16 m
                         # so the cube must be far enough for the ARM'S OWN detector to see
                         # it, and near enough for the arm to reach down to it. Evidence:
                         #   0.131 m -> grasp camera saw nothing (inside its blind zone)
                         #   0.138 m -> SUCCEEDED
                         #   0.140 m -> SUCCEEDED
                         #   0.157 m -> SUCCEEDED
                         #   0.171 m -> arm saw no cube
                         #   0.195 m -> IK rejected it, 81.6 mm out of envelope
                         # The nudge also undershoots by ~15 mm, so ask for slightly more.
REFINE_R   = 0.42        # Intermediate stop for re-measuring. Must be comfortably inside the
                         # camera window, whose NEAR limit is 0.30 m at joint2=150 - refining
                         # at 0.34 m sat on that edge and kept returning nothing.
REACH_MIN,REACH_MAX = 0.15, 0.245
PLACE_STANDOFF = 0.60    # Nav2 goal near the zone, outside the inflation of placed cubes
PLACE_R    = 0.20        # final open-loop distance from the zone centre
MATCH_R    = 0.40        # a detection this close to the expected spot is "the same cube"
# Navigate at joint2=150: camera pitched 30 deg down, floor visible 0.30-1.34 m ahead.
# The grasp ends at joint2=120 holding the cube, so to carry we must come back to 150 -
# but on 2026-09-13 doing that with an ArmJoints message dropped the cube, because that
# message carries joint6 and re-commanding the gripper makes the servo re-seat and let go.
# Fix (user's): drive joint2 ALONE over /arm_joint, which has no joint6 field at all, so
# the grip is never touched. Only joint2 differs between the two poses.
NAV_POSE   = [90,150,0,0,90]
CARRY_J2   = 150
# The held cube sits right under the camera and would otherwise be marked as an obstacle
# permanently in front of the robot. Ignore the nearest slice of the depth view - the
# "bottom half" - and rely on what is beyond it.
# The near-range exclusion is PHASE-DEPENDENT, and getting that wrong cost a day:
#   while CARRYING  : the cube in the jaws sits right under the camera and would be marked as
#                     a permanent obstacle ahead, so the robot circles one spot forever.
#   while NAVIGATING: the same exclusion blinds the robot in its last ~20 cm of approach, and
#                     since the lidar cannot see a 3 cm cuboid at all, it drives straight over
#                     them. That is the hit-and-run of 2026-09-15.
# Both values are correct for their own phase and wrong for the other, so it must be switched
# at the phase boundary rather than set once globally.
NEAR_IGNORE_CARRY = 0.35   # ignore anything this close while a cube is held
NEAR_IGNORE_NAV   = 0.0    # see everything the camera can while driving
NEAR_IGNORE_GRASP = 5.0    # mark NOTHING from depth while the arm is moving.
# Measured 2026-09-15: a cube in the gripper returns ZERO depth points in either the
# observation pose (nearest return 0.297 m) or the carry pose (0.452 m) - it sits inside the
# sensor's blind zone, which is why it appears black. So the held cube was never the phantom
# obstacle. The real hazard is the GRASP itself: the arm sweeps through the camera's view at
# close range while the costmap is still using the last STATIC camera transform, which never
# expires just because its publisher stopped. Those points are marked at wrong positions, and
# raytrace_min_range 0.45 then makes them PERMANENT because nothing closer is ever cleared.
GRIP_OPEN, GRIP_HOLD = 30, 142
GOAL_TIMEOUT = 150.0
STUCK_DIST, STUCK_YAW, STUCK_WIN = 0.04, 0.12, 16.0
BATT_ABORT = 10.80
PAT=re.compile(r"map\s+\(([-+0-9.]+),\s*([-+0-9.]+)\)\s+height\s+([0-9.]+)")

class Collector(Node):
    def __init__(self):
        super().__init__("collect_cubes")
        lat=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.rgb=self.depth=None; self.rgb_stamp=None; self.batt=None; self.scan=None; self.map=None
        self.create_subscription(OccupancyGrid,"/map",lambda m:setattr(self,"map",m),lat)
        self.b=CvBridge()
        self.create_subscription(Image,"/camera/color/image_raw",self._cb_rgb,1)
        self.create_subscription(Image,"/camera/depth/image_raw",lambda m:setattr(self,"depth",self.b.imgmsg_to_cv2(m,"32FC1").astype(np.float32)),1)
        self.create_subscription(LaserScan,"/scan",lambda m:setattr(self,"scan",m),rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Float32,"/battery",lambda m:setattr(self,"batt",float(m.data)),10)
        self.mk=self.create_publisher(MarkerArray,"/cube_markers",lat)
        self.cmd=self.create_publisher(Twist,"/cmd_vel",10)
        self.arm=self.create_publisher(ArmJoints,"arm6_joints",10)
        self.arm1=self.create_publisher(ArmJoint,"arm_joint",10)
        self.nav=ActionClient(self,NavigateToPose,"navigate_to_pose")
        self.plan=ActionClient(self,ComputePathToPose,"compute_path_to_pose")
        self.clr_l=self.create_client(ClearEntireCostmap,"/local_costmap/clear_entirely_local_costmap")
        self.clr_g=self.create_client(ClearEntireCostmap,"/global_costmap/clear_entirely_global_costmap")
        self.tfbuf=tf2_ros.Buffer(); tf2_ros.TransformListener(self.tfbuf,self)
        self.model=YOLO(MODEL)
        self.cubes=[]

    # ---------- basics ----------
    def _cb_rgb(self,m):
        # Keep the CAPTURE TIME with the image. Transforming a detection with the LATEST
        # transform instead of the one for the shutter instant silently bakes every bit of
        # motion between capture and processing into the cube's position.
        self.rgb=self.b.imgmsg_to_cv2(m,"bgr8")
        self.rgb_stamp=m.header.stamp

    def spin(self,s):
        t=self.get_clock().now()
        while (self.get_clock().now()-t).nanoseconds<s*1e9: rclpy.spin_once(self,timeout_sec=0.05)
    def pose(self,retries=1):
        """Robot pose in the map frame, retrying through AMCL's intermittent dropouts.

        AMCL loses map->odom about 2.5% of the time under load. Callers used to assume this
        could not fail; on 2026-09-13 one unguarded call crashed the mission while the robot
        was holding a cube. Retrying costs milliseconds in the normal case."""
        for i in range(max(1,retries)):
            try:
                t=self.tfbuf.lookup_transform("map","base_footprint",rclpy.time.Time()).transform
                q=t.rotation
                return (t.translation.x,t.translation.y,
                        math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2)))
            except Exception:
                if i+1<retries:
                    for _ in range(6): rclpy.spin_once(self,timeout_sec=0.08)
        return None

    def pose_sure(self,timeout=8.0):
        """Pose that waits. Use wherever None would abort or crash the mission."""
        t0=time.time()
        while time.time()-t0<timeout:
            p=self.pose()
            if p is not None: return p
            for _ in range(6): rclpy.spin_once(self,timeout_sec=0.08)
        return None
    def localisation_ok(self,max_miss=0.15):
        """AMCL can stay `active` and keep publishing /amcl_pose while its map->odom
        transform has gone stale - on 2026-09-13 that sent the robot to an approach pose
        computed in a frame that no longer existed, and it ended up facing nothing.
        A pose that publishes is not a robot that is localised; check the SCAN against
        the MAP, which is the only statement that means anything."""
        if self.pose() is None:
            print("   localisation: no map->base_footprint transform at all"); return False
        if self.map is None or self.scan is None: return True
        m=self.map
        g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
        walls=np.argwhere(g>=65)
        if not len(walls): return True
        res=m.info.resolution
        wx=m.info.origin.position.x+(walls[:,1]+0.5)*res
        wy=m.info.origin.position.y+(walls[:,0]+0.5)*res
        s=self.scan
        try:
            tr=self.tfbuf.lookup_transform("map",s.header.frame_id,rclpy.time.Time()).transform
        except Exception:
            print("   localisation: cannot transform the scan into the map"); return False
        q=tr.rotation
        T=tfs.affines.compose([tr.translation.x,tr.translation.y,tr.translation.z],
                              tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z]),[1,1,1])
        r=np.array(s.ranges); ang=s.angle_min+np.arange(len(r))*s.angle_increment
        ok=np.isfinite(r)&(r>s.range_min)&(r<s.range_max)
        if ok.sum()<40: return True
        pts=np.stack([r[ok]*np.cos(ang[ok]),r[ok]*np.sin(ang[ok]),
                      np.zeros(ok.sum()),np.ones(ok.sum())])
        P=(T@pts).T[:,:2]
        miss=float(np.median([np.min(np.hypot(wx-p[0],wy-p[1])) for p in P]))
        good = miss<=max_miss
        print("   localisation: scan-to-map median miss %.3f m -> %s"%(miss,"ok" if good else "BAD"))
        return good

    def relocalise(self):
        print("   re-localising (global scatter + in-place rotation) ...")
        r=sh("cd /home/jetson/calib && ROS_DOMAIN_ID=30 python3 localise.py",timeout=300)
        for l in [l for l in r.stdout.splitlines() if "median miss" in l or "LOCALISED" in l
                  or "NOT CONVERGED" in l]:
            print("      | %s"%l.strip())
        self.spin(1.0)
        return self.localisation_ok()

    def stop(self):
        t=Twist()
        for _ in range(12): self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)
    def set_near_ignore(self, metres, why=""):
        """Switch the costmap's near-range exclusion, and VERIFY it took.

        A silent failure here is dangerous in both directions - either the robot chases a
        phantom obstacle it is carrying, or it goes blind to the cubes in front of it - so
        the value is read back rather than assumed.
        """
        ok=True
        for cm in ("local_costmap","global_costmap"):
            sh("ROS_DOMAIN_ID=30 ros2 param set /%s/%s "
               "obstacle_layer.pointcloud.obstacle_min_range %.3f"%(cm,cm,metres), timeout=40)
        r=sh("ROS_DOMAIN_ID=30 ros2 param get /local_costmap/local_costmap "
             "obstacle_layer.pointcloud.obstacle_min_range", timeout=40)
        got=r.stdout.strip().split()[-1] if r.stdout.strip() else "?"
        try: ok = abs(float(got)-metres) < 1e-3
        except ValueError: ok=False
        print("   near-range exclusion -> %.2f m %s [%s]"
              %(metres, why, "ok" if ok else "FAILED, read back %s"%got))
        # stale marks from the previous phase must not survive the switch
        self.clear_costmaps()
        return ok

    def clear_costmaps(self):
        for c in (self.clr_l,self.clr_g):
            if c.wait_for_service(timeout_sec=4.0):
                f=c.call_async(ClearEntireCostmap.Request())
                rclpy.spin_until_future_complete(self,f,timeout_sec=6)
    def send_arm(self,j,grip,ms=1800):
        m=ArmJoints()
        m.joint1,m.joint2,m.joint3,m.joint4,m.joint5=[int(v) for v in j[:5]]
        m.joint6=int(grip); m.time=int(ms)
        t=time.time()
        while self.arm.get_subscription_count()==0 and time.time()-t<6:
            rclpy.spin_once(self,timeout_sec=0.05)
        for _ in range(3):
            self.arm.publish(m); rclpy.spin_once(self,timeout_sec=0.03); time.sleep(0.05)
        time.sleep(ms/1000.0+0.3)

    def move_joint(self,jid,angle,ms=2000):
        """Move ONE joint. Used to return to the navigation pose while carrying: the
        single-joint message has no joint6 field, so the gripper is never re-commanded
        and the cube cannot be released."""
        m=ArmJoint(); m.id=int(jid); m.joint=int(angle); m.time=int(ms)
        t=time.time()
        while self.arm1.get_subscription_count()==0 and time.time()-t<6:
            rclpy.spin_once(self,timeout_sec=0.05)
        for _ in range(3):
            self.arm1.publish(m); rclpy.spin_once(self,timeout_sec=0.03); time.sleep(0.05)
        time.sleep(ms/1000.0+0.4)

    # ---------- markers ----------
    def publish(self):
        COL={"pending":(0.1,1.0,0.2),"picked":(0.15,0.4,1.0),
             "false":(1.0,0.1,0.1),"failed":(1.0,0.72,0.0)}
        ma=MarkerArray()
        for i,c in enumerate(self.cubes):
            r,g,b=COL[c["state"]]
            m=Marker(); m.header.frame_id="map"; m.header.stamp=self.get_clock().now().to_msg()
            m.ns="cubes"; m.id=i; m.type=Marker.CUBE; m.action=Marker.ADD
            m.pose.position.x=c["x"]; m.pose.position.y=c["y"]; m.pose.position.z=0.03
            m.pose.orientation.w=1.0
            m.scale.x=m.scale.y=m.scale.z=0.06
            m.color.r,m.color.g,m.color.b,m.color.a=r,g,b,0.95
            ma.markers.append(m)
            t=Marker(); t.header=m.header; t.ns="cube_labels"; t.id=1000+i
            t.type=Marker.TEXT_VIEW_FACING; t.action=Marker.ADD
            t.pose.position.x=c["x"]; t.pose.position.y=c["y"]; t.pose.position.z=0.20
            t.pose.orientation.w=1.0; t.scale.z=0.08
            t.color.r=t.color.g=t.color.b=1.0; t.color.a=1.0
            t.text="#%d %s"%(i+1,c["state"])
            ma.markers.append(t)
        self.mk.publish(ma)

    # ---------- perception ----------
    def detect(self):
        """Returns a list of detections, or None if the MAP TRANSFORM was unavailable.

        The distinction matters enormously. AMCL drops out about 2.5% of the time under
        YOLO load (measured: 9 losses in 347 lookups over 60 s), because the lidar stamps
        scans ~117 ms in the future and its message filter discards them. If a dropout is
        reported as "no detections", the mission concludes the cube is a FALSE POSITIVE and
        permanently marks a real cube red - which is exactly what happened three times on
        2026-09-13. "I could not look" is not "there is nothing there"."""
        if self.rgb is None or self.depth is None: return None
        rgb=self.rgb.copy(); dep=self.depth.copy()
        stamp=self.rgb_stamp
        tr=None; used_latest=False
        for _ in range(15):
            try:
                if stamp is not None:
                    tr=self.tfbuf.lookup_transform("map","camera_color_optical_frame",
                                                   rclpy.time.Time.from_msg(stamp)).transform
                else:
                    tr=self.tfbuf.lookup_transform("map","camera_color_optical_frame",
                                                   rclpy.time.Time()).transform
                break
            except Exception:
                rclpy.spin_once(self,timeout_sec=0.1)
        if tr is None and stamp is not None:
            # the capture-time transform is genuinely unavailable (buffer gap, or the camera
            # stamps ahead of ROS time as the lidar does). Say so rather than pretending.
            try:
                tr=self.tfbuf.lookup_transform("map","camera_color_optical_frame",
                                               rclpy.time.Time()).transform
                used_latest=True
            except Exception:
                return None
        if tr is None: return None
        if used_latest and not getattr(self,"_warned_latest_tf",False):
            print("       NOTE: no transform at the image timestamp; using the latest instead"
                  " - cube positions will carry any motion since capture")
            self._warned_latest_tf=True
        q=tr.rotation
        T=tfs.affines.compose([tr.translation.x,tr.translation.y,tr.translation.z],
                              tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z]),[1,1,1])
        res=self.model.predict(rgb,imgsz=640,conf=CONF,verbose=False)[0]
        fx,fy,cx0,cy0=K[0],K[4],K[2],K[5]
        out=[]
        for bx in res.boxes:
            x1,y1,x2,y2=[int(v) for v in bx.xyxy[0]]
            mx1,my1=x1+(x2-x1)//4, y1+(y2-y1)//4
            mx2,my2=x2-(x2-x1)//4, y2-(y2-y1)//4
            patch=dep[max(0,my1):my2, max(0,mx1):mx2]
            patch=patch[np.isfinite(patch)&(patch>0)]
            if patch.size<10: continue
            z=float(np.median(patch))*0.001
            if not (0.25<z<2.0): continue
            u=(x1+x2)/2.0
            vb=float(y2)
            dir_cam=np.array([(u-cx0)/fx,(vb-cy0)/fy,1.0,0.0])
            Cw=(T@np.array([0.0,0.0,0.0,1.0]))[:3]; dw=(T@dir_cam)[:3]
            if abs(dw[2])<1e-6: continue
            t=-Cw[2]/dw[2]
            if t<=0: continue
            F=Cw+t*dw
            out.append((float(F[0]),float(F[1]),float(bx.conf[0])))
        return out

    # ---------- motion ----------
    def goto(self,x,y,yaw=None):
        if not self.nav.wait_for_server(timeout_sec=10.0): return "NO SERVER"
        g=NavigateToPose.Goal()
        g.pose.header.frame_id="map"; g.pose.header.stamp=self.get_clock().now().to_msg()
        g.pose.pose.position.x=float(x); g.pose.pose.position.y=float(y)
        if yaw is None: g.pose.pose.orientation.w=1.0
        else:
            g.pose.pose.orientation.z=math.sin(yaw/2.0); g.pose.pose.orientation.w=math.cos(yaw/2.0)
        f=self.nav.send_goal_async(g); rclpy.spin_until_future_complete(self,f,timeout_sec=12)
        h=f.result()
        if h is None or not h.accepted: return "REJECTED"
        rf=h.get_result_async(); t0=time.time(); last=self.pose(); last_t=time.time()
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
        if not rf.done(): h.cancel_goal_async(); self.stop(); return "TIMEOUT"
        return {4:"SUCCEEDED",5:"CANCELED",6:"ABORTED"}.get(rf.result().status,"?")

    def front_clear(self,dist):
        """Lidar check before any open-loop move: is the way ahead actually empty?"""
        s=self.scan
        if s is None: return True
        r=np.array(s.ranges); ang=s.angle_min+np.arange(len(r))*s.angle_increment
        ok=np.isfinite(r)&(r>s.range_min)&(r<s.range_max)&(np.abs(ang)<math.radians(25))
        if not ok.any(): return True
        return float(r[ok].min())>dist+0.18

    def nudge(self,forward=0.0,turn=0.0,v=0.07,w=0.4):
        """Short odometry-closed move. Used only for the last few cm, where Nav2's goal
        tolerance is coarser than the arm's reach envelope."""
        if abs(turn)>1e-3:
            p0=self.pose_sure()
            if p0 is None:
                print("       no pose for the turn - skipping"); return False
            t=Twist(); t.angular.z=math.copysign(w,turn); t0=time.time()
            while time.time()-t0<abs(turn)/w+1.5:
                p=self.pose()
                if p and abs((p[2]-p0[2]+math.pi)%(2*math.pi)-math.pi)>=abs(turn): break
                self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)
            self.stop(); self.spin(0.4)
        if abs(forward)>1e-3:
            if forward>0 and not self.front_clear(forward):
                print("       forward path not clear on lidar - skipping nudge"); return False
            p0=self.pose_sure()
            if p0 is None:
                # Silent failure here cost several grasps on 2026-09-13: nudge() read the
                # pose once, got a transform dropout, and returned False having moved
                # nothing and printed nothing - so the robot "approached" to 0.608 m and
                # the arm then found no cube.
                print("       no pose for the forward move - skipping"); return False
            t=Twist(); t.linear.x=math.copysign(v,forward); t0=time.time()
            while time.time()-t0<abs(forward)/v+2.0:
                p=self.pose()
                if p and math.hypot(p[0]-p0[0],p[1]-p0[1])>=abs(forward): break
                self.cmd.publish(t); rclpy.spin_once(self,timeout_sec=0.02)
            self.stop(); self.spin(0.4)
            pe=self.pose_sure(timeout=3.0)
            if pe is not None:
                got=math.hypot(pe[0]-p0[0],pe[1]-p0[1])
                if got < abs(forward)-0.05:
                    print("       WARNING: asked for %.3f m, achieved %.3f m"%(abs(forward),got))
        return True

def sh(cmd,timeout=240):
    return subprocess.run(cmd,shell=True,capture_output=True,text=True,timeout=timeout)

def tf_publisher(action):
    """Start/stop the static camera transform publisher.

    Must use start_new_session so the child OUTLIVES this call - `subprocess.run` with a
    trailing & does NOT survive, which on 2026-09-13 left the robot driving with no camera
    transform at all after a failed grasp.
    """
    if action=="stop":
        # bracket trick: the pattern text itself never matches the pattern, so this
        # cannot kill the shell running it (a trap hit repeatedly in this project)
        sh("ps -eo pid,cmd | awk '/nav_camera_t[f]/ {print $1}' | xargs -r kill -9")
        time.sleep(2)
        return
    subprocess.Popen("cd /home/jetson/calib && NAV_J2=%d python3 -u nav_camera_tf.py"%CARRY_J2,
                     shell=True, start_new_session=True,
                     stdout=open("/tmp/navcam.log","w"), stderr=subprocess.STDOUT)
    time.sleep(26)

def load():
    out=[]
    for l in open(CUBES_FILE):
        m=PAT.search(l)
        if m and float(m.group(3))<=0.10:
            out.append(dict(x=float(m.group(1)),y=float(m.group(2)),state="pending"))
    return out

def main():
    rclpy.init(); e=Collector()
    e.cubes=load()
    print("%d floor cube(s) to collect; drop zone at (%+.3f,%+.3f)"%(len(e.cubes),DROP[0],DROP[1]))
    # A run killed mid-carry leaves the exclusion at the CARRY value, which would blind the
    # next run during its approach. Always begin from a known state.
    e.set_near_ignore(NEAR_IGNORE_NAV, "(mission start, jaws empty)")
    t0=time.time()
    while time.time()-t0<40 and (e.rgb is None or e.depth is None or e.pose() is None):
        rclpy.spin_once(e,timeout_sec=0.2)
    if e.pose() is None: print("ABORT: no pose"); return 2
    e.publish()

    try:
        return run_mission(e)
    finally:
        # However this ends - success, exception, Ctrl-C - the costmap must not be left
        # ignoring near obstacles, or the next thing to drive is blind.
        try: e.set_near_ignore(NEAR_IGNORE_NAV, "(mission over)")
        except Exception: pass
        rclpy.shutdown()

def run_mission(e):
    order=sorted(range(len(e.cubes)),
                 key=lambda i:math.hypot(e.cubes[i]["x"]-e.pose()[0],e.cubes[i]["y"]-e.pose()[1]))
    if LIMIT: order=order[:LIMIT]
    done=false_=failed=0
    for n,i in enumerate(order,1):
        c=e.cubes[i]
        if e.batt is not None and e.batt<BATT_ABORT:
            print("\nBATTERY %.2f V - stopping"%e.batt); break
        print("\n=== [%d/%d] cube #%d at (%+.2f,%+.2f)  batt %s ==="
              %(n,len(order),i+1,c["x"],c["y"],"%.2f V"%e.batt if e.batt else "?"))
        if not e.localisation_ok():
            if not e.relocalise():
                print("   ABORT: cannot localise - every map coordinate is meaningless"); break

        # 1. stand back from it and look
        p=e.pose_sure()
        if p is None:
            print("   lost the map transform before approaching - leaving as pending"); continue
        ang=math.atan2(p[1]-c["y"],p[0]-c["x"])
        ax,ay=c["x"]+APPROACH_R*math.cos(ang), c["y"]+APPROACH_R*math.sin(ang)
        want=ang+math.pi
        gap=math.hypot(ax-p[0],ay-p[1])
        if gap<0.25:
            # Already standing at the approach pose. Nav2's path-follower ABORTS here -
            # there is no path to follow, only a rotation, and it cannot make progress
            # toward a goal it is already on. Turn in place instead.
            turn=(want-p[2]+math.pi)%(2*math.pi)-math.pi
            print("   already %.3f m from the approach pose - turning %.0f deg in place"
                  %(gap,math.degrees(turn)))
            e.nudge(turn=turn); r="SUCCEEDED"
        else:
            r=e.goto(ax,ay,yaw=want)
            print("   approach: %s"%r)
            if r in ("ABORTED","TIMEOUT","STUCK"):
                # Nav2 gave up. If we are near enough, finish the job open-loop rather
                # than abandoning a cube the robot is standing next to.
                p2=e.pose()
                if p2 and math.hypot(ax-p2[0],ay-p2[1])<0.55:
                    turn=(want-p2[2]+math.pi)%(2*math.pi)-math.pi
                    print("   within %.2f m anyway - aligning open-loop"
                          %math.hypot(ax-p2[0],ay-p2[1]))
                    e.nudge(turn=turn); r="SUCCEEDED"
        if r!="SUCCEEDED":
            print("   could not reach it - leaving as pending"); continue
        e.spin(1.5)

        # 2. is it actually there?
        seen=[]; looks=0
        for _ in range(8):
            dd=e.detect()
            if dd is None:            # transform dropout - retry, do NOT count as a look
                e.spin(0.5); continue
            looks+=1
            seen+= [x for x in dd if math.hypot(x[0]-c["x"],x[1]-c["y"])<MATCH_R]
            e.spin(0.5)
            if len(seen)>=3: break
        if looks<3:
            print("   could not get a reliable look (map transform kept dropping) -"
                  " leaving as pending rather than calling it false")
            continue
        if not seen:
            print("   NOTHING THERE in %d good look(s) -> false positive"%looks)
            c["state"]="false"; false_+=1; e.publish(); continue
        cx=float(np.median([s[0] for s in seen])); cy=float(np.median([s[1] for s in seen]))
        print("   confirmed at (%+.3f,%+.3f) from %d sighting(s)"%(cx,cy,len(seen)))
        c["x"],c["y"]=cx,cy; e.publish()

        # 3. close in, in TWO stages.
        #    The navigation camera only sees floor from 0.30 m out, so once the robot is at
        #    grasp distance the cube is too close to be re-measured - the old single-stage
        #    version always reported "no longer visible", which was a property of the
        #    geometry, not of the cube. So refine at REFINE_R, where it is still visible,
        #    and only then make the short blind move to GRASP_R.
        p=e.pose_sure()
        if p is None:
            print("   lost the map transform before lining up - leaving as pending"); continue
        d=math.hypot(cx-p[0],cy-p[1]); bearing=math.atan2(cy-p[1],cx-p[0])
        turn=(bearing-p[2]+math.pi)%(2*math.pi)-math.pi
        print("   cube is %.3f m away, %.0f deg off the nose"%(d,math.degrees(turn)))
        e.nudge(turn=turn)
        if d-REFINE_R>0.04:
            e.nudge(forward=d-REFINE_R)
            e.spin(1.2)
            again=[]
            for _ in range(6):
                dd=e.detect()
                if dd is None: e.spin(0.4); continue
                again += [x for x in dd if math.hypot(x[0]-cx,x[1]-cy)<MATCH_R]
                e.spin(0.4)
                if len(again)>=3: break
            if again:
                nx=float(np.median([s[0] for s in again])); ny=float(np.median([s[1] for s in again]))
                print("   refined at %.3f m: cube now (%+.3f,%+.3f), moved %.3f m"
                      %(REFINE_R,nx,ny,math.hypot(nx-cx,ny-cy)))
                cx,cy=nx,ny; c["x"],c["y"]=cx,cy; e.publish()
                pp=e.pose()
                b2=math.atan2(cy-pp[1],cx-pp[0])
                e.nudge(turn=(b2-pp[2]+math.pi)%(2*math.pi)-math.pi)
            else:
                print("   could not re-measure at %.2f m - using the earlier estimate"%REFINE_R)
        p=e.pose_sure() or e.pose()
        if p is None: print("   no pose for the final move - skipping"); continue
        d3=math.hypot(cx-p[0],cy-p[1])
        if d3-GRASP_R>0.03: e.nudge(forward=d3-GRASP_R)
        p=e.pose_sure() or (cx+GRASP_R,cy,0.0)
        d2=math.hypot(cx-p[0],cy-p[1])
        print("   final stand-off %.3f m (arm reaches %.2f-%.2f m at grip height)"
              %(d2,REACH_MIN,REACH_MAX))

        # 4. grasp - the arm moves, so depth marking must be silenced AND the TF taken down
        e.set_near_ignore(NEAR_IGNORE_GRASP, "(arm moving - trust no depth)")
        tf_publisher("stop")
        g=sh("cd /home/jetson/calib && ROS_DOMAIN_ID=30 python3 stationary_pickup.py",timeout=300)
        tail=[l for l in g.stdout.splitlines() if l.strip()][-3:]
        for l in tail: print("      | %s"%l)
        ok = (g.returncode==0)
        if not ok:
            print("   GRASP FAILED")
            c["state"]="failed"; failed+=1; e.publish()
            e.set_near_ignore(NEAR_IGNORE_NAV, "(nothing held)")
            e.send_arm(NAV_POSE,GRIP_OPEN,1800)
            tf_publisher("start"); e.clear_costmaps(); continue

        # 5. back to the navigation pose WITHOUT touching the gripper
        print("   grasped; raising joint2 %d -> %d via /arm_joint (gripper untouched)"
              %(120,CARRY_J2))
        e.move_joint(2,CARRY_J2,2200)
        e.set_near_ignore(NEAR_IGNORE_CARRY, "(carrying a cube)")
        tf_publisher("start"); e.clear_costmaps()

        # 6. to the drop zone
        p=e.pose_sure()
        if p is None:
            print("   lost the map transform while holding a cube - placing it here instead")
            sh("cd /home/jetson/calib && ROS_DOMAIN_ID=30 python3 place_cube.py",timeout=180)
            e.set_near_ignore(NEAR_IGNORE_NAV, "(cube released)")
            c["state"]="failed"; failed+=1; e.publish(); continue
        ang=math.atan2(p[1]-DROP[1],p[0]-DROP[0])
        sx,sy=DROP[0]+PLACE_STANDOFF*math.cos(ang), DROP[1]+PLACE_STANDOFF*math.sin(ang)
        want=ang+math.pi
        gap=math.hypot(sx-p[0],sy-p[1])
        if gap<0.25:
            turn=(want-p[2]+math.pi)%(2*math.pi)-math.pi
            print("   already at the zone stand-off - turning %.0f deg in place"%math.degrees(turn))
            e.nudge(turn=turn); r="SUCCEEDED"
        else:
            r=e.goto(sx,sy,yaw=want)
            print("   to drop zone: %s"%r)
            if r in ("ABORTED","TIMEOUT","STUCK"):
                p2=e.pose_sure()
                if p2 and math.hypot(sx-p2[0],sy-p2[1])<0.55:
                    print("   near enough - aligning open-loop")
                    e.nudge(turn=(want-p2[2]+math.pi)%(2*math.pi)-math.pi); r="SUCCEEDED"
        if r!="SUCCEEDED":
            print("   could not reach the zone while holding a cube - placing it where it stands")
            sh("cd /home/jetson/calib && ROS_DOMAIN_ID=30 python3 place_cube.py",timeout=180)
            e.set_near_ignore(NEAR_IGNORE_NAV, "(cube released where it stood)")
            c["state"]="failed"; failed+=1; e.publish(); continue

        # final alignment onto the zone itself
        p=e.pose_sure()
        if p is None:
            print("   lost the map transform at the zone - placing from where it stands")
        else:
            bearing=math.atan2(DROP[1]-p[1],DROP[0]-p[0])
            e.nudge(turn=(bearing-p[2]+math.pi)%(2*math.pi)-math.pi)
            p=e.pose_sure()
            if p is not None:
                dz=math.hypot(DROP[0]-p[0],DROP[1]-p[1])
                if dz-PLACE_R>0.04: e.nudge(forward=dz-PLACE_R)

        # 7. place
        pl=sh("cd /home/jetson/calib && ROS_DOMAIN_ID=30 python3 place_cube.py",timeout=180)
        for l in [l for l in pl.stdout.splitlines() if l.strip()][-2:]: print("      | %s"%l)
        if pl.returncode!=0:
            print("   PLACE FAILED"); c["state"]="failed"; failed+=1
        else:
            c["state"]="picked"; done+=1
            print("   PLACED in the zone")
        e.set_near_ignore(NEAR_IGNORE_NAV, "(cube released)")
        e.publish()
        e.nudge(forward=-0.28)           # back off so the next approach is not blocked
        e.send_arm(NAV_POSE,GRIP_OPEN,1800)   # claw open again, ready for the next cube
        e.clear_costmaps()

    e.stop(); e.publish()
    print("\n=== collected %d, false positives %d, failed %d ==="%(done,false_,failed))
    for i,c in enumerate(e.cubes):
        print("   #%-2d (%+.2f,%+.2f)  %s"%(i+1,c["x"],c["y"],c["state"]))
    with open("/home/jetson/maps/collect_result.txt","w") as f:
        for i,c in enumerate(e.cubes):
            f.write("cube %d: map (%+.2f, %+.2f) height 0.030 m, 1 sighting(s), conf 1.00 %s\n"
                    %(i+1,c["x"],c["y"],c["state"]))
    return 0

if __name__=="__main__": sys.exit(main())
