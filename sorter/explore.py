#!/usr/bin/env python3
"""Autonomous frontier exploration for mapping a room with cartographer + Nav2.

A frontier is a known-free cell touching unknown space - the boundary of what the
robot has seen. Driving to frontiers repeatedly is what expands the map. Exploration
ends when no frontier cluster is large enough to be worth visiting.
"""
import sys, math, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import tf2_ros
import cv2

TIME_BUDGET   = float(sys.argv[1]) if len(sys.argv)>1 else 480.0   # seconds
MIN_CLUSTER   = 4        # cells; smaller frontiers are noise
MIN_GOAL_DIST = 0.40     # m; ignore frontiers we are basically standing on
GOAL_TIMEOUT  = 180.0    # at 0.08 m/s a 3 m traverse TIMED OUT at 100 s
STUCK_DIST    = 0.04     # m of progress required...
STUCK_WIN     = 16.0     # ...within this many seconds. Must exceed Nav2's own
                         # recovery behaviours, which make no goal-ward progress
BATT_ABORT    = 10.65    # V; the STM32 alarm fires at 10.5 and cannot be silenced
FREE_MAX      = 55       # there is an UNCERTAIN band (45-55) between free and unknown;
                         # at 45 only 6 frontier cells exist, at 55 there are 253
OCC_MIN       = 60
CLEARANCE_M   = 0.05     # barely any: frontiers hug walls. PULL_BACK_M keeps the GOAL clear,
                         # and Nav2's own costmap decides what is actually reachable
PULL_BACK_M   = 0.45     # stand off from the frontier: the robot cannot fit tight gaps,
                         # and the lidar sees past the goal anyway

class Explorer(Node):
    def __init__(self):
        super().__init__("explorer")
        self.map=None
        self.create_subscription(OccupancyGrid,"/map",self.cb_map,1)
        self.ac=ActionClient(self,NavigateToPose,"navigate_to_pose")
        self.tfbuf=tf2_ros.Buffer(); self.tfl=tf2_ros.TransformListener(self.tfbuf,self)
        self.vel=self.create_publisher(Twist,"/cmd_vel",1)
        self.blacklist=[]
        self.batt=None
        self.create_subscription(Float32,"/battery",self.cb_batt,1)
    def cb_batt(self,m): self.batt=float(m.data)
    def cb_map(self,m): self.map=m
    def spin(self,s=0.4):
        t0=time.time()
        while time.time()-t0<s: rclpy.spin_once(self,timeout_sec=0.05)
    def robot_xy(self):
        try:
            t=self.tfbuf.lookup_transform("map","base_footprint",rclpy.time.Time())
            return t.transform.translation.x,t.transform.translation.y
        except Exception:
            return None
    def stop(self):
        for _ in range(4): self.vel.publish(Twist()); time.sleep(0.05)
    def back_off(self, secs=2.5):
        """Reverse out of whatever we drove into (a bean bag traps rather than blocks)."""
        t=Twist(); t.linear.x=-0.08; t0=time.time()
        while time.time()-t0<secs:
            self.vel.publish(t); rclpy.spin_once(self,timeout_sec=0.05); time.sleep(0.05)
        self.stop()
        t=Twist(); t.angular.z=0.6; t0=time.time()
        while time.time()-t0<1.5:
            self.vel.publish(t); rclpy.spin_once(self,timeout_sec=0.05); time.sleep(0.05)
        self.stop()

    def frontiers(self):
        m=self.map
        if m is None: return [],None
        g0=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
        # cartographer CROPS the grid to the mapped region, so cells beyond the edge are
        # absent rather than unknown. Pad with unknown or edge frontiers are invisible.
        g=np.pad(g0,1,constant_values=-1)
        free=(g>=0)&(g<FREE_MAX)
        unknown=(g<0)
        occupied=(g>=OCC_MIN)
        # a frontier cell is free and touches unknown
        unk_d=cv2.dilate(unknown.astype(np.uint8),np.ones((3,3),np.uint8))
        front=(free&(unk_d>0)).astype(np.uint8)
        # keep clear of obstacles
        r=int(round(CLEARANCE_M/m.info.resolution))
        occ_d=cv2.dilate(occupied.astype(np.uint8),np.ones((2*r+1,2*r+1),np.uint8))
        front[occ_d>0]=0
        n,lab,st,cen=cv2.connectedComponentsWithStats(front,8)
        out=[]
        for i in range(1,n):
            if st[i,cv2.CC_STAT_AREA]<MIN_CLUSTER: continue
            cx,cy=cen[i]
            wx=m.info.origin.position.x+(cx-1+0.5)*m.info.resolution   # -1 undoes the pad
            wy=m.info.origin.position.y+(cy-1+0.5)*m.info.resolution
            out.append((wx,wy,int(st[i,cv2.CC_STAT_AREA])))
        return out,g0

    def is_free(self,x,y,radius=0.15):
        """Refuse goals that are not in known-free space - a goal inside an obstacle or in
        unknown space is accepted by Nav2 and then never reached."""
        m=self.map
        if m is None: return True
        g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
        cx=int((x-m.info.origin.position.x)/m.info.resolution)
        cy=int((y-m.info.origin.position.y)/m.info.resolution)
        r=max(1,int(radius/m.info.resolution))
        y0,y1=max(0,cy-r),min(m.info.height,cy+r+1)
        x0,x1=max(0,cx-r),min(m.info.width,cx+r+1)
        if x0>=x1 or y0>=y1: return False
        if not (0<=cx<m.info.width and 0<=cy<m.info.height): return False
        patch=g[y0:y1,x0:x1]
        # Reject only what is genuinely unusable:
        #   - the patch contains an OBSTACLE (goal sits on something), or
        #   - the goal's own cell is unknown (goal is outside the mapped area).
        # Demanding the whole patch be free is wrong: a frontier stand-off legitimately
        # sits next to unknown space - that is what makes it a frontier. Requiring 50%
        # free rejected 5 valid goals in a row on 2026-09-12 and blacklisted them all,
        # exhausting the candidate list while the map stopped growing at 17.8 m2.
        if bool((patch>=OCC_MIN).any()): return False
        c=g[cy,cx]
        return bool(0<=c<FREE_MAX)

    def goto(self,x,y,timeout=GOAL_TIMEOUT):
        if not self.ac.wait_for_server(timeout_sec=10.0): return "NO SERVER"
        g=NavigateToPose.Goal()
        g.pose.header.frame_id="map"; g.pose.header.stamp=self.get_clock().now().to_msg()
        g.pose.pose.position.x=float(x); g.pose.pose.position.y=float(y)
        g.pose.pose.orientation.w=1.0
        fut=self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self,fut,timeout_sec=12.0)
        gh=fut.result()
        if gh is None or not gh.accepted: return "REJECTED"
        rf=gh.get_result_async(); t0=time.time()
        last=self.robot_xy(); last_t=time.time()
        while time.time()-t0<timeout:
            rclpy.spin_once(self,timeout_sec=0.3)
            if rf.done(): break
            now=self.robot_xy()
            if now and last:
                if math.hypot(now[0]-last[0],now[1]-last[1])>STUCK_DIST:
                    last, last_t = now, time.time()
                elif time.time()-last_t > STUCK_WIN:
                    gh.cancel_goal_async(); self.stop()
                    print("       STUCK - no progress for %.0fs, backing off"%STUCK_WIN)
                    self.back_off()
                    return "STUCK"
        if not rf.done():
            gh.cancel_goal_async(); self.stop(); return "TIMEOUT"
        return {4:"SUCCEEDED",5:"CANCELED",6:"ABORTED"}.get(rf.result().status,str(rf.result().status))

def main():
    rclpy.init(); e=Explorer()
    t0=time.time()
    while time.time()-t0<20 and e.map is None: rclpy.spin_once(e,timeout_sec=0.3)
    if e.map is None: print("no /map - is cartographer running?"); return
    print("exploration starting, budget %.0f s"%TIME_BUDGET)
    visited=0
    while time.time()-t0<TIME_BUDGET:
        e.spin(1.2)
        if e.batt is not None and e.batt<BATT_ABORT:
            print("  BATTERY %.2f V below %.2f V - stopping before the alarm fires"%(e.batt,BATT_ABORT))
            break
        fr,g=e.frontiers()
        rp=e.robot_xy()
        if rp is None: print("  no robot pose yet"); e.spin(1.0); continue
        known=int((np.array(e.map.data)>=0).sum())*e.map.info.resolution**2
        cand=[]; too_close=0; blacklisted=0
        for (x,y,a) in fr:
            d=math.hypot(x-rp[0],y-rp[1])
            if d<MIN_GOAL_DIST: too_close+=1; continue
            if any(math.hypot(x-bx,y-by)<0.4 for bx,by in e.blacklist): blacklisted+=1; continue
            # stand off: aim PULL_BACK_M short of the frontier, along the line from the robot
            ux,uy=(x-rp[0])/d,(y-rp[1])/d
            sx,sy=x-ux*PULL_BACK_M, y-uy*PULL_BACK_M
            if math.hypot(sx-rp[0],sy-rp[1])<0.25: sx,sy=x,y     # too short to bother
            # Goals 0.5-0.6 m out kept going STUCK: after the 0.45 m pull-back the target
            # is almost under the robot, so Nav2 shuffles without making progress.
            if math.hypot(sx-rp[0],sy-rp[1])<MIN_GOAL_DIST: too_close+=1; continue
            cand.append((d/max(a,1)**0.5,sx,sy,a,d,x,y))
        print("  mapped %.1f m2 | frontiers %d | candidates %d (too close %d, blacklisted %d) | robot (%+.2f,%+.2f) | batt %s"
              %(known,len(fr),len(cand),too_close,blacklisted,rp[0],rp[1],
                "%.2fV"%e.batt if e.batt else "?"))
        if not cand:
            print("  no frontiers left - exploration complete"); break
        cand.sort()
        _,gx,gy,area,dist,fx,fy=cand[0]
        if not e.is_free(gx,gy):
            print("    -> goal (%+.2f,%+.2f) is not in free space, skipping"%(gx,gy))
            e.blacklist.append((fx,fy)); continue
        print("    -> goal (%+.2f,%+.2f)  %.1f m away, cluster %d cells"%(gx,gy,dist,area))
        r=e.goto(gx,gy)
        print("       %s"%r)
        if r in ("ABORTED","REJECTED","TIMEOUT","STUCK"):
            # blacklist the FRONTIER, which is what the candidate filter compares against
            e.blacklist.append((fx,fy))
            if r=="STUCK":
                p=e.robot_xy()
                if p: e.blacklist.append(p)   # never aim back at the trap itself
        else:
            visited+=1
    e.stop()
    print("\nvisited %d frontiers in %.0f s"%(visited,time.time()-t0))
    known=int((np.array(e.map.data)>=0).sum())*e.map.info.resolution**2
    print("final mapped area: %.1f m2  (%dx%d cells)"%(known,e.map.info.width,e.map.info.height))

if __name__=="__main__": main()
