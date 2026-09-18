#!/usr/bin/env python3
"""Find the clearest 15x15 cm patch of floor in the room and draw it on the map.

"Clear" is measured properly, with a distance transform over everything the robot must not
put a cube on or near:
  - mapped walls
  - UNKNOWN space (we cannot claim a cell is clear if it was never observed)
  - every cube already detected, so the drop zone is not placed on top of one

The winner is the free cell whose distance to the nearest such obstacle is greatest. A tie is
broken toward the centre of the mapped floor, which keeps the zone reachable from all sides.

Finally it asks Nav2 whether a robot standing position beside the zone is actually plannable -
a drop zone the robot cannot drive to is useless.
"""
import sys, math, numpy as np, rclpy, cv2, re
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

BOX        = 0.15     # the zone is 15 x 15 cm
CUBES      = sys.argv[1] if len(sys.argv)>1 else "/home/jetson/maps/cubes_confirmed.txt"
CUBE_CLEAR = 0.25     # keep this far from any detected cube
STAND_OFF  = 0.45     # where the robot would stand to place into the zone
FREE_MAX   = 55

PAT=re.compile(r"map\s+\(([-+0-9.]+),\s*([-+0-9.]+)\)\s+height\s+([0-9.]+)")

def load_cubes(path):
    out=[]
    try:
        for l in open(path):
            m=PAT.search(l)
            if m and float(m.group(3))<=0.10:      # floor cubes only
                out.append((float(m.group(1)),float(m.group(2))))
    except FileNotFoundError:
        print("   (no cube file at %s - ignoring cubes)"%path)
    return out

def main():
    rclpy.init(); n=Node("find_dropzone")
    lat=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
    d={}
    n.create_subscription(OccupancyGrid,"/map",lambda m:d.__setitem__("m",m),lat)
    pub=n.create_publisher(MarkerArray,"/dropzone",lat)
    plan=ActionClient(n,ComputePathToPose,"compute_path_to_pose")
    t0=n.get_clock().now()
    while (n.get_clock().now()-t0).nanoseconds<20e9 and "m" not in d: rclpy.spin_once(n,timeout_sec=0.2)
    if "m" not in d: print("no /map"); return 2
    m=d["m"]; res=m.info.resolution
    g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)

    free    = ((g>=0)&(g<FREE_MAX))
    blocked = ~free                      # walls AND unknown both count as blocked
    cubes=load_cubes(CUBES)
    print("map %dx%d, %d free cells, %d blocked (walls + unknown), %d floor cubes"
          %(m.info.width,m.info.height,int(free.sum()),int(blocked.sum()),len(cubes)))

    occ=blocked.astype(np.uint8).copy()
    r=max(1,int(round(CUBE_CLEAR/res)))
    for cx,cy in cubes:
        px=int((cx-m.info.origin.position.x)/res); py=int((cy-m.info.origin.position.y)/res)
        cv2.circle(occ,(px,py),r,1,-1)

    # distance (in cells) from every free cell to the nearest blocked cell / cube
    dist=cv2.distanceTransform((1-occ).astype(np.uint8),cv2.DIST_L2,5)*res
    need=BOX*math.sqrt(2)/2.0            # half-diagonal: the whole box must fit
    cand=np.argwhere(dist>need)
    if not len(cand): print("no cell clears %.3f m"%need); return 2

    ys,xs=np.where(free)
    cxm=m.info.origin.position.x+(xs.mean()+0.5)*res
    cym=m.info.origin.position.y+(ys.mean()+0.5)*res
    print("centre of mapped floor: (%+.2f, %+.2f)"%(cxm,cym))

    best=None
    for (cy,cx) in cand:
        wx=m.info.origin.position.x+(cx+0.5)*res
        wy=m.info.origin.position.y+(cy+0.5)*res
        clear=float(dist[cy,cx])
        # maximise clearance; nudge toward the centre only to break near-ties
        score=clear-0.02*math.hypot(wx-cxm,wy-cym)
        if best is None or score>best[0]: best=(score,wx,wy,clear)
    _,bx,by,clear=best
    print("\nclearest 15x15 cm patch: (%+.3f, %+.3f)"%(bx,by))
    print("   nearest wall / unknown / cube: %.3f m"%clear)
    print("   the 15 cm box needs %.3f m of half-diagonal, so it fits with %.3f m to spare"
          %(need,clear-need))
    if cubes:
        dc=min(math.hypot(bx-c[0],by-c[1]) for c in cubes)
        print("   nearest detected cube: %.3f m"%dc)

    # can the robot actually get to it?
    reach=None
    if plan.wait_for_server(timeout_sec=8.0):
        for ang in range(0,360,45):
            sx=bx+STAND_OFF*math.cos(math.radians(ang))
            sy=by+STAND_OFF*math.sin(math.radians(ang))
            gm=ComputePathToPose.Goal(); gm.use_start=False
            p=PoseStamped(); p.header.frame_id="map"
            p.pose.position.x=sx; p.pose.position.y=sy; p.pose.orientation.w=1.0
            gm.goal=p
            f=plan.send_goal_async(gm); rclpy.spin_until_future_complete(n,f,timeout_sec=8)
            h=f.result()
            if h is None or not h.accepted: continue
            rf=h.get_result_async(); rclpy.spin_until_future_complete(n,rf,timeout_sec=15)
            if rf.result() and rf.result().result.path.poses:
                reach=(sx,sy,ang,len(rf.result().result.path.poses)); break
    print("   robot can stand at (%+.2f,%+.2f) [%d deg], path of %d poses"%reach
          if reach else "   WARNING: no reachable standing position found beside the zone")

    def build():
        ma=MarkerArray()
        f=Marker()
        f.header.frame_id="map"; f.header.stamp=n.get_clock().now().to_msg()
        f.ns="dropzone"; f.id=0; f.type=Marker.CUBE; f.action=Marker.ADD
        f.pose.position.x=bx; f.pose.position.y=by; f.pose.position.z=0.005
        f.pose.orientation.w=1.0
        f.scale.x=f.scale.y=BOX; f.scale.z=0.01
        f.color.r,f.color.g,f.color.b,f.color.a=0.1,0.5,1.0,0.55
        ma.markers.append(f)
        o=Marker()
        o.header=f.header; o.ns="dropzone"; o.id=1
        o.type=Marker.LINE_STRIP; o.action=Marker.ADD
        o.pose.orientation.w=1.0; o.scale.x=0.012
        o.color.r,o.color.g,o.color.b,o.color.a=0.0,0.9,1.0,1.0
        from geometry_msgs.msg import Point
        h=BOX/2.0
        for dx,dy in ((-h,-h),(h,-h),(h,h),(-h,h),(-h,-h)):
            o.points.append(Point(x=bx+dx,y=by+dy,z=0.012))
        ma.markers.append(o)
        t=Marker()
        t.header=f.header; t.ns="dropzone"; t.id=2
        t.type=Marker.TEXT_VIEW_FACING; t.action=Marker.ADD
        t.pose.position.x=bx; t.pose.position.y=by; t.pose.position.z=0.28
        t.pose.orientation.w=1.0; t.scale.z=0.10
        t.color.r=t.color.g=t.color.b=1.0; t.color.a=1.0
        t.text="DROP ZONE 15x15cm\n(%+.2f, %+.2f)  clear %.2f m"%(bx,by,clear)
        ma.markers.append(t)
        return ma

    pub.publish(build())
    n.create_timer(2.0, lambda: pub.publish(build()))
    print("\npublishing the zone on /dropzone - add a MarkerArray display for it in RViz")
    rclpy.spin(n)

if __name__=="__main__": sys.exit(main())
