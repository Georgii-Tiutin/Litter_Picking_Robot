"""Are the known cubes actually represented in the costmap the planner uses?

A cube the robot cannot see is a cube it will drive into. This checks, for every confirmed
cube, whether the LOCAL costmap holds an obstacle at that spot, and how far the robot is from
it - so we can tell whether marking works at range, close in, or not at all.
"""
import math, re, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros

CUBES="/home/jetson/maps/cubes_2026-09-15.txt"
PAT=re.compile(r"cube\s+(\d+):\s+map\s+\(([-+0-9.]+),\s*([-+0-9.]+)\)")

rclpy.init(); n=Node("cubecost")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
d={}
lat=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(OccupancyGrid,"/local_costmap/costmap",lambda m:d.__setitem__("l",m),lat)
n.create_subscription(OccupancyGrid,"/global_costmap/costmap",lambda m:d.__setitem__("g",m),lat)
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<15e9 and len(d)<2: rclpy.spin_once(n,timeout_sec=0.2)

tr=None
for _ in range(60):
    rclpy.spin_once(n,timeout_sec=0.1)
    try:
        t=buf.lookup_transform("map","base_footprint",rclpy.time.Time()).transform
        tr=(t.translation.x,t.translation.y); break
    except Exception: continue
print("robot at (%+.2f,%+.2f)"%tr if tr else "no pose")

cubes=[]
for l in open(CUBES):
    m=PAT.search(l)
    if m: cubes.append((m.group(1),float(m.group(2)),float(m.group(3))))

def to_frame(frame, x, y):
    """Cube positions are in MAP. The local costmap lives in ODOM. Comparing the two
    directly is the same frame error that produced phantom 'unexplained' cells on
    2026-09-12 - convert first."""
    if frame == "map": return (x, y)
    try:
        t=buf.lookup_transform(frame, "map", rclpy.time.Time()).transform
    except Exception:
        return None
    q=t.rotation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2))
    c,s_=math.cos(yaw),math.sin(yaw)
    return (t.translation.x + c*x - s_*y, t.translation.y + s_*x + c*y)

def probe(grid, x, y, radius=0.12):
    if grid is None: return None
    pt=to_frame(grid.header.frame_id, x, y)
    if pt is None: return "no tf"
    x, y = pt
    a=np.array(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)
    res=grid.info.resolution
    cx=int((x-grid.info.origin.position.x)/res); cy=int((y-grid.info.origin.position.y)/res)
    r=max(1,int(radius/res))
    y0,y1=max(0,cy-r),min(grid.info.height,cy+r+1)
    x0,x1=max(0,cx-r),min(grid.info.width,cx+r+1)
    if x0>=x1 or y0>=y1: return "outside"
    patch=a[y0:y1,x0:x1]
    if (patch>=100).any(): return "LETHAL"
    if (patch>=99).any(): return "inscribed"
    if (patch>0).any():   return "cost %d"%patch.max()
    return "clear"

print("\n%-8s %-18s %8s   %-12s %-12s"%("cube","map position","dist","local costmap","global costmap"))
for cid,x,y in cubes:
    dist=math.hypot(x-tr[0],y-tr[1]) if tr else float("nan")
    print("%-8s (%+.2f,%+.2f)%4s %7.2f m   %-12s %-12s"
          %("#"+cid,x,y,"",dist,probe(d.get("l"),x,y),probe(d.get("g"),x,y)))
print("\n'clear' at close range = the robot cannot see that cube and may drive into it")
rclpy.shutdown()
