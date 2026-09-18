"""Print cube marker positions and check they are distinct and not inside mapped walls."""
import math, numpy as np, rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
rclpy.init(); n=Node("rm"); d={}
lat=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(MarkerArray,"/cube_markers",lambda m:d.__setitem__("m",m),lat)
n.create_subscription(OccupancyGrid,"/map",lambda m:d.__setitem__("g",m),lat)
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<15e9 and len(d)<2: rclpy.spin_once(n,timeout_sec=0.2)
ma=d.get("m"); g=d.get("g")
if ma is None: print("no markers"); raise SystemExit
grid=None
if g is not None:
    grid=np.array(g.data,dtype=np.int16).reshape(g.info.height,g.info.width)
cubes=[]
for m in ma.markers:
    if m.ns!="cubes": continue
    cubes.append((m.id,m.pose.position.x,m.pose.position.y,m.pose.position.z))
labels={}
for m in ma.markers:
    if m.ns=="cube_labels": labels[m.id-1000]=m.text
print("%d cube marker(s):"%len(cubes))
for i,x,y,z in cubes:
    cell=""
    if grid is not None:
        cx=int((x-g.info.origin.position.x)/g.info.resolution)
        cy=int((y-g.info.origin.position.y)/g.info.resolution)
        if 0<=cx<g.info.width and 0<=cy<g.info.height:
            v=grid[cy,cx]
            cell = "in WALL(%d)"%v if v>=65 else ("in unknown" if v<0 else "on free floor")
        else: cell="OUTSIDE map"
    print("   %-22s map (%+.3f, %+.3f) z %.3f   %s"%(labels.get(i,"cube"),x,y,z,cell))
print("\npairwise distances (merge radius is 0.22 m):")
for a in range(len(cubes)):
    for b in range(a+1,len(cubes)):
        dd=math.hypot(cubes[a][1]-cubes[b][1],cubes[a][2]-cubes[b][2])
        if dd<0.60: print("   cube %d <-> cube %d : %.2f m  %s"%(cubes[a][0]+1,cubes[b][0]+1,dd,
                          "<-- SUSPICIOUS, may be one cube counted twice" if dd<0.40 else ""))
rclpy.shutdown()
