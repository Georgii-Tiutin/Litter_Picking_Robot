"""Print the neighbourhood of the robot from /map, /global_costmap and /local_costmap
side by side, so a phantom obstacle can be attributed to its source."""
import math, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

R=1.4   # metres around the robot to show
rclpy.init(); n=Node("dump_costmap")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
g={}
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
for t,k in (("/map","map"),("/global_costmap/costmap","global"),("/local_costmap/costmap","local")):
    n.create_subscription(OccupancyGrid,t,lambda m,k=k:g.__setitem__(k,m),qos)
cloud={}
n.create_subscription(PointCloud2,"/camera/depth/points",lambda m:cloud.__setitem__("c",m),
                      rclpy.qos.qos_profile_sensor_data)
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<10e9 and (len(g)<3 or "c" not in cloud):
    rclpy.spin_once(n,timeout_sec=0.2)

tr=buf.lookup_transform("map","base_footprint",rclpy.time.Time()).transform
rx,ry=tr.translation.x,tr.translation.y
q=tr.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2))
print("robot (%+.2f,%+.2f) heading %+.0f deg   [R]=robot, x=lethal100, +=inscribed99, .=free, ?=unknown"
      %(rx,ry,math.degrees(yaw)))

def show(key):
    m=g.get(key)
    if m is None: print("  %s: missing"%key); return
    a=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
    res=m.info.resolution
    cx=int((rx-m.info.origin.position.x)/res); cy=int((ry-m.info.origin.position.y)/res)
    r=int(R/res); step=max(1,r//22)
    print("\n=== %s  (res %.3f, %dx%d, origin %+.2f,%+.2f)"%(key,res,m.info.width,m.info.height,
          m.info.origin.position.x,m.info.origin.position.y))
    for yy in range(cy+r,cy-r-1,-step):
        row=""
        for xx in range(cx-r,cx+r+1,step):
            if abs(xx-cx)<=step//2 and abs(yy-cy)<=step//2: row+="R"; continue
            if not(0<=xx<m.info.width and 0<=yy<m.info.height): row+=" "; continue
            v=a[yy,xx]
            row += "?" if v<0 else ("." if v<50 else ("o" if v<99 else ("+" if v==99 else "x")))
        print("   "+row)
    # counts within the footprint radius
    for rad in (0.20,0.30):
        rr=int(rad/res); patch=a[max(0,cy-rr):cy+rr+1, max(0,cx-rr):cx+rr+1]
        print("   within %.2f m of the robot: %d cells >=99, %d ==100, %d unknown, %d free"
              %(rad,(patch>=99).sum(),(patch==100).sum(),(patch<0).sum(),((patch>=0)&(patch<50)).sum()))

for k in ("map","global","local"): show(k)

c=cloud.get("c")
if c is not None:
    arr=point_cloud2.read_points(c,field_names=("x","y","z"),skip_nans=True)
    p=np.stack([arr["x"],arr["y"],arr["z"]],axis=-1)
    d=np.linalg.norm(p,axis=1)
    print("\n=== /camera/depth/points: %d pts, frame %s"%(len(p),c.header.frame_id))
    print("   nearest %.3f m; points closer than 0.30 m: %d; 0.30-0.50 m: %d"
          %(d.min(),(d<0.30).sum(),((d>=0.30)&(d<0.50)).sum()))
rclpy.shutdown()
