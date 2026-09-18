"""Is AMCL's pose right? Project live lidar returns into the RECORDED map and measure how
far each falls from the nearest wall cell. Good localisation puts most within ~10 cm."""
import math, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros, transforms3d as tfs
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

rclpy.init(); n=Node("checkloc")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
d={}
n.create_subscription(OccupancyGrid,"/map",lambda m:d.__setitem__("m",m),
    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(LaserScan,"/scan",lambda m:d.__setitem__("s",m),rclpy.qos.qos_profile_sensor_data)
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<15e9 and len(d)<2: rclpy.spin_once(n,timeout_sec=0.2)

m=d["m"]; res=m.info.resolution
g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
walls=np.argwhere(g>=65)
wx=m.info.origin.position.x+(walls[:,1]+0.5)*res
wy=m.info.origin.position.y+(walls[:,0]+0.5)*res
print("recorded map has %d wall cells"%len(walls))

s=d["s"]
# retry while spinning: a single lookup races the TF buffer's newest entry
tr=None
for _ in range(60):
    rclpy.spin_once(n,timeout_sec=0.1)
    s=d["s"]
    try:
        tr=buf.lookup_transform("map",s.header.frame_id,rclpy.time.Time()).transform
        break
    except Exception:
        continue
if tr is None:
    print("could not look up map -> %s"%s.header.frame_id); raise SystemExit(1)
q=tr.rotation
T=tfs.affines.compose([tr.translation.x,tr.translation.y,tr.translation.z],
                      tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z]),[1,1,1])
r=np.array(s.ranges); ang=s.angle_min+np.arange(len(r))*s.angle_increment
ok=np.isfinite(r)&(r>s.range_min)&(r<s.range_max)
pts=np.stack([r[ok]*np.cos(ang[ok]),r[ok]*np.sin(ang[ok]),np.zeros(ok.sum()),np.ones(ok.sum())])
P=(T@pts).T[:,:2]
dist=np.array([np.min(np.hypot(wx-p[0],wy-p[1])) for p in P])
print("%d lidar returns projected into the map"%len(P))
print("   median miss %.3f m, mean %.3f m"%(np.median(dist),dist.mean()))
for th in (0.05,0.10,0.20,0.50):
    print("   within %.2f m of a mapped wall: %5.1f%%"%(th,100.0*(dist<th).sum()/len(dist)))
v="GOOD - the robot is where AMCL thinks it is" if np.median(dist)<0.12 else \
  "POOR - pose likely wrong; set it with RViz '2D Pose Estimate'"
print("\n   verdict: %s"%v)
rclpy.shutdown()
