"""Global localisation: spread AMCL's particles over the whole map, then rotate in place so
it has the motion it needs to converge, and VERIFY the result against the recorded map.

Rotation only - no translation - so this is safe to run without a clear path.
"""
import math, time, numpy as np, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import tf2_ros, transforms3d as tfs
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

TURN_SPEED = 0.5     # rad/s
TURN_SECS  = 26.0    # ~2 full revolutions

rclpy.init(); n=Node("localise")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
d={}
n.create_subscription(OccupancyGrid,"/map",lambda m:d.__setitem__("m",m),
    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(LaserScan,"/scan",lambda m:d.__setitem__("s",m),rclpy.qos.qos_profile_sensor_data)
n.create_subscription(PoseArray,"/particle_cloud",lambda m:d.__setitem__("pc",m),rclpy.qos.qos_profile_sensor_data)
n.create_subscription(PoseWithCovarianceStamped,"/amcl_pose",lambda m:d.__setitem__("ap",m),10)
cmd=n.create_publisher(Twist,"/cmd_vel",10)

def spin(sec):
    t=n.get_clock().now()
    while (n.get_clock().now()-t).nanoseconds<sec*1e9: rclpy.spin_once(n,timeout_sec=0.05)

spin(3.0)
if "m" not in d: print("no /map"); raise SystemExit(1)

cli=n.create_client(Empty,"/reinitialize_global_localization")
if cli.wait_for_service(timeout_sec=10.0):
    f=cli.call_async(Empty.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    print("particles scattered across the whole map")
else:
    print("WARN: no global localisation service")
spin(2.0)

def spread():
    pc=d.get("pc")
    if pc is None or not pc.poses: return None,0
    xy=np.array([[p.position.x,p.position.y] for p in pc.poses])
    return float(np.hypot(*(xy.std(axis=0)))), len(pc.poses)

s0,n0=spread()
print("before turning: %d particles, spread %.2f m"%(n0,s0 if s0 else -1))

print("rotating in place at %.1f rad/s for %.0f s ..."%(TURN_SPEED,TURN_SECS))
t=Twist(); t.angular.z=TURN_SPEED
t0=time.time(); last=0
while time.time()-t0 < TURN_SECS:
    cmd.publish(t); rclpy.spin_once(n,timeout_sec=0.05)
    if time.time()-t0-last >= 6.0:
        last=time.time()-t0
        sp,cnt=spread()
        print("   t=%4.0fs  %d particles, spread %.2f m"%(last,cnt,sp if sp else -1))
# stop, firmly
t=Twist()
for _ in range(15): cmd.publish(t); rclpy.spin_once(n,timeout_sec=0.03); time.sleep(0.03)
print("stopped")
spin(3.0)

sp,cnt=spread()
print("after turning: %d particles, spread %.2f m"%(cnt,sp if sp else -1))
ap=d.get("ap")
if ap:
    p=ap.pose.pose.position; c=ap.pose.covariance
    print("amcl pose (%.3f, %.3f)  covariance xx %.3f yy %.3f yaw %.3f"%(p.x,p.y,c[0],c[7],c[35]))

# --- the real test: does the live lidar line up with the recorded map? ---
m=d["m"]; res=m.info.resolution
g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
walls=np.argwhere(g>=65)
wx=m.info.origin.position.x+(walls[:,1]+0.5)*res
wy=m.info.origin.position.y+(walls[:,0]+0.5)*res
tr=None
for _ in range(80):
    rclpy.spin_once(n,timeout_sec=0.1)
    s=d.get("s")
    if s is None: continue
    try:
        tr=buf.lookup_transform("map",s.header.frame_id,rclpy.time.Time()).transform; break
    except Exception: continue
if tr is None: print("no map->laser transform"); raise SystemExit(1)
q=tr.rotation
T=tfs.affines.compose([tr.translation.x,tr.translation.y,tr.translation.z],
                      tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z]),[1,1,1])
s=d["s"]; r=np.array(s.ranges); ang=s.angle_min+np.arange(len(r))*s.angle_increment
ok=np.isfinite(r)&(r>s.range_min)&(r<s.range_max)
pts=np.stack([r[ok]*np.cos(ang[ok]),r[ok]*np.sin(ang[ok]),np.zeros(ok.sum()),np.ones(ok.sum())])
P=(T@pts).T[:,:2]
dist=np.array([np.min(np.hypot(wx-p[0],wy-p[1])) for p in P])
print("\nVERIFY against the recorded map (%d returns):"%len(P))
print("   median miss %.3f m"%np.median(dist))
for th in (0.05,0.10,0.20,0.50):
    print("   within %.2f m of a mapped wall: %5.1f%%"%(th,100.0*(dist<th).sum()/len(dist)))
print("\n   %s"%("LOCALISED - the robot knows where it is"
      if np.median(dist)<0.12 else "NOT CONVERGED - needs a short drive, rotation was not enough"))
rclpy.shutdown()
