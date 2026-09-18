"""Transform EVERY depth point into base_footprint via the live TF and histogram the
heights. If the floor does not sit at z~0 everywhere, the obstacle layer marks the floor."""
import numpy as np, rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py import point_cloud2
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import transforms3d as tfs

rclpy.init(); n=Node("floor_hist")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
c={}; mp={}
n.create_subscription(PointCloud2,"/camera/depth/points",lambda m:c.__setitem__("c",m),
                      rclpy.qos.qos_profile_sensor_data)
n.create_subscription(OccupancyGrid,"/map",lambda m:mp.__setitem__("m",m),
    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<10e9 and ("c" not in c or "m" not in mp):
    rclpy.spin_once(n,timeout_sec=0.2)

m=mp.get("m")
if m is not None:
    a=np.array(m.data,dtype=np.int16)
    print("/map from cartographer: %dx%d, %d unknown, %d free(<50), %d occupied(>=50)"
          %(m.info.width,m.info.height,(a<0).sum(),((a>=0)&(a<50)).sum(),(a>=50).sum()))
    print("   -> occupied is %.1f%% of the KNOWN cells"
          %(100.0*(a>=50).sum()/max(1,(a>=0).sum())))

msg=c["c"]; f=msg.header.frame_id
tr=buf.lookup_transform("base_footprint",f,rclpy.time.Time()).transform
q=tr.rotation
T=tfs.affines.compose([tr.translation.x,tr.translation.y,tr.translation.z],
                      tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z]),[1,1,1])
print("\nTF base_footprint <- %s"%f)
print("   translation (%.4f, %.4f, %.4f)"%(tr.translation.x,tr.translation.y,tr.translation.z))
arr=point_cloud2.read_points(msg,field_names=("x","y","z"),skip_nans=True)
p=np.stack([arr["x"],arr["y"],arr["z"]],axis=-1)
ph=np.concatenate([p,np.ones((len(p),1))],axis=1)
b=(T@ph.T).T[:,:3]
X,Y,Z=b[:,0],b[:,1],b[:,2]
print("   %d points -> base_footprint  x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f"
      %(len(b),X.min(),X.max(),Y.min(),Y.max(),Z.min(),Z.max()))

print("\nHeight of points by forward distance (floor MUST read ~0.00):")
print("   range        n      z median   z 5%%     z 95%%    n with 0.05<z<0.60")
for lo,hi in ((0.3,0.5),(0.5,0.7),(0.7,0.9),(0.9,1.1),(1.1,1.3),(1.3,1.6),(1.6,2.0)):
    s=(X>=lo)&(X<hi)
    if s.sum()<50: print("   %.1f-%.1f m   %6d   (too few)"%(lo,hi,s.sum())); continue
    z=Z[s]; marked=((z>0.05)&(z<0.60)).sum()
    print("   %.1f-%.1f m   %6d   %+.3f    %+.3f   %+.3f    %6d  (%4.1f%%)"
          %(lo,hi,s.sum(),np.median(z),np.percentile(z,5),np.percentile(z,95),marked,100.0*marked/s.sum()))
tot=((Z>0.05)&(Z<0.60)&(X<1.2)).sum()
print("\n   TOTAL points the obstacle layer would MARK (0.05<z<0.60, x<1.2): %d of %d"%(tot,len(b)))
rclpy.shutdown()
