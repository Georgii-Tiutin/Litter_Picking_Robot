"""For every lethal cell in the local costmap, ask whether any live sensor explains it."""
import math, numpy as np, rclpy, time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros, transforms3d as tfs
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

rclpy.init(); n=Node("attribute")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
d={}
n.create_subscription(OccupancyGrid,"/local_costmap/costmap",lambda m:d.__setitem__("cm",m),
    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(LaserScan,"/scan",lambda m:d.__setitem__("ls",m),rclpy.qos.qos_profile_sensor_data)
n.create_subscription(PointCloud2,"/camera/depth/points",lambda m:d.__setitem__("pc",m),rclpy.qos.qos_profile_sensor_data)
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<12e9 and len(d)<3: rclpy.spin_once(n,timeout_sec=0.2)

cm=d["cm"]; now=n.get_clock().now().nanoseconds
age=(now-rclpy.time.Time.from_msg(cm.header.stamp).nanoseconds)/1e9
print("local costmap stamp age %.1f s  (large age = the costmap is NOT updating)"%age)
# take a second sample to see whether it changes at all
first=np.array(cm.data,dtype=np.int16).copy()
t1=n.get_clock().now()
while (n.get_clock().now()-t1).nanoseconds<3e9: rclpy.spin_once(n,timeout_sec=0.2)
second=np.array(d["cm"].data,dtype=np.int16)
print("cells changed over 3 s: %d of %d"%((first!=second).sum(),len(first)))

cm=d["cm"]; res=cm.info.resolution
a=np.array(cm.data,dtype=np.int16).reshape(cm.info.height,cm.info.width)
REF=cm.header.frame_id
print("costmap frame: %s  (all comparisons now done in THIS frame)"%REF)
tr=buf.lookup_transform(REF,"base_footprint",rclpy.time.Time()).transform
rx,ry=tr.translation.x,tr.translation.y

# lidar returns in the costmap frame
ls=d["ls"]; tl=buf.lookup_transform(REF,ls.header.frame_id,rclpy.time.Time()).transform
ql=tl.rotation
Tl=tfs.affines.compose([tl.translation.x,tl.translation.y,tl.translation.z],
                       tfs.quaternions.quat2mat([ql.w,ql.x,ql.y,ql.z]),[1,1,1])
r=np.array(ls.ranges); ang=ls.angle_min+np.arange(len(r))*ls.angle_increment
ok=np.isfinite(r)&(r>ls.range_min)&(r<ls.range_max)
lp=np.stack([r[ok]*np.cos(ang[ok]),r[ok]*np.sin(ang[ok]),np.zeros(ok.sum()),np.ones(ok.sum())])
L=(Tl@lp).T[:,:2]

# depth points in the costmap frame, only those in the marking band
pc=d["pc"]; tp=buf.lookup_transform(REF,pc.header.frame_id,rclpy.time.Time()).transform
qp=tp.rotation
Tp=tfs.affines.compose([tp.translation.x,tp.translation.y,tp.translation.z],
                       tfs.quaternions.quat2mat([qp.w,qp.x,qp.y,qp.z]),[1,1,1])
arr=point_cloud2.read_points(pc,field_names=("x","y","z"),skip_nans=True)
P=np.stack([arr["x"],arr["y"],arr["z"]],axis=-1)
P=np.concatenate([P,np.ones((len(P),1))],axis=1)
Pm=(Tp@P.T).T
band=Pm[(Pm[:,2]>0.05)&(Pm[:,2]<0.60)][:,:2]
print("lidar returns %d, depth points in the 0.05-0.60 m band %d"%(len(L),len(band)))

lethal=np.argwhere(a==100)
expl_l=expl_d=unexp=0; unex_d=[]
for (cy,cx) in lethal:
    x=cm.info.origin.position.x+(cx+0.5)*res; y=cm.info.origin.position.y+(cy+0.5)*res
    dist=math.hypot(x-rx,y-ry)
    if dist>1.4: continue
    nl=np.min(np.hypot(L[:,0]-x,L[:,1]-y)) if len(L) else 9
    nd=np.min(np.hypot(band[:,0]-x,band[:,1]-y)) if len(band) else 9
    if nl<0.12: expl_l+=1
    elif nd<0.12: expl_d+=1
    else: unexp+=1; unex_d.append(dist)
print("\nTRUE LETHAL cells (cost 100 only) within 1.4 m: %d"%(expl_l+expl_d+unexp))
print("   explained by LIDAR : %d"%expl_l)
print("   explained by DEPTH : %d"%expl_d)
print("   UNEXPLAINED        : %d"%unexp)
if unex_d:
    u=np.array(unex_d)
    print("   unexplained sit %.2f-%.2f m from the robot (median %.2f)"%(u.min(),u.max(),np.median(u)))

# where are the true lethal cells, relative to the robot?
print("\nmap of TRUE LETHAL cells only (X), robot at R, 5 cm per char, 2.8 m across:")
res=cm.info.resolution
cx0=int((rx-cm.info.origin.position.x)/res); cy0=int((ry-cm.info.origin.position.y)/res)
for yy in range(cy0+28,cy0-29,-2):
    row=""
    for xx in range(cx0-28,cx0+29,1):
        if abs(xx-cx0)<=1 and abs(yy-cy0)<=1: row+="R"; continue
        if not(0<=xx<cm.info.width and 0<=yy<cm.info.height): row+=" "; continue
        row += "X" if a[yy,xx]==100 else "."
    print("   "+row)
rclpy.shutdown()
