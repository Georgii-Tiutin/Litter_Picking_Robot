"""Clear the local costmap, then watch the unexplained lethal count with the robot
stationary. Decay to ~0 = accumulated residue. Plateau = something regenerates it."""
import math, numpy as np, rclpy, time
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from nav2_msgs.srv import ClearEntireCostmap
import tf2_ros, transforms3d as tfs
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

rclpy.init(); n=Node("decay")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
d={}
n.create_subscription(OccupancyGrid,"/local_costmap/costmap",lambda m:d.__setitem__("cm",m),
    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(LaserScan,"/scan",lambda m:d.__setitem__("ls",m),rclpy.qos.qos_profile_sensor_data)
n.create_subscription(PointCloud2,"/camera/depth/points",lambda m:d.__setitem__("pc",m),rclpy.qos.qos_profile_sensor_data)
cli=n.create_client(ClearEntireCostmap,"/local_costmap/clear_entirely_local_costmap")
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<12e9 and len(d)<3: rclpy.spin_once(n,timeout_sec=0.2)

def spin(sec):
    t=n.get_clock().now()
    while (n.get_clock().now()-t).nanoseconds<sec*1e9: rclpy.spin_once(n,timeout_sec=0.05)

def measure():
    cm=d["cm"]; res=cm.info.resolution
    a=np.array(cm.data,dtype=np.int16).reshape(cm.info.height,cm.info.width)
    REF=cm.header.frame_id      # the costmap lives in odom, NOT map - compare in ITS frame
    tr=buf.lookup_transform(REF,"base_footprint",rclpy.time.Time()).transform
    rx,ry=tr.translation.x,tr.translation.y
    ls=d["ls"]; tl=buf.lookup_transform(REF,ls.header.frame_id,rclpy.time.Time()).transform
    ql=tl.rotation
    Tl=tfs.affines.compose([tl.translation.x,tl.translation.y,tl.translation.z],
                           tfs.quaternions.quat2mat([ql.w,ql.x,ql.y,ql.z]),[1,1,1])
    r=np.array(ls.ranges); ang=ls.angle_min+np.arange(len(r))*ls.angle_increment
    ok=np.isfinite(r)&(r>ls.range_min)&(r<ls.range_max)
    lp=np.stack([r[ok]*np.cos(ang[ok]),r[ok]*np.sin(ang[ok]),np.zeros(ok.sum()),np.ones(ok.sum())])
    L=(Tl@lp).T[:,:2]
    pc=d["pc"]; tp=buf.lookup_transform(REF,pc.header.frame_id,rclpy.time.Time()).transform
    qp=tp.rotation
    Tp=tfs.affines.compose([tp.translation.x,tp.translation.y,tp.translation.z],
                           tfs.quaternions.quat2mat([qp.w,qp.x,qp.y,qp.z]),[1,1,1])
    arr=point_cloud2.read_points(pc,field_names=("x","y","z"),skip_nans=True)
    P=np.stack([arr["x"],arr["y"],arr["z"]],axis=-1)
    Pm=(Tp@np.concatenate([P,np.ones((len(P),1))],axis=1).T).T
    band=Pm[(Pm[:,2]>0.03)&(Pm[:,2]<0.60)][:,:2]
    tot=un=0
    for (cy,cx) in np.argwhere(a==100):
        x=cm.info.origin.position.x+(cx+0.5)*res; y=cm.info.origin.position.y+(cy+0.5)*res
        if math.hypot(x-rx,y-ry)>1.4: continue
        tot+=1
        nl=np.min(np.hypot(L[:,0]-x,L[:,1]-y)) if len(L) else 9
        nd=np.min(np.hypot(band[:,0]-x,band[:,1]-y)) if len(band) else 9
        if nl>=0.12 and nd>=0.12: un+=1
    return tot,un

print("clearing, then watching with the robot stationary...")
if cli.wait_for_service(timeout_sec=8.0):
    f=cli.call_async(ClearEntireCostmap.Request()); rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    print("   cleared")
for t in (2,5,10,20,35,50):
    spin(t if t==2 else t-prev if False else 0)
    pass
prev=0
for t in (2,5,10,20,35,50,70):
    spin(t-prev); prev=t
    tot,un=measure()
    print("   t=%3ds   lethal %4d   unexplained %4d"%(t,tot,un))
rclpy.shutdown()
