#!/usr/bin/env python3
"""Static test: does the depth camera put an obstacle in the costmap where the lidar sees nothing?"""
import sys, time, math, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros, transforms3d as tfs

class C(Node):
    def __init__(self):
        super().__init__("check_bb")
        self.cm=None; self.scan=None; self.cloud=None
        self.create_subscription(OccupancyGrid,"/local_costmap/costmap",self.cb_cm,1)
        self.create_subscription(LaserScan,"/scan",self.cb_s,1)
        self.create_subscription(PointCloud2,"/camera/depth/points",self.cb_p,1)
        self.buf=tf2_ros.Buffer(); self.tfl=tf2_ros.TransformListener(self.buf,self)
    def cb_cm(self,m): self.cm=m
    def cb_s(self,m): self.scan=m
    def cb_p(self,m): self.cloud=m

def main():
    rclpy.init(); c=C(); t0=time.time()
    while time.time()-t0<25 and (c.cm is None or c.scan is None or c.cloud is None):
        rclpy.spin_once(c,timeout_sec=0.3)
    missing=[n for n,v in (("costmap",c.cm),("scan",c.scan),("cloud",c.cloud)) if v is None]
    if missing: print("missing: %s"%missing); return 2

    # 1. what the DEPTH camera sees ahead, in base frame
    for _ in range(40): rclpy.spin_once(c,timeout_sec=0.1)   # let the TF buffer fill
    tr=None; t0=time.time()
    while time.time()-t0<20 and tr is None:
        try: tr=c.buf.lookup_transform("base_footprint",c.cloud.header.frame_id,rclpy.time.Time())
        except Exception: rclpy.spin_once(c,timeout_sec=0.2)
    if tr is None: print("no camera TF"); return 2
    q=tr.transform.rotation; t=tr.transform.translation
    R=tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z]); T=np.array([t.x,t.y,t.z])
    a=pc2.read_points(c.cloud,field_names=("x","y","z"),skip_nans=True)
    pts=np.stack([a["x"],a["y"],a["z"]],axis=-1).astype(np.float64)
    base=(R@pts.T).T+T
    fwd,lat,up=base[:,0],base[:,1],base[:,2]
    obj=(up>0.05)&(up<0.60)&(fwd>0.25)&(fwd<1.5)&(np.abs(lat)<0.6)
    print("DEPTH: %d points sit 0.05-0.60 m above the floor ahead"%obj.sum())
    if obj.sum()>50:
        print("   nearest %.2f m, spans x %.2f-%.2f, y %+.2f..%+.2f, top %.2f m"
              %(fwd[obj].min(),fwd[obj].min(),fwd[obj].max(),
                lat[obj].min(),lat[obj].max(),up[obj].max()))

    # 2. what the LIDAR sees in that same wedge
    r=np.array(c.scan.ranges); n=len(r)
    ang=c.scan.angle_min+c.scan.angle_increment*np.arange(n)
    ok=(r>0.05)&np.isfinite(r)
    lx=r*np.cos(ang); ly=r*np.sin(ang)
    # same wedge the costmap check uses, so the two are comparable
    wedge=ok&(lx>0.25)&(lx<1.5)&(np.abs(ly)<0.6)
    if wedge.sum():
        i=np.argmin(lx[wedge])
        print("LIDAR in the SAME wedge: %d returns, nearest x %.2f m (y %+.2f)"
              %(wedge.sum(),lx[wedge][i],ly[wedge][i]))
    else:
        print("LIDAR in the SAME wedge: nothing")
    sel=(np.abs(np.degrees(ang))<35)&ok
    print("LIDAR straight ahead (+/-35 deg): nearest %.2f m"
          %(r[sel].min() if sel.sum() else float("nan")))

    # 3. what the COSTMAP now contains ahead of the robot
    m=c.cm; g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
    try:
        tb=c.buf.lookup_transform(m.header.frame_id,"base_footprint",rclpy.time.Time())
        rx,ry=tb.transform.translation.x,tb.transform.translation.y
        qq=tb.transform.rotation
        yaw=math.atan2(2*(qq.w*qq.z+qq.x*qq.y),1-2*(qq.y**2+qq.z**2))
    except Exception:
        print("no robot pose in costmap frame"); return 2
    lethal=0; near=[]
    for iy in range(m.info.height):
        for ix in range(m.info.width):
            v=g[iy,ix]
            if v<100: continue     # /costmap is OccupancyGrid-scaled: 100=lethal, 99=inscribed
            wx=m.info.origin.position.x+(ix+0.5)*m.info.resolution
            wy=m.info.origin.position.y+(iy+0.5)*m.info.resolution
            dx,dy=wx-rx,wy-ry
            f= dx*math.cos(yaw)+dy*math.sin(yaw)
            l=-dx*math.sin(yaw)+dy*math.cos(yaw)
            if 0.25<f<1.5 and abs(l)<0.6:
                lethal+=1; near.append(f)
    print("COSTMAP: %d LETHAL cells (cost 100) in the wedge ahead%s"
          %(lethal, ("  nearest %.2f m"%min(near)) if near else ""))
    print()
    depth_sees = obj.sum()>50
    lidar_blind = (not sel.sum()) or r[sel].min()>1.4
    if depth_sees and lethal>0:
        print("PASS: the depth camera put an obstacle in the costmap.%s"
              %("  Lidar sees nothing there - exactly the case that trapped the robot."
                if lidar_blind else "  (lidar also sees it)"))
        return 0
    if depth_sees and lethal==0:
        print("depth sees the object but the COSTMAP has not marked it"); return 1
    print("depth camera does not see an object ahead - is the bean bag in view?")
    return 1

if __name__=="__main__": sys.exit(main())
