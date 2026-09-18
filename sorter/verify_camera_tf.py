#!/usr/bin/env python3
"""Validate the published camera TF by checking the FLOOR projects to z ~ 0 in base frame.

If distant floor points read high, the transform's pitch is wrong and the costmap will
mark phantom obstacles at a fixed range. This must pass before any driving.
"""
import sys, time, math, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import transforms3d as tfs

PARENT="base_footprint"

class V(Node):
    def __init__(self):
        super().__init__("verify_cam_tf")
        self.cloud=None
        self.create_subscription(PointCloud2,"/camera/depth/points",self.cb,1)
        self.buf=tf2_ros.Buffer(); self.tfl=tf2_ros.TransformListener(self.buf,self)
    def cb(self,m): self.cloud=m

def main():
    rclpy.init(); v=V()
    t0=time.time()
    while time.time()-t0<20 and v.cloud is None: rclpy.spin_once(v,timeout_sec=0.3)
    if v.cloud is None: print("no /camera/depth/points received"); return 2
    print("cloud frame: %s   %d points"%(v.cloud.header.frame_id,
          v.cloud.width*v.cloud.height))
    CHILD=v.cloud.header.frame_id          # use whatever the cloud is actually stamped with
    t0=time.time(); tr=None
    while time.time()-t0<10 and tr is None:
        try: tr=v.buf.lookup_transform(PARENT,CHILD,rclpy.time.Time())
        except Exception: rclpy.spin_once(v,timeout_sec=0.2)
    if tr is None: print("TF %s -> %s NOT available"%(PARENT,CHILD)); return 2
    q=tr.transform.rotation; t=tr.transform.translation
    R=tfs.quaternions.quat2mat([q.w,q.x,q.y,q.z])
    T=np.array([t.x,t.y,t.z])
    print("TF found: translation (%.4f, %.4f, %.4f)"%(t.x,t.y,t.z))

    arr=pc2.read_points(v.cloud,field_names=("x","y","z"),skip_nans=True)
    pts=np.stack([arr["x"],arr["y"],arr["z"]],axis=-1).astype(np.float64)
    if len(pts)>60000:
        pts=pts[np.random.default_rng(0).choice(len(pts),60000,replace=False)]
    base=(R@pts.T).T + T
    fwd=base[:,0]; lat=base[:,1]; up=base[:,2]
    print("\npoints in base frame: %d"%len(base))
    print("  forward x: %.2f .. %.2f m"%(fwd.min(),fwd.max()))
    print("  height  z: %.2f .. %.2f m"%(up.min(),up.max()))
    print("\nheight of the lowest 30%% of points, binned by forward distance")
    print("  (this is the FLOOR - it should read ~0.00 m everywhere)")
    print("  %8s %8s %8s %8s"%("x range","n","z median","z p90"))
    ok=True
    for lo,hi in ((0.3,0.5),(0.5,0.7),(0.7,0.9),(0.9,1.2),(1.2,1.5)):
        sel=(fwd>=lo)&(fwd<hi)&(np.abs(lat)<0.4)
        if sel.sum()<200: print("  %4.1f-%4.1f   %6d  (too few)"%(lo,hi,sel.sum())); continue
        zs=np.sort(up[sel]); floor=zs[:max(50,int(0.30*len(zs)))]
        med=float(np.median(floor)); p90=float(np.percentile(floor,90))
        flag="" if abs(med)<0.03 else "   <-- OFF"
        if abs(med)>=0.03: ok=False
        print("  %4.1f-%4.1f   %6d  %+8.3f %+8.3f%s"%(lo,hi,sel.sum(),med,p90,flag))
    print("\n%s"%("FLOOR PROJECTS CORRECTLY - transform is usable"
                  if ok else "TRANSFORM IS OFF - do not enable the costmap source yet"))
    return 0 if ok else 1

if __name__=="__main__": sys.exit(main())
