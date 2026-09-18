#!/usr/bin/env python3
"""Detect cuboids: YOLO11n + depth-based floor-plane gating.

A detection is accepted only if its 3D point lies ON the floor plane (within a
height window), which rejects furniture, pictures and clutter that the model
false-positives on. Depth is a MEDIAN over a patch, never a single pixel.
"""
import sys, numpy as np, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL="/home/jetson/cuboid_best_baseline.pt"
CONF=0.20
H_MIN, H_MAX = 0.005, 0.12      # object height above floor plane (m)
Z_MIN, Z_MAX = 0.15, 2.0        # plausible working distance (m)

class Det(Node):
    def __init__(self):
        super().__init__("cuboid_detect")
        self.b=CvBridge(); self.rgb=None; self.depth=None; self.K=None
        self.create_subscription(Image,"/camera/color/image_raw",self.cb_rgb,1)
        self.create_subscription(Image,"/camera/depth/image_raw",self.cb_d,1)
        self.create_subscription(CameraInfo,"/camera/depth/camera_info",self.cb_k,1)
        self.model=YOLO(MODEL)
    def cb_rgb(self,m): self.rgb=self.b.imgmsg_to_cv2(m,"bgr8")
    def cb_d(self,m):   self.depth=self.b.imgmsg_to_cv2(m,"32FC1").astype(np.float32)
    def cb_k(self,m):   self.K=np.array(m.k).reshape(3,3)

    def to3d(self,u,v,z):
        fx,fy,cx,cy=self.K[0,0],self.K[1,1],self.K[0,2],self.K[1,2]
        return np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])

    def floor_plane(self):
        """RANSAC the dominant plane in the lower half of the depth image."""
        d=self.depth; h,w=d.shape
        vs,us=np.mgrid[h//2:h:6, 0:w:6]
        zs=d[vs,us]/1000.0
        ok=(zs>Z_MIN)&(zs<Z_MAX)
        if ok.sum()<200: return None
        P=np.stack([self.to3d(u,v,z) for u,v,z in zip(us[ok],vs[ok],zs[ok])])
        best,bestn=None,0
        rng=np.random.default_rng(0)
        for _ in range(120):
            i=rng.choice(len(P),3,replace=False)
            a,b,c=P[i]
            n=np.cross(b-a,c-a); nn=np.linalg.norm(n)
            if nn<1e-6: continue
            n=n/nn; dist=np.abs((P-a)@n)
            cnt=int((dist<0.02).sum())
            if cnt>bestn: bestn,best=cnt,(n,a)
        if best is None or bestn<len(P)*0.3: return None
        n,a=best
        inl=P[np.abs((P-a)@n)<0.02]
        c=inl.mean(axis=0)
        u_,s_,vt=np.linalg.svd(inl-c); n=vt[-1]
        if n@np.array([0,1,0])<0: n=-n         # point "down" in camera frame
        return n,c,bestn/len(P)

    def detect(self):
        if self.rgb is None or self.depth is None or self.K is None: return None,[]
        fp=self.floor_plane()
        r=self.model.predict(self.rgb,imgsz=640,conf=CONF,verbose=False)[0]
        out=[]
        for bx in r.boxes:
            x1,y1,x2,y2=[float(v) for v in bx.xyxy[0]]; conf=float(bx.conf[0])
            u,v=int((x1+x2)/2),int((y1+y2)/2)
            patch=self.depth[max(0,v-4):v+5, max(0,u-4):u+5]
            good=patch[(patch>0)&np.isfinite(patch)]
            if good.size<5: out.append((conf,u,v,None,None,"no depth")); continue
            z=float(np.median(good))/1000.0
            if not (Z_MIN<z<Z_MAX): out.append((conf,u,v,z,None,"bad range")); continue
            p=self.to3d(u,v,z)
            if fp is None: out.append((conf,u,v,z,None,"no floor")); continue
            n,c,frac=fp
            hgt=float(-(p-c)@n)                      # height above the floor plane
            verdict="CUBOID" if H_MIN<hgt<H_MAX else f"reject h={hgt:+.3f}"
            out.append((conf,u,v,z,hgt,verdict))
        return fp,out

def main():
    rclpy.init(); d=Det()
    import time; t=time.time()
    while time.time()-t<12 and (d.rgb is None or d.depth is None or d.K is None):
        rclpy.spin_once(d,timeout_sec=0.3)
    fp,res=d.detect()
    if fp: print(f"floor plane found, inlier fraction {fp[2]:.2f}")
    else:  print("floor plane NOT found")
    print(f"{len(res)} raw detections:")
    for conf,u,v,z,h,verd in res:
        zs=f"{z:.2f}m" if z else "  -  "
        hs=f"{h:+.3f}m" if h is not None else "   -   "
        print(f"  conf {conf:.2f} px({u:3d},{v:3d}) z {zs} h {hs}  {verd}")
    acc = sum(1 for r in res if r[5] == "CUBOID"); print("accepted:", acc)
if __name__=="__main__": main()
