#!/usr/bin/env python3
"""Robot's colour + depth view with best.pt detections and the floor-plane gate.

Green box  = accepted as a floor cube (height above the fitted floor plane is in range)
Red box    = rejected (too high / no depth / out of range) - i.e. a false positive
"""
import numpy as np, cv2, rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import yaml, transforms3d as tfs
OFFS=yaml.safe_load(open("/home/jetson/yahboomcar_ws/src/arm_kin/param/offset_value.yaml"))
# end-effector pose at the observation pose [90,120,0,0,90], from FK
CUR_END=[0.1458589529828534,0.00022969568906952754,0.18566515428310748,
         0.00012389155580734876,1.0471973953319513,8.297829493472317e-05]
END2CAM=np.array([[0,0,1,-0.101],[-1,0,0,0.002],[0,-1,0,4.82e-02],[0,0,0,1]])
GRASP_MIN,GRASP_MAX=0.150,0.245        # radial distance the arm can reach at grip height

CONF=0.25
H_MIN,H_MAX=0.005,0.12
Z_MIN,Z_MAX=0.12,2.0
EVERY=3                       # run inference every Nth frame

class View(Node):
    def __init__(self):
        super().__init__("vision_view")
        self.b=CvBridge(); self.rgb=None; self.depth=None
        self.K=[477.57421875,0.0,319.3820495605469,0.0,477.55718994140625,238.64108276367188,0.0,0.0,1.0]
        self.model=YOLO("/home/jetson/cuboid_best_baseline.pt")
        self.n=0; self.dets=[]; self.fpi=None; self.zone=None
        self.create_subscription(Image,"/camera/color/image_raw",self.cb_rgb,1)
        self.create_subscription(Image,"/camera/depth/image_raw",self.cb_d,1)
        cv2.namedWindow("robot vision  [ colour + best.pt | depth ]", cv2.WINDOW_AUTOSIZE)
    def cb_rgb(self,m): self.rgb=self.b.imgmsg_to_cv2(m,"bgr8")
    def cb_d(self,m):   self.depth=self.b.imgmsg_to_cv2(m,"32FC1").astype(np.float32)
    def cam3d(self,u,v,z):
        fx,fy,cx,cy=self.K[0],self.K[4],self.K[2],self.K[5]
        return np.array([(u-cx)*z/fx,(v-cy)*z/fy,z])
    def floor(self):
        d=self.depth; h,w=d.shape
        vs,us=np.mgrid[h//2:h:8,0:w:8]; zs=d[vs,us]/1000.0
        ok=(zs>Z_MIN)&(zs<Z_MAX)
        if ok.sum()<120: return None
        P=np.stack([self.cam3d(u,v,z) for u,v,z in zip(us[ok],vs[ok],zs[ok])])
        c=P.mean(axis=0); n=np.linalg.svd(P-c)[2][-1]
        if n@np.array([0,1,0])<0: n=-n
        self.fpi=float(ok.sum())/ok.size
        return n,c
    def to_base(self,u,v,z):
        cam=self.cam3d(u,v,z)
        q=tfs.euler.euler2quat(CUR_END[3],CUR_END[4],CUR_END[5])
        end=tfs.affines.compose(np.asarray(CUR_END[0:3]),tfs.quaternions.quat2mat(q),[1,1,1])
        cm=tfs.affines.compose(np.squeeze(cam),tfs.euler.euler2mat(0,0,0),[1,1,1])
        T=tfs.affines.decompose(np.matmul(end,np.matmul(END2CAM,cm)))[0]
        return T[0]+OFFS["x_offset"],T[1]+OFFS["y_offset"]
    def compute_zone(self):
        """Mark the floor pixels whose base-frame radial distance is graspable."""
        d=self.depth
        m=np.zeros((480,640),np.uint8)
        step=16
        for v in range(120,480,step):
            for u in range(0,640,step):
                z=d[v,u]/1000.0
                if not (0.12<z<2.0) or not np.isfinite(z): continue
                x,y=self.to_base(u,v,z)
                r=math.hypot(x,y)
                if GRASP_MIN<r<GRASP_MAX and abs(y)<0.10:
                    cv2.rectangle(m,(u-step//2,v-step//2),(u+step//2,v+step//2),255,-1)
        self.zone=cv2.morphologyEx(m,cv2.MORPH_CLOSE,np.ones((25,25),np.uint8))
    def infer(self):
        fp=self.floor()
        r=self.model.predict(self.rgb,imgsz=640,conf=CONF,verbose=False)[0]
        out=[]
        for bx in r.boxes:
            x1,y1,x2,y2=[int(t) for t in bx.xyxy[0]]
            u,v=(x1+x2)//2,(y1+y2)//2
            patch=self.depth[max(0,v-4):v+5,max(0,u-4):u+5]
            g=patch[(patch>0)&np.isfinite(patch)]
            if g.size<5: out.append((x1,y1,x2,y2,float(bx.conf[0]),None,None,"no depth")); continue
            z=float(np.median(g))/1000.0
            if not(Z_MIN<z<Z_MAX): out.append((x1,y1,x2,y2,float(bx.conf[0]),z,None,"range")); continue
            hgt=None
            if fp is not None:
                n,c=fp; hgt=float(-(self.cam3d(u,v,z)-c)@n)
            ok = hgt is not None and H_MIN<hgt<H_MAX
            out.append((x1,y1,x2,y2,float(bx.conf[0]),z,hgt,"CUBE" if ok else "not floor"))
        self.dets=out
    def draw(self):
        if self.rgb is None or self.depth is None: return
        self.n+=1
        if self.n%EVERY==0:
            try: self.infer()
            except Exception as e: pass
        if self.n%30==1:
            try: self.compute_zone()
            except Exception as e: pass
        c=cv2.resize(self.rgb,(640,480)).copy()
        d=cv2.resize(self.depth,(640,480))
        lo,hi=150.0,1200.0
        v=np.clip((d-lo)/(hi-lo),0,1); v[(d<=0)|~np.isfinite(d)]=0
        dv=cv2.applyColorMap((v*255).astype(np.uint8),cv2.COLORMAP_JET)
        dv[(d<=0)|~np.isfinite(d)]=(0,0,0)
        if self.zone is not None:
            ov=c.copy(); ov[self.zone>0]=(0,180,0)
            c=cv2.addWeighted(ov,0.25,c,0.75,0)
            cnts,_=cv2.findContours(self.zone,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(c,cnts,-1,(0,220,0),2)
            if cnts:
                M=max(cnts,key=cv2.contourArea); mm=cv2.moments(M)
                if mm["m00"]>0:
                    cx,cy=int(mm["m10"]/mm["m00"]),int(mm["m01"]/mm["m00"])
                    cv2.putText(c,"PLACE BLOCK HERE",(cx-95,cy),
                                cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
        n_ok=0
        for x1,y1,x2,y2,conf,z,hgt,verd in self.dets:
            ok = verd=="CUBE"; n_ok+=ok
            col=(0,220,0) if ok else (0,0,255)
            for im in (c,dv): cv2.rectangle(im,(x1,y1),(x2,y2),col,2)
            lbl="%.2f"%conf + (" %.0fmm"%(hgt*1000) if hgt is not None else " "+verd)
            cv2.putText(c,lbl,(x1,max(14,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,0.5,col,2)
            if ok: cv2.putText(c,"%.0fmm away"%(z*1000),(x1,y2+16),
                               cv2.FONT_HERSHEY_SIMPLEX,0.45,col,1)
        cv2.putText(c,"COLOUR + best.pt   green=floor cube  red=rejected",(8,22),
                    cv2.FONT_HERSHEY_SIMPLEX,0.52,(255,255,255),2)
        cv2.putText(c,"accepted: %d of %d"%(n_ok,len(self.dets)),(8,468),
                    cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,220,0),2)
        cv2.putText(dv,"DEPTH 0.15-1.2 m  (black = no return)",(8,22),
                    cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
        ctr=d[240,320]
        cv2.putText(dv,("centre %.0f mm"%ctr) if ctr>0 else "centre: invalid",(8,468),
                    cv2.FONT_HERSHEY_SIMPLEX,0.55,(255,255,255),2)
        for im in (c,dv): cv2.drawMarker(im,(320,240),(255,255,255),cv2.MARKER_CROSS,16,2)
        cv2.imshow("robot vision  [ colour + best.pt | depth ]", np.hstack([c,dv]))
        cv2.waitKey(1)

def main():
    rclpy.init(); v=View()
    while rclpy.ok():
        rclpy.spin_once(v,timeout_sec=0.05); v.draw()

if __name__=="__main__": main()
