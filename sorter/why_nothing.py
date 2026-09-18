"""Why did the re-detection see nothing? Report pose, ALL detections (no match filter),
and save the camera view."""
import math, numpy as np, rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros, transforms3d as tfs
from ultralytics import YOLO

EXPECT=(-0.26,-0.31)
K=[477.57421875,0.0,319.3820495605469,0.0,477.55718994140625,238.64108276367188,0.0,0.0,1.0]
rclpy.init(); n=Node("why"); b=CvBridge()
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
d={}
n.create_subscription(Image,"/camera/color/image_raw",lambda m:d.__setitem__("rgb",b.imgmsg_to_cv2(m,"bgr8")),1)
n.create_subscription(Image,"/camera/depth/image_raw",lambda m:d.__setitem__("dep",b.imgmsg_to_cv2(m,"32FC1").astype(np.float32)),1)
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<15e9 and len(d)<2: rclpy.spin_once(n,timeout_sec=0.2)

t=buf.lookup_transform("map","base_footprint",rclpy.time.Time()).transform
q=t.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2))
rx,ry=t.translation.x,t.translation.y
print("robot at (%+.3f,%+.3f) yaw %+.1f deg"%(rx,ry,math.degrees(yaw)))
dist=math.hypot(EXPECT[0]-rx,EXPECT[1]-ry)
bear=math.atan2(EXPECT[1]-ry,EXPECT[0]-rx)
off=math.degrees((bear-yaw+math.pi)%(2*math.pi)-math.pi)
print("expected cube (%+.3f,%+.3f): %.3f m away, %+.1f deg off the nose"%(EXPECT[0],EXPECT[1],dist,off))
print("   camera sees floor roughly 0.30-1.34 m ahead, FOV ~58 deg wide")

tr=buf.lookup_transform("map","camera_color_optical_frame",rclpy.time.Time()).transform
qq=tr.rotation
T=tfs.affines.compose([tr.translation.x,tr.translation.y,tr.translation.z],
                      tfs.quaternions.quat2mat([qq.w,qq.x,qq.y,qq.z]),[1,1,1])
model=YOLO("/home/jetson/cuboid_best_baseline.pt")
rgb=d["rgb"]; dep=d["dep"]
res=model.predict(rgb,imgsz=640,conf=0.10,verbose=False)[0]   # LOW conf to see everything
print("\n%d raw detection(s) at conf>=0.10:"%len(res.boxes))
fx,fy,cx0,cy0=K[0],K[4],K[2],K[5]
vis=rgb.copy()
for bx in res.boxes:
    x1,y1,x2,y2=[int(v) for v in bx.xyxy[0]]
    c=float(bx.conf[0])
    mx1,my1=x1+(x2-x1)//4, y1+(y2-y1)//4
    mx2,my2=x2-(x2-x1)//4, y2-(y2-y1)//4
    patch=dep[max(0,my1):my2, max(0,mx1):mx2]
    patch=patch[np.isfinite(patch)&(patch>0)]
    z=float(np.median(patch))*0.001 if patch.size>=10 else -1
    u=(x1+x2)/2.0; vb=float(y2)
    dir_cam=np.array([(u-cx0)/fx,(vb-cy0)/fy,1.0,0.0])
    Cw=(T@np.array([0.0,0.0,0.0,1.0]))[:3]; dw=(T@dir_cam)[:3]
    wx=wy=float("nan")
    if abs(dw[2])>1e-6:
        tt=-Cw[2]/dw[2]
        if tt>0:
            F=Cw+tt*dw; wx,wy=F[0],F[1]
    dd=math.hypot(wx-EXPECT[0],wy-EXPECT[1]) if wx==wx else float("nan")
    print("   conf %.2f  box(%d,%d,%d,%d)  depth %.2f m  -> map (%+.3f,%+.3f)  %.2f m from expected%s"
          %(c,x1,y1,x2,y2,z,wx,wy,dd," <-- would MATCH" if dd==dd and dd<0.40 else ""))
    cv2.rectangle(vis,(x1,y1),(x2,y2),(0,255,0),2)
    cv2.putText(vis,"%.2f"%c,(x1,max(14,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
cv2.imwrite("/tmp/robot_sees.jpg",vis)
print("\nsaved /tmp/robot_sees.jpg")
rclpy.shutdown()
