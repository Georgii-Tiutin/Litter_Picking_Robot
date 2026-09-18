"""Does map->base_footprint survive while the robot is STATIONARY and YOLO is loading the
Orin? That is the exact condition under which AMCL kept silently dying."""
import time, rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros
from ultralytics import YOLO

rclpy.init(); n=Node("tfsurv"); b=CvBridge()
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
d={}
n.create_subscription(Image,"/camera/color/image_raw",lambda m:d.__setitem__("rgb",b.imgmsg_to_cv2(m,"bgr8")),1)
model=YOLO("/home/jetson/cuboid_best_baseline.pt")
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<10e9 and "rgb" not in d: rclpy.spin_once(n,timeout_sec=0.2)
print("robot is stationary; running inference repeatedly for 60 s\n")
ok=fail=0
start=time.time()
while time.time()-start<60:
    for _ in range(8): rclpy.spin_once(n,timeout_sec=0.05)
    if "rgb" in d:
        model.predict(d["rgb"],imgsz=640,conf=0.25,verbose=False)   # load the CPU
    try:
        buf.lookup_transform("map","base_footprint",rclpy.time.Time())
        ok+=1; mark="ok"
    except Exception as e:
        fail+=1; mark="LOST: "+str(e)[:40]
    print("   t=%4.0fs  map->base_footprint %s"%(time.time()-start,mark))
print("\n   %d ok, %d lost over 60 s of inference"%(ok,fail))
print("   %s"%("STABLE - safe to run the mission" if fail==0 else "STILL DROPPING OUT"))
rclpy.shutdown()
