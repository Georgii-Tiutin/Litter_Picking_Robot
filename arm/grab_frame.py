import sys, rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/frame.jpg"
rclpy.init(); n = rclpy.create_node("grab"); br = CvBridge()
got = {}
def cb(m): got["f"] = br.imgmsg_to_cv2(m, "bgr8")
s = n.create_subscription(Image, "/camera/color/image_raw", cb, 10)
import time; t0=time.time()
while "f" not in got and time.time()-t0 < 5: rclpy.spin_once(n, timeout_sec=0.2)
if "f" in got:
    cv2.imwrite(out, got["f"]); print("saved", out, got["f"].shape)
else: print("NO FRAME")
rclpy.shutdown()
