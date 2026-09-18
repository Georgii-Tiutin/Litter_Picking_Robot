import sys, time, math, threading, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
dist_cm = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5   # + fwd, - back
speed   = float(sys.argv[2]) if len(sys.argv) > 2 else 0.07  # m/s cruise
target = abs(dist_cm) / 100.0
rclpy.init()
n = rclpy.create_node("drive_base")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
od = {"x": None, "y": None}
def cb(m):
    od["x"] = m.pose.pose.position.x; od["y"] = m.pose.pose.position.y
n.create_subscription(Odometry, "/odom_raw", cb, 10)
stop = False
def spin():
    while not stop: rclpy.spin_once(n, timeout_sec=0.02)
th = threading.Thread(target=spin); th.start()
t0 = time.time()
while od["x"] is None and time.time()-t0 < 3: time.sleep(0.05)
if od["x"] is None:
    print("NO ODOM"); stop=True; th.join(); rclpy.shutdown(); sys.exit(1)
x0, y0 = od["x"], od["y"]
tw = Twist(); tw.linear.x = speed if dist_cm >= 0 else -speed
moved = 0.0; t0 = time.time()
while moved < target and time.time()-t0 < 6.0:
    pub.publish(tw); time.sleep(0.02)
    moved = math.hypot(od["x"]-x0, od["y"]-y0)
z = Twist()
for _ in range(20): pub.publish(z); time.sleep(0.02)
time.sleep(0.3)
final = math.hypot(od["x"]-x0, od["y"]-y0)
print("target=%.1fcm moved=%.1fcm (incl coast)" % (dist_cm, final*100*(1 if dist_cm>=0 else 1)))
stop = True; th.join(); rclpy.shutdown()
