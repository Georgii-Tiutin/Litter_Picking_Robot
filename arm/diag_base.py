import rclpy, time, threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rclpy.init()
n = rclpy.create_node("diag_base")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
od = {"vx": 0.0, "x": 0.0, "got": False}
def cb(m):
    od["vx"] = m.twist.twist.linear.x; od["x"] = m.pose.pose.position.x; od["got"] = True
n.create_subscription(Odometry, "/odom_raw", cb, 10)
time.sleep(1.0)
stop = False
def spin():
    while not stop: rclpy.spin_once(n, timeout_sec=0.02)
th = threading.Thread(target=spin); th.start()
time.sleep(0.5)
print("odom alive:", od["got"], "x0=%.3f" % od["x"])
tw = Twist(); tw.linear.x = 0.12
t0 = time.time()
while time.time() - t0 < 1.0:
    pub.publish(tw); time.sleep(0.1)
    print("cmd vx=0.12 -> odom vx=%.3f x=%.3f" % (od["vx"], od["x"]))
z = Twist()
for _ in range(15): pub.publish(z); time.sleep(0.02)
stop = True; th.join()
print("x_final=%.3f" % od["x"])
rclpy.shutdown()
