import sys, time
import rclpy
from rclpy.node import Node
from arm_msgs.msg import ArmJoint

jid = int(sys.argv[1]); val = int(sys.argv[2]); t = int(sys.argv[3]) if len(sys.argv) > 3 else 1000
rclpy.init()
n = rclpy.create_node("set_joint")
pub = n.create_publisher(ArmJoint, "/arm_joint", 10)
time.sleep(0.8)
m = ArmJoint(); m.id = jid; m.joint = val; m.time = t
for _ in range(3):
    pub.publish(m); time.sleep(0.15)
n.get_logger().info("joint %d -> %d (t=%d)" % (jid, val, t))
time.sleep(0.4)
rclpy.shutdown()
