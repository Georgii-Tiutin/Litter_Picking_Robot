import sys, time
import rclpy
from rclpy.node import Node
from arm_msgs.msg import ArmJoints

class P(Node):
    def __init__(self, pose, t=2000):
        super().__init__("set_pose")
        self.pub = self.create_publisher(ArmJoints, "/arm6_joints", 10)
        time.sleep(1.0)
        m = ArmJoints()
        m.joint1, m.joint2, m.joint3, m.joint4, m.joint5, m.joint6 = [int(v) for v in pose]
        m.time = int(t)
        for _ in range(3):
            self.pub.publish(m); time.sleep(0.2)
        self.get_logger().info("sent %s t=%d" % (pose, t))
        time.sleep(0.5)

pose = [int(x) for x in sys.argv[1:7]] if len(sys.argv) >= 7 else [90,145,90,45,90,0]
rclpy.init(); P(pose); rclpy.shutdown()
