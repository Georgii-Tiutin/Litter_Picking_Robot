#!/usr/bin/env python3
"""Publish raw servo angles. usage: set_joints.py J1 J2 J3 J4 J5 J6 [TIME_MS]"""
import sys, time, rclpy
from rclpy.node import Node
from arm_msgs.msg import ArmJoints


def wait_for_subscriber(node, pub, timeout=5.0):
    """ROS2 discovery is not instant: publishing before the subscriber is
    matched silently drops the message. Wait for the match first."""
    t0 = time.time()
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 > timeout:
            print("WARN: no subscriber on arm6_joints after %.1fs" % timeout)
            return False
    return True

def main():
    v = [int(a) for a in sys.argv[1:7]]
    tms = int(sys.argv[7]) if len(sys.argv) > 7 else 2500
    rclpy.init()
    n = Node("calib_setjoints")
    pub = n.create_publisher(ArmJoints, "arm6_joints", 10)
    m = ArmJoints()
    m.joint1, m.joint2, m.joint3, m.joint4, m.joint5, m.joint6 = v
    m.time = tms
    wait_for_subscriber(n, pub)
    for _ in range(5):
        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.05); time.sleep(0.15)
    print(f"sent {v} time={tms}")
    time.sleep(tms/1000.0 + 0.5)

if __name__ == "__main__":
    main()
