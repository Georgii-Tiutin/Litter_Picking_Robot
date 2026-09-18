#!/usr/bin/env python3
"""
arm_state_pub.py — Publish /joint_states for the M3PRO arm at a fixed (commanded) pose,
so RViz's RobotModel arm matches the real arm. The arm is open-loop (no feedback), and
the vendor's dummy joint_state_publisher forces all joints to 0 — kill that first, then run
this. Servo-degree -> URDF-radian mapping is (deg - 90) * pi/180 (kin_srv's convention).

Usage:  python3 arm_state_pub.py --joints 90 40 60 20 90   # center observe pose
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINT_NAMES = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joiint", "arm5_Joint"]
D2U = math.pi / 180.0


class ArmStatePub(Node):
    def __init__(self, joints):
        super().__init__("arm_state_pub")
        self.positions = [(d - 90) * D2U for d in joints[:5]]
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"publishing /joint_states {ARM_JOINT_NAMES} = "
            f"{[round(p, 3) for p in self.positions]} rad"
        )

    def _tick(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM_JOINT_NAMES
        js.position = self.positions
        self.pub.publish(js)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joints", type=float, nargs=5, default=[90, 40, 60, 20, 90],
                    help="servo degrees for arm joints 1..5")
    args = ap.parse_args()
    rclpy.init()
    node = ArmStatePub(args.joints)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
