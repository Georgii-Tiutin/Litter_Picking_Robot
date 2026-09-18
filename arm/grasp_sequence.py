#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Full grasp sequence from ONE persistent node (reliable command delivery).

Spawning a fresh node per command drops the first message during DDS
discovery, so the arm doesn't move. This node does discovery once, then every
command lands. Also resends each command a few times as belt-and-suspenders.

Sequence: observation -> descend(open) -> close(grasp) -> hold -> release ->
back to observation. The grasp re-seats the cube square to the jaws, so on
release it's left in the jaw-aligned "ideal" pose.

Usage: python3 grasp_sequence.py [motor5] [hold_secs]
  motor5   wrist roll used during the grasp (default 90)
  hold_secs seconds to hold the closed grip (default 10)
"""
import sys
import time
import rclpy
from arm_msgs.msg import ArmJoints, ArmJoint

OBS = [90, 45, 45, 0, 90, 0]
GRIP_OPEN = 0
GRIP_CLOSE = 145

M5 = int(sys.argv[1]) if len(sys.argv) > 1 else 90
HOLD = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0


def main():
    rclpy.init()
    node = rclpy.create_node("grasp_sequence")
    pub_joints = node.create_publisher(ArmJoints, "/arm6_joints", 10)
    pub_joint = node.create_publisher(ArmJoint, "/arm_joint", 10)
    time.sleep(1.5)  # DDS discovery once

    def pose(j, t=2000, reps=3):
        m = ArmJoints()
        m.joint1, m.joint2, m.joint3, m.joint4, m.joint5, m.joint6 = [int(v) for v in j]
        m.time = int(t)
        for _ in range(reps):
            pub_joints.publish(m); time.sleep(0.15)
        node.get_logger().info("pose -> %s (t=%d)" % (j, t))

    def grip(val, t=800, reps=3):
        m = ArmJoint(); m.id = 6; m.joint = int(val); m.time = int(t)
        for _ in range(reps):
            pub_joint.publish(m); time.sleep(0.15)
        node.get_logger().info("grip -> %d" % val)

    descend = [90, 0, 90, 0, M5, GRIP_OPEN]

    node.get_logger().info("[0] observation"); pose(OBS, 2000); time.sleep(2.5)
    node.get_logger().info("[1] descend (open)"); pose(descend, 2000); time.sleep(3.0)
    node.get_logger().info("[2] close -> grasp"); grip(GRIP_CLOSE, 800); time.sleep(2.0)
    node.get_logger().info("[3] hold %.1fs" % HOLD); time.sleep(HOLD)
    node.get_logger().info("[4] release"); grip(GRIP_OPEN, 800); time.sleep(2.0)
    node.get_logger().info("[5] back to observation"); pose(OBS, 2000); time.sleep(3.0)
    node.get_logger().info("DONE")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
