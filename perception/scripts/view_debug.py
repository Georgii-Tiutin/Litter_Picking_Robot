#!/usr/bin/env python3
"""Tiny viewer for /cube/debug_image (or any other Image topic).

Uses cv2.imshow which is known to work on the robot's local display
(same path the hsv_tuner uses). image_view / rqt_image_view have been
finicky on this setup, so this script is the reliable fallback.

Usage on the robot:
  source /opt/ros/humble/setup.bash
  source ~/yahboomcar_ws/install/setup.bash
  export ROS_DOMAIN_ID=30
  python3 ~/project0/perception/scripts/view_debug.py             # default /cube/debug_image
  python3 ~/project0/perception/scripts/view_debug.py /other/topic # any Image topic
Press 'q' in the window or Ctrl-C to quit.
"""

import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Viewer(Node):
    def __init__(self, topic):
        super().__init__("debug_image_viewer")
        self.bridge = CvBridge()
        self.frame = None
        self.topic = topic
        self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"subscribed to {topic}")

    def cb(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/cube/debug_image"
    rclpy.init()
    node = Viewer(topic)
    win = f"viewer:{topic}"
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is not None:
                cv2.imshow(win, node.frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
