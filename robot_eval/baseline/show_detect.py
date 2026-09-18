#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Display the /detect_image stream in a window on the robot's monitor."""
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

WIN = "Cuboid Detection (best.pt)  -  press q to close"


class Viewer(Node):
    def __init__(self):
        super().__init__("detect_viewer")
        self.bridge = CvBridge()
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, 1280, 960)
        self.create_subscription(Image, "/detect_image", self.cb, 1)

    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow(WIN, img)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            rclpy.shutdown()


def main():
    rclpy.init()
    n = Viewer()
    try:
        rclpy.spin(n)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
