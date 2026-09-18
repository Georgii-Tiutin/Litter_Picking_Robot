#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Show N detection streams side-by-side in one window on the robot monitor."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

WIN = "best.pt  |  v3-clean  |  two-stage   (same feed)  -  press q to close"
TOPICS = ["/detect_best", "/detect_v3clean", "/detect_twostage"]


class Compare(Node):
    def __init__(self):
        super().__init__("compare_viewer")
        self.bridge = CvBridge()
        self.frames = {t: None for t in TOPICS}
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, 1920, 500)
        for t in TOPICS:
            self.create_subscription(Image, t, self._mk(t), 1)
        self.create_timer(0.03, self.render)

    def _mk(self, topic):
        def cb(msg):
            self.frames[topic] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        return cb

    def render(self):
        h, w = 480, 640
        blank = np.zeros((h, w, 3), np.uint8)
        panels = []
        for t in TOPICS:
            f = self.frames[t]
            panels.append(cv2.resize(f, (w, h)) if f is not None else blank.copy())
            panels.append(np.full((h, 4, 3), 60, np.uint8))
        combo = np.hstack(panels[:-1])
        cv2.imshow(WIN, combo)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            rclpy.shutdown()


def main():
    rclpy.init()
    n = Compare()
    try:
        rclpy.spin(n)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
