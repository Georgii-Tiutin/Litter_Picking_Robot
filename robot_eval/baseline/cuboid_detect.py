#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run a custom YOLO model (best.pt) on the M3 Pro depth-camera color stream.

Publishes annotated frames to /detect_image, logs per-frame detections,
and saves the latest annotated frame to ~/models/cuboid_v1/out/latest.jpg
(plus a few sample hits) so it can be verified headless over SSH.
"""
import os
import time

import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MODEL_PATH = os.path.expanduser(os.environ.get("CUBOID_MODEL", "~/models/cuboid_v1/best.pt"))
OUT_DIR = os.path.expanduser(os.environ.get("CUBOID_OUT", "~/models/cuboid_v1/out"))
CONF = float(os.environ.get("CUBOID_CONF", "0.25"))
TOPIC = os.environ.get("CUBOID_TOPIC", "detect_image")
LABEL = os.environ.get("CUBOID_LABEL", "")
NODE_NAME = os.environ.get("CUBOID_NODE", "cuboid_detect_node")

os.makedirs(OUT_DIR, exist_ok=True)
model = YOLO(MODEL_PATH)


class CuboidDetect(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, TOPIC, 1)
        self.sub = self.create_subscription(
            Image, "/camera/color/image_raw", self.cb, 10
        )
        self.frames = 0
        self.hits = 0
        self.t0 = time.time()
        self.get_logger().info(f"Loaded model: {MODEL_PATH} (conf={CONF}); waiting for frames...")

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = model.predict(frame, conf=CONF, device=0, verbose=False)
        r = results[0]
        annotated = r.plot()  # BGR
        self.frames += 1
        n = len(r.boxes)
        if LABEL:
            txt = f"{LABEL}  |  {n} det"
            cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 34), (0, 0, 0), -1)
            cv2.putText(annotated, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2, cv2.LINE_AA)
        self.pub.publish(self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))

        if n:
            self.hits += 1
            names = r.names
            dets = [
                f"{names[int(b.cls)]}:{float(b.conf):.2f}" for b in r.boxes
            ]
            self.get_logger().info(f"frame {self.frames}: {n} det -> {', '.join(dets)}")
            if self.hits <= 5:
                cv2.imwrite(os.path.join(OUT_DIR, f"hit_{self.hits:02d}.jpg"), annotated)

        cv2.imwrite(os.path.join(OUT_DIR, "latest.jpg"), annotated)

        if self.frames % 30 == 0:
            fps = self.frames / (time.time() - self.t0)
            self.get_logger().info(
                f"[stats] frames={self.frames} hits={self.hits} ~{fps:.1f} fps"
            )


def main():
    rclpy.init()
    node = CuboidDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
