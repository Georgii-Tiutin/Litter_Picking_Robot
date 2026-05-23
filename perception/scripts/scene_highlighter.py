#!/usr/bin/env python3
"""Recolour the Orbbec full-scene registered RGB-D point cloud:

  - points whose corresponding pixel falls inside the adaptive HSV
    blue mask → saturated blue
  - everything else → mid-grey
  - NaN / invalid depth points → dropped entirely

Publishes sensor_msgs/PointCloud2 on /scene/cube_highlighted, throttled
to ~15 Hz to keep RViz / network load reasonable.

Inputs:
  /camera/color/image_raw             (sensor_msgs/Image, bgr8)
  /camera/depth/image_raw             (sensor_msgs/Image, 16UC1 mm)
  /camera/depth_registered/points     (sensor_msgs/PointCloud2, organised
                                       at the colour resolution)

The colour image and registered cloud are time-synced with
ApproximateTimeSynchronizer (slop 0.05 s). Depth is taken as the latest
arrived — only the adaptive mask uses it, and its values change slowly
enough that frame-perfect sync is unnecessary.
"""

import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from blue_mask import HSV_DEFAULTS, compute_blue_mask  # noqa: E402


HSV_CFG_PATH = Path("/home/jetson/project0/perception/config/hsv_blue.yaml")

# BGR triplets for the two output colours.
CUBE_BGR = (255, 64, 0)      # saturated blue
GREY_BGR = (128, 128, 128)   # mid grey

PUBLISH_RATE_HZ = 15.0


def _pack_rgb(bgr):
    b, g, r = bgr
    return (int(r) << 16) | (int(g) << 8) | int(b)


def _pf_format(pf_datatype):
    """Map a PointField datatype enum value to a numpy dtype string."""
    return {
        PointField.INT8:    "i1",
        PointField.UINT8:   "u1",
        PointField.INT16:   "<i2",
        PointField.UINT16:  "<u2",
        PointField.INT32:   "<i4",
        PointField.UINT32:  "<u4",
        PointField.FLOAT32: "<f4",
        PointField.FLOAT64: "<f8",
    }.get(pf_datatype, "u1")


def cloud_to_xyz(cloud_msg):
    """Zero-copy extract (N,3) float32 XYZ from a PointCloud2 message."""
    names = [f.name for f in cloud_msg.fields]
    offsets = [f.offset for f in cloud_msg.fields]
    formats = [_pf_format(f.datatype) for f in cloud_msg.fields]
    dt = np.dtype({
        "names": names, "formats": formats,
        "offsets": offsets, "itemsize": cloud_msg.point_step,
    })
    pts = np.frombuffer(cloud_msg.data, dtype=dt)
    return np.stack([pts["x"], pts["y"], pts["z"]], axis=1)


class SceneHighlighter(Node):
    def __init__(self):
        super().__init__("scene_highlighter")
        self.bridge = CvBridge()
        self.latest_depth_mm = None
        self.last_pub_t = 0.0
        self.publish_period = 1.0 / PUBLISH_RATE_HZ

        self.hsv = self._load_hsv()
        self.get_logger().info(f"hsv config: {self.hsv}")

        # Pre-packed colour constants
        self._rgb_cube = np.uint32(_pack_rgb(CUBE_BGR))
        self._rgb_grey = np.uint32(_pack_rgb(GREY_BGR))

        self.create_subscription(
            Image, "/camera/depth/image_raw",
            self.cb_depth, qos_profile_sensor_data,
        )

        color_sub = Subscriber(
            self, Image, "/camera/color/image_raw",
            qos_profile=qos_profile_sensor_data,
        )
        cloud_sub = Subscriber(
            self, PointCloud2, "/camera/depth_registered/points",
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = ApproximateTimeSynchronizer(
            [color_sub, cloud_sub], queue_size=5, slop=0.05,
        )
        self.sync.registerCallback(self.cb_sync)

        self.pub_cloud = self.create_publisher(
            PointCloud2, "/scene/cube_highlighted", 2,
        )
        self.get_logger().info(
            f"scene_highlighter ready, throttled to {PUBLISH_RATE_HZ:.0f} Hz"
        )

    # ------------------------------------------------------------------
    def _load_hsv(self):
        if HSV_CFG_PATH.exists():
            try:
                cfg = yaml.safe_load(HSV_CFG_PATH.read_text()) or {}
                return {**HSV_DEFAULTS, **cfg}
            except Exception as e:
                self.get_logger().warn(f"hsv yaml load failed: {e}")
        return HSV_DEFAULTS.copy()

    # ------------------------------------------------------------------
    def cb_depth(self, msg):
        try:
            self.latest_depth_mm = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        except Exception as e:
            self.get_logger().error(f"cv_bridge depth: {e}")

    # ------------------------------------------------------------------
    def cb_sync(self, color_msg, cloud_msg):
        # Throttle: skip frames cheaply before doing any decoding work.
        now = time.monotonic()
        if now - self.last_pub_t < self.publish_period:
            return
        self.last_pub_t = now

        try:
            img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge color: {e}")
            return

        H, W = img.shape[:2]
        if cloud_msg.width * cloud_msg.height != H * W:
            self.get_logger().warn(
                f"cloud size {cloud_msg.width}x{cloud_msg.height} ≠ "
                f"image {W}x{H}; skipping (registration mismatch)"
            )
            return

        # Adaptive blue mask
        mask = compute_blue_mask(img, self.latest_depth_mm, self.hsv)
        mask_flat = (mask > 0).reshape(-1)

        # Extract XYZ; drop invalid points (NaN / inf, or Z<=0).
        xyz = cloud_to_xyz(cloud_msg)
        finite = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
        if not finite.any():
            return
        xyz = xyz[finite]
        is_cube = mask_flat[finite]
        n = xyz.shape[0]

        rgb = np.where(is_cube, self._rgb_cube, self._rgb_grey).astype(np.uint32)

        buf = np.empty(n, dtype=[
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4"),
        ])
        buf["x"] = xyz[:, 0].astype(np.float32)
        buf["y"] = xyz[:, 1].astype(np.float32)
        buf["z"] = xyz[:, 2].astype(np.float32)
        buf["rgb"] = rgb

        out = PointCloud2()
        out.header = cloud_msg.header
        out.height = 1
        out.width = n
        out.fields = [
            PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * n
        out.is_dense = True
        out.data = buf.tobytes()
        self.pub_cloud.publish(out)


def main():
    rclpy.init()
    node = SceneHighlighter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
