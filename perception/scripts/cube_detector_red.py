#!/usr/bin/env python3
"""Dual-track cuboid detector (Phase 1.2) — RED CUBOID variant.

Original red cuboid detector preserved for reference and for sessions
where the red cuboid is in use instead of the blue one. Tuned 2026-05-04
against the red 30x30x60 mm cuboid with tag36h11 ID=3 on one 30x30 face.
Use cube_detector.py (blue variant) as the active detector going forward.

Track A — AprilTag (preferred):
  dt_apriltags detector for tag36h11 ID=3 on /camera/color/image_raw.
  Outputs full 6-DOF pose. class_id="apriltag_3".

Track B — HSV red segmentation (always-on fallback):
  Loads HSV thresholds from ~/project0/perception/config/hsv_red.yaml
  (tunable via hsv_tuner_red.py). Largest contour passing the area
  filter is taken as the cuboid. Centroid depth-lifted via
  /camera/depth/image_raw + camera_info intrinsics. Yaw inferred from
  cv2.minAreaRect. class_id="hsv_red".

Priority: emit Track A if tag detected this frame, else emit Track B.
Both share the output topic /cube/detections (vision_msgs/Detection3DArray).
"""

import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from dt_apriltags import Detector
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

CUBOID_TAG_FAMILY = "tag36h11"
CUBOID_TAG_ID = 3            # red cuboid: tag36h11 ID=3 (probed 2026-05-04)
CUBOID_TAG_SIZE_M = 0.019    # 19 mm black-square edge

# Cuboid physical dimensions (m)
CUBOID_W = 0.030  # short axis
CUBOID_L = 0.060  # long axis

HSV_CFG_PATH = Path("/home/jetson/project0/perception/config/hsv_red.yaml")
HSV_DEFAULTS = {
    "h_low_1": 0,    "h_high_1": 10,
    "h_low_2": 170,  "h_high_2": 179,
    "s_min": 100,    "s_max": 255,
    "v_min": 60,     "v_max": 255,
    "morph_kernel": 5,
    "min_area": 500,
    "aspect_min_x10": 14,  # cuboid projected aspect ≥ 1.4
    "aspect_max_x10": 35,  # cuboid projected aspect ≤ 3.5
}


def load_hsv_cfg():
    if HSV_CFG_PATH.exists():
        try:
            cfg = yaml.safe_load(HSV_CFG_PATH.read_text()) or {}
            return {**HSV_DEFAULTS, **cfg}
        except Exception:
            pass
    return HSV_DEFAULTS.copy()


class CubeDetector(Node):
    def __init__(self):
        super().__init__("cube_detector")
        self.bridge = CvBridge()
        self.detector = Detector(
            families=CUBOID_TAG_FAMILY, nthreads=2,
            quad_decimate=1.0, refine_edges=True,
        )
        self.K = None
        self.frame_id = "camera_color_optical_frame"
        self.latest_color = None
        self.latest_depth = None
        self.latest_color_stamp = None
        self.hsv = load_hsv_cfg()
        self.get_logger().info(f"hsv config: {self.hsv}")

        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self.cb_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self.cb_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self.cb_depth, qos_profile_sensor_data)
        self.pub = self.create_publisher(Detection3DArray,
                                         "/cube/detections", 10)
        self.viz_pub = self.create_publisher(Image, "/cube/debug_image", 5)
        self.last_log_t = 0.0

    def cb_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.frame_id = msg.header.frame_id or self.frame_id
            self.get_logger().info(
                f"camera_info: fx={self.K[0,0]:.2f} cx={self.K[0,2]:.2f} "
                f"frame={self.frame_id}"
            )

    def cb_depth(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"cv_bridge depth: {e}")

    def cb_color(self, msg):
        if self.K is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge color: {e}")
            return
        self.latest_color = img
        self.latest_color_stamp = msg.header.stamp

        # Track A: AprilTag
        tag_det = self.try_apriltag(img)
        # Track B: HSV (always run for visibility / debug)
        hsv_det = self.try_hsv(img)
        chosen = tag_det if tag_det is not None else hsv_det

        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        if chosen is not None:
            out.detections.append(chosen)
        self.pub.publish(out)

        # Debug image
        viz = self.make_debug_image(img, tag_det, hsv_det)
        try:
            viz_msg = self.bridge.cv2_to_imgmsg(viz, encoding="bgr8")
            viz_msg.header.stamp = msg.header.stamp
            viz_msg.header.frame_id = self.frame_id
            self.viz_pub.publish(viz_msg)
        except Exception:
            pass

        now = time.monotonic()
        if now - self.last_log_t > 2.0:
            self.last_log_t = now
            track = ("apriltag" if tag_det else "hsv" if hsv_det else "none")
            self.get_logger().info(f"track={track}")

    # ------------------------------------------------------------------
    # Track A: AprilTag
    # ------------------------------------------------------------------
    def try_apriltag(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cam = (self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2])
        try:
            dets = self.detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=cam, tag_size=CUBOID_TAG_SIZE_M,
            )
        except Exception as e:
            self.get_logger().error(f"apriltag detect: {e}")
            return None
        for d in dets:
            if d.tag_id != CUBOID_TAG_ID:
                continue
            R = np.array(d.pose_R)
            t = np.array(d.pose_t).flatten()
            # The tag pose's z-axis points OUT of the tag face (toward camera).
            # The cuboid's geometric centre is half its short dim BEHIND the
            # tagged face (i.e. -CUBOID_W/2 along the tag's z-axis in tag frame).
            centre_in_tag = np.array([0.0, 0.0, -CUBOID_W / 2.0])
            centre_in_cam = R @ centre_in_tag + t

            det = Detection3D()
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = f"apriltag_{CUBOID_TAG_ID}"
            hyp.hypothesis.score = float(min(1.0, d.decision_margin / 100.0))
            hyp.pose.pose = self.rt_to_pose(R, centre_in_cam)
            # Tight covariance: ~5 mm / 1 deg from calibration analysis
            hyp.pose.covariance = self.diag_cov(
                pos_sigma=0.005, rot_sigma=math.radians(1.0)
            )
            det.results.append(hyp)
            det.bbox = self.bbox_aligned_to_pose(R, centre_in_cam)
            return det
        return None

    # ------------------------------------------------------------------
    # Track B: HSV red
    # ------------------------------------------------------------------
    def try_hsv(self, img):
        if self.latest_depth is None:
            return None
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv,
            (self.hsv["h_low_1"], self.hsv["s_min"], self.hsv["v_min"]),
            (self.hsv["h_high_1"], self.hsv["s_max"], self.hsv["v_max"]))
        m2 = cv2.inRange(hsv,
            (self.hsv["h_low_2"], self.hsv["s_min"], self.hsv["v_min"]),
            (self.hsv["h_high_2"], self.hsv["s_max"], self.hsv["v_max"]))
        mask = cv2.bitwise_or(m1, m2)
        ks = max(1, int(self.hsv["morph_kernel"]) | 1)
        kernel = np.ones((ks, ks), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        amin = self.hsv["aspect_min_x10"] / 10.0
        amax = self.hsv["aspect_max_x10"] / 10.0
        # Filter by area + aspect ratio, keep largest survivor
        candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.hsv["min_area"]:
                continue
            rect_c = cv2.minAreaRect(c)
            (_, _), (wc, hc), _ = rect_c
            asp = max(wc, hc) / max(1.0, min(wc, hc))
            if not (amin <= asp <= amax):
                continue
            candidates.append((area, c, rect_c))
        if not candidates:
            return None
        area, c, rect = max(candidates, key=lambda x: x[0])
        (cx, cy), (w_px, h_px), ang_deg = rect
        # Lift centroid to 3D using the depth at (cx, cy)
        depth = self.latest_depth
        xi, yi = int(cx), int(cy)
        if not (0 <= xi < depth.shape[1] and 0 <= yi < depth.shape[0]):
            return None
        # Sample a 5x5 patch median for noise-robustness
        x0, y0 = max(0, xi - 2), max(0, yi - 2)
        x1, y1 = min(depth.shape[1], xi + 3), min(depth.shape[0], yi + 3)
        patch = depth[y0:y1, x0:x1]
        if patch.size == 0:
            return None
        valid = patch[(patch > 0)]
        if valid.size < 4:
            return None
        z_mm = float(np.median(valid))
        # Orbbec depth is uint16 mm.
        z_m = z_mm / 1000.0 if z_mm > 50 else z_mm  # already metres if very small
        if z_m <= 0.05 or z_m > 2.0:
            return None
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx_int, cy_int = self.K[0, 2], self.K[1, 2]
        X = (cx - cx_int) * z_m / fx
        Y = (cy - cy_int) * z_m / fy
        Z = z_m
        # Yaw from contour orientation: camera-frame yaw of the long axis.
        # cv2.minAreaRect angle is in degrees, in [-90, 0).
        yaw_deg = ang_deg if w_px >= h_px else ang_deg + 90.0
        yaw_rad = math.radians(yaw_deg)
        # Rotation: assume cuboid lying flat (long axis horizontal in image),
        # yaw rotation about camera Z (out-of-image) axis.
        cz, sz = math.cos(yaw_rad), math.sin(yaw_rad)
        R = np.array([[cz, -sz, 0.0],
                      [sz,  cz, 0.0],
                      [0.0, 0.0, 1.0]])

        det = Detection3D()
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "hsv_red"
        # Score: clipped contour area, normalised to 1.0 at 5000 px^2
        hyp.hypothesis.score = float(min(1.0, area / 5000.0))
        hyp.pose.pose = self.rt_to_pose(R, np.array([X, Y, Z]))
        # Looser covariance: HSV+depth is ~20 mm / 10 deg under good lighting.
        hyp.pose.covariance = self.diag_cov(
            pos_sigma=0.020, rot_sigma=math.radians(10.0)
        )
        det.results.append(hyp)
        det.bbox = self.bbox_aligned_to_pose(R, np.array([X, Y, Z]))
        return det

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def rt_to_pose(self, R, t):
        # Convert 3x3 rotation + 3-vector to geometry_msgs/Pose (quaternion).
        q = self.rot_to_quat(R)
        p = Pose()
        p.position.x = float(t[0])
        p.position.y = float(t[1])
        p.position.z = float(t[2])
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        return p

    @staticmethod
    def rot_to_quat(R):
        # Standard matrix-to-quaternion (returns x,y,z,w).
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return float(qx), float(qy), float(qz), float(qw)

    @staticmethod
    def diag_cov(pos_sigma, rot_sigma):
        cov = [0.0] * 36
        for i, s in enumerate([pos_sigma] * 3 + [rot_sigma] * 3):
            cov[i * 6 + i] = s * s
        return cov

    @staticmethod
    def bbox_aligned_to_pose(R, centre):
        b = BoundingBox3D()
        b.center.position.x = float(centre[0])
        b.center.position.y = float(centre[1])
        b.center.position.z = float(centre[2])
        # Bounding box rotation matches detection rotation
        q = CubeDetector.rot_to_quat(R)
        b.center.orientation.x = q[0]
        b.center.orientation.y = q[1]
        b.center.orientation.z = q[2]
        b.center.orientation.w = q[3]
        b.size.x = CUBOID_L
        b.size.y = CUBOID_W
        b.size.z = CUBOID_W
        return b

    def make_debug_image(self, img, tag_det, hsv_det):
        out = img.copy()
        h_, w_ = out.shape[:2]
        if tag_det is not None:
            cv2.putText(out, "track: APRILTAG (tag36h11 id=3)", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 230, 0), 2)
            p = tag_det.results[0].pose.pose.position
            cv2.putText(out,
                        f"pos: ({p.x*1000:.0f}, {p.y*1000:.0f}, {p.z*1000:.0f}) mm",
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
        elif hsv_det is not None:
            cv2.putText(out, "track: HSV red (fallback)", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 230), 2)
            p = hsv_det.results[0].pose.pose.position
            cv2.putText(out,
                        f"pos: ({p.x*1000:.0f}, {p.y*1000:.0f}, {p.z*1000:.0f}) mm",
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
        else:
            cv2.putText(out, "track: NONE", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 230), 2)
        return out


def main():
    rclpy.init()
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
