#!/usr/bin/env python3
"""Dual-track cuboid detector (Phase 1.2) — BLUE CUBOID variant (active).

Track A — AprilTag (preferred):
  dt_apriltags detector for tag36h11 ID=1 on /camera/color/image_raw.
  20 mm black-square edge on a 45x45 mm face. Outputs full 6-DOF pose.
  class_id="apriltag_1".

Track B — HSV blue segmentation (always-on fallback):
  Loads HSV thresholds from ~/project0/perception/config/hsv_blue.yaml
  (tunable via hsv_tuner.py). Largest contour passing the area + aspect
  filter is taken as the cuboid. Centroid depth-lifted via
  /camera/depth/image_raw + camera_info intrinsics. Yaw inferred from
  cv2.minAreaRect. class_id="hsv_blue".

Priority: emit Track A if tag detected this frame, else emit Track B.
Both share the output topic /cube/detections (vision_msgs/Detection3DArray).

For the prior red cuboid configuration see cube_detector_red.py.
"""

import argparse
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from dt_apriltags import Detector

# Allow the sibling helper module to be imported regardless of cwd.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from blue_mask import compute_blue_mask as _shared_compute_blue_mask  # noqa: E402
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

CUBOID_TAG_FAMILY = "tag36h11"
CUBOID_TAG_ID = 1            # blue cuboid (probed 2026-05-08, margin 92, 100% lock)
CUBOID_TAG_SIZE_M = 0.020    # 20 mm black-square edge (user-measured)

# Cuboid physical dimensions (m).
# Original spec was 30x30x60 mm but a re-measurement of the tag (20 mm
# vs probe-implied 13.3 mm) showed the cuboid is ~1.5x larger than
# initially reported. Updated 2026-05-08.
CUBOID_W = 0.045  # short axis (was 0.030)
CUBOID_L = 0.090  # long axis  (was 0.060)

HSV_CFG_PATH = Path("/home/jetson/project0/perception/config/hsv_blue.yaml")
# Blue is single-band in OpenCV (no wraparound like red). Both H ranges
# are set to the same blue band; the OR'd mask is identical to a single
# inRange. Kept the dual-range structure so the tuner UI is unchanged.
HSV_DEFAULTS = {
    "h_low_1": 100,  "h_high_1": 130,
    "h_low_2": 100,  "h_high_2": 130,
    "s_min": 80,     "s_max": 255,
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


class PoseTracker:
    """Tier-1 short-term pose memory.

    Stores recent (t, centroid, R) tuples. Position is extrapolated linearly
    from the last two samples; rotation is held at the last good value
    (extrapolating rotation reliably is harder and the gain is small for
    cube-like objects over <1 s windows). If the time gap since the last
    update exceeds `gap_reset_s`, the velocity estimate is discarded so we
    don't carry stale motion across a pause.
    """

    def __init__(self, history_size=10, gap_reset_s=1.0):
        self.history = deque(maxlen=history_size)
        self.gap_reset_s = float(gap_reset_s)

    def update(self, t, centroid, R):
        if self.history and (t - self.history[-1][0]) > self.gap_reset_s:
            self.history.clear()
        self.history.append((
            float(t),
            np.asarray(centroid, dtype=float).copy(),
            np.asarray(R, dtype=float).copy(),
        ))

    def is_stale(self, t, max_age_s):
        if not self.history:
            return True
        return (t - self.history[-1][0]) > max_age_s

    def last_age(self, t):
        if not self.history:
            return float("inf")
        return t - self.history[-1][0]

    def predict(self, t):
        if not self.history:
            return None
        t_last, c_last, R_last = self.history[-1]
        if len(self.history) < 2:
            return c_last, R_last
        t_prev, c_prev, _ = self.history[-2]
        dt = t_last - t_prev
        if dt <= 1e-6:
            return c_last, R_last
        v = (c_last - c_prev) / dt
        c_pred = c_last + v * (t - t_last)
        return c_pred, R_last


class CubeDetector(Node):
    def __init__(self, show_display=True):
        super().__init__("cube_detector")
        self.show_display = show_display
        self.preview_window_open = False
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
        # Per-frame viz state from try_hsv, consumed by make_debug_image.
        # Tuple of (contour_or_None, R_3x3, centroid_in_cam) or None.
        # contour is None when the entry is a predicted (held) pose.
        self._hsv_viz = None

        # Tier-1 short-term pose memory.
        gap_reset = float(self.hsv.get("predict_gap_reset_s", 1.0))
        self.tracker = PoseTracker(history_size=10, gap_reset_s=gap_reset)

        # Diagnostic logging throttle (try_hsv early-returns).
        self._diag_last_t = 0.0
        self._diag_period_s = 1.0

        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self.cb_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self.cb_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self.cb_depth, qos_profile_sensor_data)
        self.pub = self.create_publisher(Detection3DArray,
                                         "/cube/detections", 10)
        self.viz_pub = self.create_publisher(Image, "/cube/debug_image", 5)
        self.cloud_pub = self.create_publisher(PointCloud2,
                                               "/cube/blue_cloud", 2)
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
        self._hsv_viz = None

        # Track A: AprilTag
        tag_det = self.try_apriltag(img)
        # Shared HSV mask: used by Track B AND the /cube/blue_cloud publisher
        blue_mask = self.compute_blue_mask(img)
        # Track B: HSV (always run for visibility / debug)
        hsv_det = self.try_hsv(img, blue_mask)

        # Tier-1 short-term memory: feed observation into tracker, or fall
        # back to a predicted pose if the mask dropped out this frame.
        t_ros = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        max_age = float(self.hsv.get("predict_max_age_s", 0.5))
        if hsv_det is not None and self._hsv_viz is not None:
            _, R_obs, centroid_obs = self._hsv_viz
            self.tracker.update(t_ros, centroid_obs, R_obs)
        elif hsv_det is None and not self.tracker.is_stale(t_ros, max_age):
            pred = self.tracker.predict(t_ros)
            if pred is not None:
                c_pred, R_pred = pred
                age = self.tracker.last_age(t_ros)
                hsv_det = self._make_predicted_detection(c_pred, R_pred, age, max_age)
                self._hsv_viz = (None, R_pred, c_pred)

        # 3D point cloud of every blue pixel with valid depth.
        self.publish_blue_cloud(blue_mask, img, msg.header.stamp)
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

        # Local cv2 window (same pattern as hsv_tuner). Falls back gracefully
        # if no DISPLAY available.
        if self.show_display:
            try:
                cv2.imshow("cube_detector", viz)
                cv2.waitKey(1)
                self.preview_window_open = True
            except cv2.error as e:
                if not self.preview_window_open:
                    self.get_logger().warn(
                        f"cv2.imshow failed ({e}); display disabled. "
                        f"Use view_debug.py from a desktop terminal, or "
                        f"`ros2 topic echo /cube/detections` for headless."
                    )
                self.show_display = False

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
    # Adaptive HSV mask (delegates to perception/scripts/blue_mask.py).
    # Used by try_hsv and publish_blue_cloud; same logic is re-used by
    # scene_highlighter.py, so tuning one improves both.
    # ------------------------------------------------------------------
    def compute_blue_mask(self, img):
        return _shared_compute_blue_mask(img, self.latest_depth, self.hsv)

    # ------------------------------------------------------------------
    # Track B: HSV red
    # ------------------------------------------------------------------
    def _diag(self, s):
        """1-Hz throttled info log so try_hsv early-returns are debuggable."""
        now = time.monotonic()
        if now - self._diag_last_t > self._diag_period_s:
            self._diag_last_t = now
            self.get_logger().info(s)

    def try_hsv(self, img, mask):
        if self.latest_depth is None:
            self._diag("try_hsv early-return: latest_depth is None")
            return None
        mask_px = int((mask > 0).sum())
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._diag(f"try_hsv early-return: no contours (mask_px={mask_px})")
            return None
        amin = self.hsv["aspect_min_x10"] / 10.0
        amax = self.hsv["aspect_max_x10"] / 10.0
        # Filter by area + aspect ratio, keep largest survivor
        candidates = []
        rej_area = 0
        rej_aspect = 0
        max_area_seen = 0.0
        max_asp_seen = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            max_area_seen = max(max_area_seen, area)
            if area < self.hsv["min_area"]:
                rej_area += 1
                continue
            rect_c = cv2.minAreaRect(c)
            (_, _), (wc, hc), _ = rect_c
            asp = max(wc, hc) / max(1.0, min(wc, hc))
            max_asp_seen = max(max_asp_seen, asp)
            if not (amin <= asp <= amax):
                rej_aspect += 1
                continue
            candidates.append((area, c, rect_c))
        if not candidates:
            self._diag(
                f"try_hsv early-return: no candidates "
                f"(mask_px={mask_px} contours={len(contours)} "
                f"max_area={max_area_seen:.0f} max_asp={max_asp_seen:.2f} "
                f"rej_area={rej_area} rej_aspect={rej_aspect} "
                f"min_area={self.hsv['min_area']} asp_range=[{amin:.1f},{amax:.1f}])"
            )
            return None
        area, c, _ = max(candidates, key=lambda x: x[0])

        # Build a filled mask of just the chosen contour, sample 3D points
        # from every pixel inside it whose depth is valid, then run PCA.
        # Erode by 4 px to drop the silhouette-edge depth noise the Orbbec
        # is prone to.
        depth = self.latest_depth
        filled = np.zeros(depth.shape[:2], dtype=np.uint8)
        cv2.drawContours(filled, [c], -1, 255, thickness=cv2.FILLED)
        filled = cv2.erode(filled, np.ones((9, 9), np.uint8))
        ys, xs = np.where((filled > 0) & (depth > 0))
        if xs.size < 80:
            self._diag(f"try_hsv early-return: too few depth pixels after erode "
                       f"(xs.size={int(xs.size)}, contour_area={area:.0f})")
            return None
        z_mm = depth[ys, xs].astype(np.float32)
        # Orbbec depth is uint16 mm; if values look already-metric (<50), trust them.
        z_m = np.where(z_mm > 50.0, z_mm / 1000.0, z_mm)
        keep = (z_m > 0.05) & (z_m < 2.0)
        xs, ys, z_m = xs[keep], ys[keep], z_m[keep]
        if xs.size < 80:
            self._diag(f"try_hsv early-return: too few in-range depth points "
                       f"(xs.size={int(xs.size)})")
            return None
        # Reject pixels >20 mm from the median depth: removes flyer outliers
        # (multipath, holes filled with the far-plane) without trimming the
        # face itself, which is <5 mm thick at this distance.
        z_med = float(np.median(z_m))
        keep = np.abs(z_m - z_med) < 0.020
        xs, ys, z_m = xs[keep], ys[keep], z_m[keep]
        if xs.size < 80:
            return None
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx_int, cy_int = self.K[0, 2], self.K[1, 2]
        Xs = (xs - cx_int) * z_m / fx
        Ys = (ys - cy_int) * z_m / fy
        pts = np.stack([Xs, Ys, z_m], axis=1).astype(np.float64)

        # PCA via eigendecomposition of the 3x3 covariance matrix.
        # eigh returns ascending; reorder to descending so e1 = long axis.
        centroid = pts.mean(axis=0)
        centered = pts - centroid
        cov = (centered.T @ centered) / max(1, len(centered) - 1)
        eigvals, eigvecs = np.linalg.eigh(cov)
        order = np.argsort(eigvals)[::-1]
        eigvals = eigvals[order]
        eigvecs = eigvecs[:, order]
        e1 = eigvecs[:, 0]                          # long axis
        e2 = eigvecs[:, 1]                          # short axis on face
        e3 = np.cross(e1, e2)                       # face normal, right-handed
        # Orient e3 toward the camera. Camera looks +Z, so a normal pointing
        # at the camera has negative z-component. Flip e2 to preserve det=+1.
        if e3[2] > 0:
            e3 = -e3
            e2 = -e2
        R = np.column_stack([e1, e2, e3])

        # Stash for the debug-image overlay.
        self._hsv_viz = (c, R, centroid)

        det = Detection3D()
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "hsv_blue"
        # Score: clipped contour area, normalised to 1.0 at 5000 px^2
        hyp.hypothesis.score = float(min(1.0, area / 5000.0))
        hyp.pose.pose = self.rt_to_pose(R, centroid)
        # Tighter than the 2D-yaw fallback because depth participates in pose.
        hyp.pose.covariance = self.diag_cov(
            pos_sigma=0.015, rot_sigma=math.radians(5.0)
        )
        det.results.append(hyp)
        det.bbox = self.bbox_aligned_to_pose(R, centroid)
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

    # ------------------------------------------------------------------
    # Publish all blue mask pixels as a coloured 3D point cloud for RViz.
    # Field layout: x (f32) | y (f32) | z (f32) | rgb (4 bytes packed)
    # ------------------------------------------------------------------
    _CLOUD_FIELDS = [
        PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    _CLOUD_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"),
                             ("z", "<f4"), ("rgb", "<u4")])

    def publish_blue_cloud(self, mask, img_bgr, stamp):
        if self.K is None or self.latest_depth is None:
            return
        depth = self.latest_depth
        # Erode 2 px to drop silhouette-edge pixels where Orbbec depth lies.
        mask_e = cv2.erode(mask, np.ones((5, 5), np.uint8))
        ys, xs = np.where((mask_e > 0) & (depth > 0))
        if xs.size > 0:
            z_mm = depth[ys, xs].astype(np.float32)
            z_m = np.where(z_mm > 50.0, z_mm / 1000.0, z_mm)
            valid = (z_m > 0.05) & (z_m < 2.0)
            xs, ys, z_m = xs[valid], ys[valid], z_m[valid]
        if xs.size == 0:
            buf = np.empty(0, dtype=self._CLOUD_DTYPE)
        else:
            fx, fy = self.K[0, 0], self.K[1, 1]
            cx_i, cy_i = self.K[0, 2], self.K[1, 2]
            Xs = ((xs - cx_i) * z_m / fx).astype(np.float32)
            Ys = ((ys - cy_i) * z_m / fy).astype(np.float32)
            Zs = z_m.astype(np.float32)
            bgr = img_bgr[ys, xs]
            rgb_u32 = (
                (bgr[:, 2].astype(np.uint32) << 16) |
                (bgr[:, 1].astype(np.uint32) << 8) |
                 bgr[:, 0].astype(np.uint32)
            )
            buf = np.empty(len(Xs), dtype=self._CLOUD_DTYPE)
            buf["x"] = Xs
            buf["y"] = Ys
            buf["z"] = Zs
            buf["rgb"] = rgb_u32

        n = int(buf.size)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = n
        msg.fields = self._CLOUD_FIELDS
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = (n > 0)
        msg.data = buf.tobytes()
        self.cloud_pub.publish(msg)

    def project_3d(self, P):
        """Project a 3D point in the camera frame to a 2D pixel.
        Returns (u, v) ints or None if behind the camera."""
        if self.K is None or P[2] <= 1e-6:
            return None
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx_i, cy_i = self.K[0, 2], self.K[1, 2]
        u = fx * P[0] / P[2] + cx_i
        v = fy * P[1] / P[2] + cy_i
        return int(round(u)), int(round(v))

    def draw_pose_axes(self, out, R, centroid, length=0.05, alpha=1.0):
        """Draw X-red, Y-green, Z-blue axes through `centroid`.

        `alpha` in [0, 1] dims each colour — used to render predicted
        (held) poses more faintly than observed ones.
        """
        origin = self.project_3d(centroid)
        if origin is None:
            return
        a = max(0.0, min(1.0, alpha))
        base = ((0, (0, 0, 255)), (1, (0, 255, 0)), (2, (255, 0, 0)))
        for col, color in base:
            scaled = tuple(int(c * a) for c in color)
            tip = self.project_3d(centroid + R[:, col] * length)
            if tip is not None:
                cv2.arrowedLine(out, origin, tip, scaled, 2, tipLength=0.2)

    def _make_predicted_detection(self, centroid, R, age_s, max_age_s):
        """Build a Detection3D for an extrapolated (held) pose.

        Score decays linearly from 0.5 (fresh) to ~0.05 (about to expire),
        so downstream consumers can weight predicted vs observed sensibly.
        class_id is suffixed with "_predicted" for explicit distinction.
        Covariance is widened (2× position, 2× rotation) to reflect the
        loss of direct observation.
        """
        freshness = max(0.0, 1.0 - (age_s / max(1e-6, max_age_s)))
        det = Detection3D()
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "hsv_blue_predicted"
        hyp.hypothesis.score = float(0.05 + 0.45 * freshness)
        hyp.pose.pose = self.rt_to_pose(R, centroid)
        hyp.pose.covariance = self.diag_cov(
            pos_sigma=0.030, rot_sigma=math.radians(10.0),
        )
        det.results.append(hyp)
        det.bbox = self.bbox_aligned_to_pose(R, centroid)
        return det

    def make_debug_image(self, img, tag_det, hsv_det):
        out = img.copy()
        h_, w_ = out.shape[:2]
        if tag_det is not None:
            cv2.putText(out, "track: APRILTAG (tag36h11 id=1)", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 230, 0), 2)
            p = tag_det.results[0].pose.pose.position
            cv2.putText(out,
                        f"pos: ({p.x*1000:.0f}, {p.y*1000:.0f}, {p.z*1000:.0f}) mm",
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
        elif hsv_det is not None:
            cls = hsv_det.results[0].hypothesis.class_id
            score = hsv_det.results[0].hypothesis.score
            is_predicted = cls.endswith("_predicted")
            if is_predicted:
                label = f"track: HSV blue (PREDICTED, score={score:.2f})"
                colour = (0, 130, 200)
            else:
                label = "track: HSV blue (fallback)"
                colour = (0, 200, 230)
            cv2.putText(out, label, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
            p = hsv_det.results[0].pose.pose.position
            cv2.putText(out,
                        f"pos: ({p.x*1000:.0f}, {p.y*1000:.0f}, {p.z*1000:.0f}) mm",
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
            if self._hsv_viz is not None:
                c, R, centroid = self._hsv_viz
                if c is not None:
                    cv2.drawContours(out, [c], -1, (0, 200, 230), 2)
                self.draw_pose_axes(out, R, centroid,
                                    alpha=0.45 if is_predicted else 1.0)
        else:
            cv2.putText(out, "track: NONE", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 230), 2)
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-display", action="store_true",
                    help="Skip cv2.imshow window (for headless runs).")
    args = ap.parse_args()

    show = not args.no_display and bool(os.environ.get("DISPLAY"))
    if not show and not args.no_display:
        print("[cube_detector] $DISPLAY not set — running headless. "
              "Subscribe to /cube/debug_image via view_debug.py for live view.")

    rclpy.init()
    node = CubeDetector(show_display=show)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
