#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Detect the cuboid and its planar pose from a single camera frame at the
fixed observation pose [90,45,45,0,90,0].

Pipeline:
  1. Grab one /camera/color/image_raw frame.
  2. YOLO best.pt -> highest-confidence cube box -> ROI (padded).
  3. Inside the ROI, segment by SATURATION (high S = colored cube; low S =
     grey carpet AND the gripper's shadow). Shadow-immune, color-agnostic for
     a solid cube on a neutral floor. Falls back to "not-floor" value-based
     mask only if the saturation mask is empty (e.g. a white/grey cube).
  4. Largest contour -> cv2.minAreaRect -> center + box points.
  5. Long-axis angle computed from the longest box edge (unambiguous, [0,180)).
  6. Report center offset from image center (centering feedback for drive-up)
     and the long-axis angle (feeds motor 5). Save annotated + mask images.

Outputs (for headless verification over SSH):
  OUT/pose_annotated.jpg, OUT/pose_mask.jpg
  and a RESULT: line on stdout with the numbers.

Env: CUBOID_MODEL, CUBOID_CONF, CUBOID_OUT, SAT_MIN, ROI_PAD.
"""
import os
import sys
import time

import numpy as np
import cv2
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL_PATH = os.path.expanduser(os.environ.get("CUBOID_MODEL", "~/models/cuboid_v1/best.pt"))
OUT_DIR = os.path.expanduser(os.environ.get("CUBOID_OUT", "~/cube_tracker/pose_out"))
CONF = float(os.environ.get("CUBOID_CONF", "0.4"))
SAT_MIN = int(os.environ.get("SAT_MIN", "70"))      # saturation threshold for cube vs grey
ROI_PAD = int(os.environ.get("ROI_PAD", "12"))      # px padding around YOLO box
CAM_TOPIC = os.environ.get("CUBOID_TOPIC", "/camera/color/image_raw")

os.makedirs(OUT_DIR, exist_ok=True)


def grab_frame(timeout=5.0):
    rclpy.init()
    node = rclpy.create_node("detect_pose_grab")
    br = CvBridge()
    box = {}
    node.create_subscription(Image, CAM_TOPIC, lambda m: box.setdefault("f", br.imgmsg_to_cv2(m, "bgr8")), 10)
    t0 = time.time()
    while "f" not in box and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    return box.get("f")


def largest_contour(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 200:
        return None
    return c


def long_axis_angle(box_pts):
    """Angle (deg, [0,180)) of the longest edge of the min-area rect, measured
    from the image +x axis, y-down. This is the cube's long-axis orientation."""
    edges = []
    for i in range(4):
        p, q = box_pts[i], box_pts[(i + 1) % 4]
        edges.append((np.hypot(*(q - p)), p, q))
    _, p, q = max(edges, key=lambda e: e[0])
    ang = np.degrees(np.arctan2(q[1] - p[1], q[0] - p[0]))
    return ang % 180.0


def main():
    frame = grab_frame()
    if frame is None:
        print("RESULT: no_frame")
        sys.exit(2)
    H, W = frame.shape[:2]
    img_cx, img_cy = W / 2.0, H / 2.0

    model = YOLO(MODEL_PATH)
    res = model.predict(frame, conf=CONF, device=0, verbose=False)[0]
    annotated = frame.copy()
    cv2.drawMarker(annotated, (int(img_cx), int(img_cy)), (255, 0, 0),
                   cv2.MARKER_CROSS, 24, 2)  # image center (blue)

    if len(res.boxes) == 0:
        # YOLO failed (e.g. shadow across the cube) -> full-frame saturation
        # fallback: the colored cube is still the most saturated blob.
        conf = 0.0
        rx1, ry1, rx2, ry2 = 0, 0, W, H
        roi = frame
        yolo_ok = False
    else:
        # highest-confidence box
        b = max(res.boxes, key=lambda bb: float(bb.conf))
        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
        conf = float(b.conf)
        rx1, ry1 = max(0, x1 - ROI_PAD), max(0, y1 - ROI_PAD)
        rx2, ry2 = min(W, x2 + ROI_PAD), min(H, y2 + ROI_PAD)
        roi = frame[ry1:ry2, rx1:rx2]
        yolo_ok = True

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    S = hsv[:, :, 1]
    mask = (S >= SAT_MIN).astype(np.uint8) * 255
    method = "saturation"
    if cv2.countNonZero(mask) < 200:
        # fallback: grey/white cube -> use value contrast vs floor median
        V = hsv[:, :, 2].astype(np.int16)
        med = int(np.median(V))
        mask = (np.abs(V - med) > 35).astype(np.uint8) * 255
        method = "value_fallback"
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    cnt = largest_contour(mask)
    # save mask (full-frame placement for clarity)
    mask_full = np.zeros((H, W), np.uint8)
    mask_full[ry1:ry2, rx1:rx2] = mask
    cv2.imwrite(os.path.join(OUT_DIR, "pose_mask.jpg"), mask_full)

    cv2.rectangle(annotated, (rx1, ry1), (rx2, ry2), (0, 200, 200), 1)  # ROI (cyan)

    if cnt is None:
        cv2.imwrite(os.path.join(OUT_DIR, "pose_annotated.jpg"), annotated)
        print(f"RESULT: seg_failed method={method} conf={conf:.2f}")
        return

    cnt = cnt + np.array([[rx1, ry1]])  # ROI -> full-frame coords
    rect = cv2.minAreaRect(cnt)
    (cx, cy), (rw, rh), _ = rect
    box_pts = cv2.boxPoints(rect)
    angle = long_axis_angle(box_pts)
    long_side, short_side = max(rw, rh), min(rw, rh)

    off_x, off_y = cx - img_cx, cy - img_cy   # +x right, +y down (px)

    # annotate
    cv2.drawContours(annotated, [box_pts.astype(int)], 0, (0, 255, 0), 2)
    cv2.drawMarker(annotated, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.line(annotated, (int(img_cx), int(img_cy)), (int(cx), int(cy)), (0, 165, 255), 1)
    txt = f"ang={angle:.1f} off=({off_x:+.0f},{off_y:+.0f}) L/S={long_side:.0f}/{short_side:.0f} {method}"
    cv2.rectangle(annotated, (0, 0), (W, 26), (0, 0, 0), -1)
    cv2.putText(annotated, txt, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.imwrite(os.path.join(OUT_DIR, "pose_annotated.jpg"), annotated)

    print("RESULT: ok "
          f"conf={conf:.2f} yolo={yolo_ok} method={method} "
          f"center_px=({cx:.1f},{cy:.1f}) offset_px=({off_x:+.1f},{off_y:+.1f}) "
          f"long_axis_deg={angle:.1f} long_px={long_side:.1f} short_px={short_side:.1f} "
          f"aspect={long_side/max(short_side,1):.2f}")


if __name__ == "__main__":
    main()
