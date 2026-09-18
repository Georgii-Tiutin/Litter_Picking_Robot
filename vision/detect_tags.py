#!/usr/bin/env python
"""Detect tag36h11 tags in an image; report id, centre, apparent size, px/mm."""
import sys, numpy as np, cv2
from pupil_apriltags import Detector

img_path = sys.argv[1]
tag_mm   = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

img  = cv2.imread(img_path)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
det  = Detector(families="tag36h11", nthreads=4, quad_decimate=1.0,
                refine_edges=1, decode_sharpening=0.25)
tags = det.detect(gray)

h, w = gray.shape
print(f"image {w}x{h}   tags found: {len(tags)}")
for t in sorted(tags, key=lambda t: t.tag_id):
    c = t.corners
    sides = [float(np.linalg.norm(c[i] - c[(i+1) % 4])) for i in range(4)]
    mean_side = sum(sides)/4
    pxmm = mean_side / tag_mm
    print(f"  id={t.tag_id:<3} centre=({t.center[0]:7.1f},{t.center[1]:7.1f})  "
          f"side_px={mean_side:6.1f} (min {min(sides):.1f} max {max(sides):.1f})  "
          f"{pxmm:5.2f} px/mm  -> frame covers {w/pxmm:6.0f} x {h/pxmm:5.0f} mm  "
          f"margin={t.decision_margin:.1f}")
