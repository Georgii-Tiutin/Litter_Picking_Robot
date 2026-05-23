"""Shared adaptive HSV blue mask used by cube_detector.py and scene_highlighter.py.

Two-stage strategy:
  1. Strict HSV pass produces confident "seed" blobs (small clusters dropped).
  2. Inside an ROI defined by the depth band of the seeds + a spatial
     dilation around them, apply a looser HSV threshold to recover pixels
     the strict pass missed (typically specular highlights where S drops or
     depth fails entirely).
  3. Final mask = strict ∪ loose-in-ROI, followed by a single morph close.

The "depth missing" branch of the ROI matters: structured-light depth
often returns 0 on bright specular patches — the very pixels we are
trying to recover. The spatial ROI prevents this allowance from leaking
into faraway blue objects.
"""

import cv2
import numpy as np


HSV_DEFAULTS = {
    "h_low_1": 100,  "h_high_1": 130,
    "h_low_2": 100,  "h_high_2": 130,
    "s_min": 80,     "s_max": 255,
    "v_min": 60,     "v_max": 255,
    "morph_kernel": 5,
    "min_area": 500,
    "aspect_min_x10": 14,
    "aspect_max_x10": 35,
    # Adaptive-mask parameters (new):
    "seed_min_area": 800,        # px² — anything smaller is ignored as noise
    "depth_band_mm": 30.0,       # ±mm around seed median depth
    "spatial_dilate_px": 25,     # px halo around seeds for the loose ROI
    "loose_s_min": 30,           # lowered S floor for the second pass
    "loose_v_min": 30,           # lowered V floor for the second pass
    # Tier-1 pose-tracker parameters (used by cube_detector.py):
    "predict_max_age_s": 0.5,    # max seconds to hold/extrapolate a pose
                                 # after the last successful HSV detection
    "predict_gap_reset_s": 1.0,  # if gap > this, drop the velocity estimate
                                 # so we don't extrapolate across long pauses
}


def compute_blue_mask(img_bgr, depth_mm, hsv_cfg):
    """Return an HxW uint8 mask (0/255) of blue pixels in img_bgr.

    Args:
        img_bgr:  HxW BGR image (uint8)
        depth_mm: HxW depth image in mm (uint16) aligned to img_bgr, or None
        hsv_cfg:  dict — keys from HSV_DEFAULTS (missing keys filled in)
    """
    cfg = {**HSV_DEFAULTS, **(hsv_cfg or {})}
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # --- Strict pass ---------------------------------------------------
    strict = cv2.inRange(
        hsv,
        (cfg["h_low_1"], cfg["s_min"], cfg["v_min"]),
        (cfg["h_high_1"], cfg["s_max"], cfg["v_max"]),
    )
    ks = max(1, int(cfg["morph_kernel"]) | 1)
    kernel = np.ones((ks, ks), np.uint8)
    strict = cv2.morphologyEx(strict, cv2.MORPH_OPEN, kernel)
    strict = cv2.morphologyEx(strict, cv2.MORPH_CLOSE, kernel)

    # --- Confident seed blobs (drop small noise clusters) --------------
    seed_area_min = int(cfg["seed_min_area"])
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(strict, connectivity=8)
    if num_labels <= 1:
        return strict
    seed_mask = np.zeros_like(strict)
    any_seed = False
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] >= seed_area_min:
            seed_mask[labels == i] = 255
            any_seed = True
    if not any_seed or depth_mm is None:
        return strict

    # --- Depth band around seeds ---------------------------------------
    valid_depth = depth_mm > 0
    seed_pix = (seed_mask > 0) & valid_depth
    if not seed_pix.any():
        return strict
    z_med = float(np.median(depth_mm[seed_pix]))
    delta = float(cfg["depth_band_mm"])
    depth_band = valid_depth & (depth_mm >= (z_med - delta)) & (depth_mm <= (z_med + delta))

    # --- Spatial halo around seeds -------------------------------------
    dil_px = int(cfg["spatial_dilate_px"]) | 1  # odd
    seed_near = cv2.dilate(seed_mask, np.ones((dil_px, dil_px), np.uint8))

    # ROI: near a seed AND (at cube depth OR depth missing). The
    # missing-depth branch is what catches specular highlights.
    roi = (seed_near > 0) & (depth_band | (~valid_depth))

    # --- Loose pass inside ROI -----------------------------------------
    loose = cv2.inRange(
        hsv,
        (cfg["h_low_1"], int(cfg["loose_s_min"]), int(cfg["loose_v_min"])),
        (cfg["h_high_1"], cfg["s_max"], cfg["v_max"]),
    )
    loose_in_roi = cv2.bitwise_and(loose, (roi.astype(np.uint8) * 255))

    # --- Combine -------------------------------------------------------
    final = cv2.bitwise_or(strict, loose_in_roi)
    final = cv2.morphologyEx(final, cv2.MORPH_CLOSE, kernel)
    return final
