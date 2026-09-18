#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Solve the pixel->floor homography from collected correspondences.

Reads homography_points.json ({X,Y,u,v} list), fits a 3x3 homography H
mapping image pixels (u,v) -> floor (X,Y) in metres, reports per-point
reprojection error, and writes floor_homography.yaml for grasp_pose.py.
Needs >= 4 points; 6-8 well-spread points recommended.
"""
import json
import os

import numpy as np
import cv2
import yaml

OUT = "/home/jetson/project0/calibration/floor_homography"
JSON = os.path.join(OUT, "homography_points.json")
YAML = os.path.join(OUT, "floor_homography.yaml")


def main():
    data = json.load(open(JSON))
    if len(data) < 4:
        print(f"NEED >=4 points, have {len(data)}")
        return
    pix = np.array([[d["u"], d["v"]] for d in data], dtype=np.float64)
    xy = np.array([[d["X"], d["Y"]] for d in data], dtype=np.float64)

    # least-squares homography over all points (method=0)
    H, _ = cv2.findHomography(pix, xy, method=0)
    proj = cv2.perspectiveTransform(pix.reshape(-1, 1, 2), H).reshape(-1, 2)
    err_mm = np.linalg.norm(proj - xy, axis=1) * 1000.0

    print("H =")
    print(H)
    print(f"\npoints={len(data)}  reproj err_mm: "
          f"mean={err_mm.mean():.1f} max={err_mm.max():.1f}")
    for d, e in zip(data, err_mm):
        print(f"  X={d['X']:+.3f} Y={d['Y']:+.3f} u={d['u']:6.1f} v={d['v']:6.1f} -> err {e:4.1f} mm")

    yaml.safe_dump({
        "H": H.tolist(),
        "frame": "arm_base_ground (X fwd, Y left, metres)",
        "obs_pose_joints": [90, 100, 0, 0, 90, 0],
        "reproj_err_mm_mean": float(err_mm.mean()),
        "reproj_err_mm_max": float(err_mm.max()),
        "note": "pixel(u,v)=bbox bottom-centre -> floor(X,Y). Valid ONLY at the fixed observation pose.",
    }, open(YAML, "w"))
    print(f"\nwrote {YAML}")


if __name__ == "__main__":
    main()
