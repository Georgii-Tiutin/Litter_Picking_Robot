#!/usr/bin/env python3
"""Solve eye-in-hand calibration from captures recorded by capture_handeye.py.

Reads ~/project0/calibration/handeye_captures.json, runs
cv2.calibrateHandEye in eye-in-hand mode (camera attached to arm), and
writes ~/project0/calibration/hand_eye_endeffector_to_camera.yaml plus
a ready-to-paste static_transform_publisher CLI line.

cv2.calibrateHandEye conventions (eye-in-hand):
  Inputs:
    R_gripper2base, t_gripper2base — pose of gripper expressed in base
    R_target2cam,   t_target2cam   — pose of target expressed in camera
  Outputs:
    R_cam2gripper, t_cam2gripper — pose of camera expressed in gripper

So the result IS the constant rigid `endeffector_T_camera_optical`
transform we need to bake into the TF tree.
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

CAL_DIR = Path("/home/jetson/project0/calibration")
JSON_IN = CAL_DIR / "handeye_captures.json"
YAML_OUT = CAL_DIR / "hand_eye_endeffector_to_camera.yaml"

METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def rpy_to_R(roll, pitch, yaw):
    """ROS RPY (Rz(yaw) * Ry(pitch) * Rx(roll), applied to column vector)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    """Inverse of rpy_to_R, returns (roll, pitch, yaw)."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def compute_residuals(R_g2b, t_g2b, R_t2c, t_t2c, R_c2g, t_c2g):
    """Sanity check: AX = XB residual stats across capture pairs."""
    n = len(R_g2b)
    X = np.eye(4)
    X[:3, :3] = R_c2g
    X[:3, 3] = t_c2g.flatten()
    rot_err_deg, trans_err_mm = [], []
    for i in range(n):
        for j in range(i + 1, n):
            A = np.eye(4)
            B = np.eye(4)
            G_i = np.eye(4)
            G_i[:3, :3] = R_g2b[i]
            G_i[:3, 3] = t_g2b[i].flatten()
            G_j = np.eye(4)
            G_j[:3, :3] = R_g2b[j]
            G_j[:3, 3] = t_g2b[j].flatten()
            A = np.linalg.inv(G_j) @ G_i  # gripper motion j<-i in gripper frame
            T_i = np.eye(4)
            T_i[:3, :3] = R_t2c[i]
            T_i[:3, 3] = t_t2c[i].flatten()
            T_j = np.eye(4)
            T_j[:3, :3] = R_t2c[j]
            T_j[:3, 3] = t_t2c[j].flatten()
            B = T_j @ np.linalg.inv(T_i)  # target motion j<-i in camera frame
            E = A @ X @ np.linalg.inv(X @ B)  # should be identity
            ang = math.acos(max(-1.0, min(1.0, (np.trace(E[:3, :3]) - 1) / 2)))
            rot_err_deg.append(math.degrees(ang))
            trans_err_mm.append(np.linalg.norm(E[:3, 3]) * 1000.0)
    return (
        float(np.mean(rot_err_deg)),
        float(np.mean(trans_err_mm)),
        float(np.median(rot_err_deg)),
        float(np.median(trans_err_mm)),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", default="TSAI", choices=list(METHODS))
    ap.add_argument("--input", default=str(JSON_IN))
    ap.add_argument("--output", default=str(YAML_OUT))
    args = ap.parse_args()

    captures = json.loads(Path(args.input).read_text())
    if len(captures) < 6:
        raise SystemExit(f"Need >=6 captures, have {len(captures)}.")

    R_g2b, t_g2b = [], []
    R_t2c, t_t2c = [], []
    for rec in captures:
        fk = rec["fk_base_to_endeffector"]
        R = rpy_to_R(fk["roll"], fk["pitch"], fk["yaw"])
        t = np.array([fk["x"], fk["y"], fk["z"]]).reshape(3, 1)
        R_g2b.append(R)
        t_g2b.append(t)
        T_tag = np.array(rec["tag_in_camera_optical_4x4"])
        R_t2c.append(T_tag[:3, :3])
        t_t2c.append(T_tag[:3, 3].reshape(3, 1))

    R_c2g, t_c2g = cv2.calibrateHandEye(
        R_gripper2base=R_g2b,
        t_gripper2base=t_g2b,
        R_target2cam=R_t2c,
        t_target2cam=t_t2c,
        method=METHODS[args.method],
    )
    t_c2g = t_c2g.flatten()
    roll, pitch, yaw = R_to_rpy(R_c2g)

    rot_mean, trans_mean, rot_med, trans_med = compute_residuals(
        R_g2b, t_g2b, R_t2c, t_t2c, R_c2g, t_c2g
    )

    yaml_text = (
        f"# eye-in-hand calibration result\n"
        f"# transform: endeffector (arm5/Gripping per arm_kin FK) -> camera_color_optical_frame\n"
        f"# method: {args.method}\n"
        f"# n_captures: {len(captures)}\n"
        f"# AX=XB residual (mean): rot={rot_mean:.3f} deg, trans={trans_mean:.3f} mm\n"
        f"# AX=XB residual (median): rot={rot_med:.3f} deg, trans={trans_med:.3f} mm\n"
        f"parent_frame: endeffector\n"
        f"child_frame: camera_color_optical_frame\n"
        f"translation:\n"
        f"  x: {t_c2g[0]:.6f}\n"
        f"  y: {t_c2g[1]:.6f}\n"
        f"  z: {t_c2g[2]:.6f}\n"
        f"rotation_rpy_rad:\n"
        f"  roll:  {roll:.6f}\n"
        f"  pitch: {pitch:.6f}\n"
        f"  yaw:   {yaw:.6f}\n"
        f"rotation_matrix:\n"
        f"  - [{R_c2g[0,0]:.6f}, {R_c2g[0,1]:.6f}, {R_c2g[0,2]:.6f}]\n"
        f"  - [{R_c2g[1,0]:.6f}, {R_c2g[1,1]:.6f}, {R_c2g[1,2]:.6f}]\n"
        f"  - [{R_c2g[2,0]:.6f}, {R_c2g[2,1]:.6f}, {R_c2g[2,2]:.6f}]\n"
    )
    Path(args.output).write_text(yaml_text)

    print(yaml_text)
    print("Suggested static TF (substitute Gripping for endeffector if your URDF chain ends there):")
    print(
        f"  ros2 run tf2_ros static_transform_publisher "
        f"{t_c2g[0]:.6f} {t_c2g[1]:.6f} {t_c2g[2]:.6f} "
        f"{yaw:.6f} {pitch:.6f} {roll:.6f} "
        f"Gripping camera_color_optical_frame"
    )
    print(f"\nWrote {args.output}")


if __name__ == "__main__":
    main()
