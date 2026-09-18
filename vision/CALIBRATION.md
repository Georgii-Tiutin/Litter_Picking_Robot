# Arm ↔ overhead-camera calibration

Fitted end-to-end: **commanded arm (x, y) ↔ observed tag pixel (u, v)**, deliberately
skipping a separate camera-intrinsics step. One fit absorbs lens distortion, camera tilt
and the arm's own kinematic error together — including the systematic 22–32 mm offset
measured six times during the Arm & 3D Gripping course.

## Rig
- Overhead: Logitech C505, 1280×720, clamped rigidly (6 kg plate), ~1 m above floor.
- **Address the camera by NAME** (`-i "C505 HD Webcam"`). avfoundation indices shuffle
  between calls — an index silently grabbed the Mac's built-in camera mid-session.
- Capture discards 14–20 warm-up frames (auto-exposure/focus settle).
- Four tag36h11 cubes mark the workspace: **800 × 450 mm centre-to-centre**
  (measured 830 × 480 mm outer-edge, minus one 30 mm cube width per axis).
- Corner tags double as a **drift detector**: if their pixels move, the rig shifted.

## Fixed pose parameters
| parameter | value | why |
|---|---|---|
| pitch | **0.349 rad (20°)** | found by sweeping joint4; presents the gripper tag squarest to the camera (0.89 vs 0.51 at joint4=80) |
| z | 0.274 m | height at that wrist angle |
| joint6 | 142 | holds the tag cube |
| scale | 1.547 px/mm | from the affine part of the fit |

## Model
`grid_data3.csv` — 50 poses, 2 captures averaged each.

| model | k | train (mm) | leave-one-out (mm) |
|---|---|---|---|
| affine | 3 | 2.94 | 3.15 |
| **quad** | 6 | 2.15 | **2.45** |
| cubic | 10 | 1.89 | 2.42 |

Quad and cubic tie within noise → **use quad** (parsimony). With only 22 samples cubic
*overfitted* (LOO 3.55 > quad 3.08); doubling the data fixed it. The bottleneck was data
quantity, not model family.

## Error budget
| source | magnitude |
|---|---|
| tag detection (2 captures, same pose) | **0.0–0.3 px** — negligible |
| arm repeatability (8 repeat visits) | **1.54 mm rms** |
| directional hysteresis (backlash) | 1.32 mm rms, **2.6 mm** worst (−y approach) |
| model generalisation | 2.45 mm |

**All error is mechanical, none optical.** Achievable placement ≈ **2–2.5 mm**.

## Rules that follow
1. **Always raster in one direction** — approaching from −y lands 2.6 mm off, repeatably.
2. **Avoid x ≈ 180–200 mm.** IK returns slightly negative joint3 there; clamping to 0 throws
   the target off by up to 14.9 mm. A real hole in the reachable set, not a bug.
3. **Stop the vendor nodes before commanding the arm.** `apriltag_detect` republishes the arm
   pose every camera frame and silently overrides external commands — no error, nothing moves.
4. **Wait for ROS 2 discovery before publishing.** A fresh node publishing immediately loses
   messages before the subscriber is matched. This is exactly the section-3 vendor bug; their
   `while not get_subscription_count()` loop was the right fix with a missing `import time`.
5. **Re-validate after clamping.** Validate → modify → not re-validating is how a pose ends up
   15 mm off target. (I wrote this bug myself before catching it.)

## Files
- `capture.sh` — one settled frame from the C505
- `detect_tags.py`, `build_homography.py` — tag detection / world frame
- `move_to.py`, `set_joints.py`, `probe_joints.py` (on robot, `~/calib/`) — guarded motion
- `collect_grid3.py`, `analyse.py`, `backlash_test.py` — calibration pipeline
- `arm_camera_model.json` — the fitted model
