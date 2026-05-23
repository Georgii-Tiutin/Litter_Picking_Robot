# Calibration

Calibrated parameters for the ROSMASTER M3PRO. Two locations: this directory
(canonical, on Mac) and `~/project0/calibration/` on the Jetson (deployed copy).

## Phase 0.6.1 — Camera intrinsics — 2026-05-03 — DONE

**Method:** captured factory intrinsics published by the Orbbec DCW2 driver
(`OrbbecSDK_ROS2`, launcher `orbbec_camera/launch/dabai_dcw2.launch.py`).
The DCW2 ships factory intrinsics burned into the device EEPROM; the driver
republishes them as standard `sensor_msgs/CameraInfo`.

Procedure (run on robot):

```bash
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch orbbec_camera dabai_dcw2.launch.py &
ros2 topic echo --once /camera/color/camera_info
ros2 topic echo --once /camera/depth/camera_info
```

Raw snapshots preserved at:
- `~/project0/calibration/camera_info_color_raw.txt`
- `~/project0/calibration/camera_info_depth_raw.txt`

Decision to use factory intrinsics over a fresh checkerboard run:

- `fx`/`fy` agree to 0.02% (479.471 vs 479.388) — no aspect-ratio bug.
- `cx`/`cy` are slightly off-center, physically realistic for an
  uncalibrated-by-the-user RGB-D consumer module.
- Distortion is `rational_polynomial` with small first-order coefficients
  (k1 ≈ −0.018, k2 ≈ 0.005, p1 ≈ 9e-5, p2 ≈ −2.7e-4). Higher orders are
  zero — the device returns coefficients for an already-rectified image.
- Acceptable error budget for cube grasp at 200–400 mm range: ~2 mm
  position error is dominated by *depth noise*, not intrinsic error.
  Re-running checkerboard would not move the needle for our use case.

**Trade-off accepted:** if Phase 2 position error proves to be lens-bound
rather than depth-bound, fall back to a checkerboard calibration with
the `camera_calibration` package (10×7, 25 mm squares).

### ⚠ Driver behaviour to be aware of

`depth_registration` defaults to `false` in the launcher, but the depth
topic is nevertheless published in `camera_color_optical_frame` at
640×480 (color resolution, not the depth sensor's native 640×400) with
the **color** camera matrix. This means the driver is silently
aligning depth into the color frame regardless of the flag.

**Implication for Phase 2:** the perception pipeline must explicitly set
`depth_registration:=true` in our project bringup launcher, both as
documentation and to guard against driver behaviour drift on a future
Orbbec SDK upgrade. With registration on, sampling depth at the cube
centroid uses the **color** camera intrinsics (this file's color
`camera_matrix`), and the depth camera intrinsics are not needed at
runtime.

### Files

| File | Format | Use |
|------|--------|-----|
| `camera_intrinsics_color.yaml` | `camera_info_manager` | RGB ↔ 3D lift; load via `CameraInfoManager` or `image_pipeline` |
| `camera_intrinsics_depth.yaml` | `camera_info_manager` | Identical to color (driver aligns); kept for symmetry |
| `camera_info_color_raw.txt` | `ros2 topic echo` dump | Provenance only |
| `camera_info_depth_raw.txt` | `ros2 topic echo` dump | Provenance only |

### Validation

`/camera/color/image_raw` rate: 28.7 Hz (nominal 30).
`/camera/depth/image_raw` rate: 9.06 Hz (nominal 10).
Both matched the rates measured during 0.5 simulation S.7 verification.

## Phase 0.6.2 — Hand-eye calibration — DONE 2026-05-04 (with documented residual floor)

### Final result

`hand_eye_endeffector_to_camera.yaml` (PARK method, 27 captures):

```
translation:
  x: -0.106538 m
  y: -0.015415 m
  z: -0.097122 m
rotation_rpy_rad:
  roll:  1.326384  (76.0°)
  pitch: -0.208860 (-12.0°)
  yaw:   -1.343833 (-77.0°)
```

Magnitude of translation: **145 mm**, consistent with the physical
camera-mount-on-arm4 vs gripper-on-arm5 geometry.

### Residuals (the bad news)

| metric | mean | median | target |
|---|---|---|---|
| AX=XB rotation residual | 19.3° | **13.7°** | < 1° |
| AX=XB translation residual | 142 mm | **82 mm** | < 5 mm |

These residuals are **far above the original target**. Root causes (in
descending order of impact):

1. **Open-loop joints (no servo position feedback).** M3PRO bus servos
   don't expose actual position to ROS — FK must trust commanded
   angles. Servo deadband, gear backlash, and gravity-load deflection
   accumulate ~1–3° per joint × 5 joints. End-effector pose error
   typically 10–25 mm and 5–10° at the calibration timescale.

2. **5-DOF arm with limited reach diversity.** The arm geometry caps
   tag-distance variation at ~38 mm regardless of commanded pose
   (verified empirically across 27 poses spanning the full reachable
   workspace). Without translational diversity, AX=XB is poorly
   conditioned.

3. **URDF link-length tolerances.** Hobby-grade arm URDFs typically
   have 1–3 mm per-link bias vs the physical assembly. Compounds with
   the 5DOF chain.

4. **AprilTag detection noise at small image scales.** Tag spans
   ~80–120 px in our captures → dt_apriltags rotation precision
   ~0.5–1°.

### What we tried before accepting these residuals

- All four solver methods (TSAI, PARK, HORAUD, DANIILIDIS): all gave
  residuals in the same band. PARK gave the lowest.
- 14-capture conservative subset: appeared to give better numbers
  (8°/38 mm) but produced a translation magnitude of 370 mm, larger
  than the entire arm — overfitting due to insufficient rotation
  diversity. Rejected.
- Iterative outlier pruning: removing the wide-rotation captures made
  per-capture residuals improve cosmetically but moved the solution
  away from the physically plausible answer. Rejected.
- 27-capture full set with PARK: kept. Higher residuals, physically
  consistent translation, full rotation diversity.

### Implications for downstream phases

- **Phase 2 (position estimation)** can use this calibration for
  *coarse* cube localisation (~5 cm accuracy). Don't trust it for
  direct grasp.
- **Phase 5 (pickup)** must use **visual servoing for the final
  approach** (last 5–10 cm). The calibration places the cube within
  the workspace bbox; vision-in-loop closes the residual gap.
- **Phase 4 (navigation)** is unaffected — Nav2 uses base_link/odom,
  not the arm/camera transform.

### Re-running if you want to try again later

The capture script and presets are unchanged on the robot. To try
again:

```bash
# Stash the current results
mv ~/project0/calibration/handeye_captures.json \
   ~/project0/calibration/handeye_captures_2026-05-04.json
mv ~/project0/calibration/hand_eye_endeffector_to_camera.yaml \
   ~/project0/calibration/hand_eye_endeffector_to_camera_2026-05-04.yaml
# Then re-run capture_handeye.py and solve_handeye.py
```

To meaningfully improve, you'd need either (a) servo position readback
(write a node that polls `Rosmaster_Lib.get_uart_servo_angle` and
publishes /joint_states), or (b) chassis-motion inclusion in AX=XB
using odometry (more captures across base translations).



Replaces the identity-placeholder `DCW2 → camera_link` static TF set up
in 0.2. Eye-in-hand on `arm4`. Plan: AprilTag36h11 fixed in workspace,
~15 commanded arm poses, solve `AX=XB` with OpenCV
`cv2.calibrateHandEye`. Output transform persisted as
`hand_eye_endeffector_to_camera.yaml`.

### Tools

- `scripts/capture_handeye.py` — interactive capture node. Subscribes to
  `/camera/color/image_raw` + `/camera/color/camera_info` + `/arm6_joints`,
  runs `dt_apriltags` per frame to detect tag36h11 ID=0, on Enter calls
  `/get_kinemarics` (FK from `arm_kin/kin_srv`) and saves the pair to
  `handeye_captures.json` plus a snapshot PNG in `handeye_captures/`.
- `scripts/solve_handeye.py` — offline solver. Reads the JSON, runs
  `cv2.calibrateHandEye` (default method TSAI, override with `--method`
  PARK | HORAUD | ANDREFF | DANIILIDIS), computes AX=XB residuals,
  writes the calibrated yaml + a ready-to-paste
  `static_transform_publisher` CLI line.
- `scripts/bringup_handeye.sh` — bringup helper. Launches the Orbbec
  driver and `arm_kin/kin_srv` together, leaves them running until
  Ctrl-C.

### Hardware setup (already done — verified 2026-05-03)

- AprilTag36h11 ID=0 printed at exact 80 mm and mounted flat on a book.
- Robot positioned in clear workspace, joystick controlling the arm.
- Smoke-tested: FK service returns sensible pose for rest joints
  `[90, 120, 10, 20, 90, 0]` → x=0.199 m, y=0.000 m, z=0.259 m,
  pitch=0.52 rad (30°). Capture script picks up live intrinsics
  (fx=479.47) and AprilTag detector loads cleanly.

### Procedure

1. Place the AprilTag in the arm's workspace at a height the eye-in-hand
   camera can see comfortably from a variety of arm poses (roughly
   level, ~25–40 cm from the base).

2. **Terminal A** (bringup):

   ```bash
   bash ~/project0/calibration/scripts/bringup_handeye.sh
   ```

   Wait until it lists `/camera/color/camera_info` and
   `/get_kinemarics`. Leave this terminal open.

3. **Terminal B** (capture):

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/yahboomcar_ws/install/setup.bash
   export ROS_DOMAIN_ID=30
   python3 ~/project0/calibration/scripts/capture_handeye.py
   ```

   You'll see `camera_info: fx=479.47 ...` confirming the camera is up.

4. Move the arm with the joystick to a pose where the camera sees the
   tag. Press **Enter** in Terminal B to capture. Repeat for **at
   least 15 poses**. Tips for a good calibration:

   - **Vary all three rotations.** Rotating the camera around at least
     two distinct axes between captures is required — pure-translation
     captures make the AX=XB problem degenerate. Aim for 30°+ rotation
     deltas between captures across yaw, pitch, and roll.
   - **Vary distance** between captures (15 cm to 40 cm camera-to-tag
     is a reasonable range — readable in `tag_z` printed at each
     capture).
   - **Tag must be fully visible**, not clipped by the image edge. The
     script warns if no tag is detected at the moment of Enter.
   - If a capture turns out bad (joints moved during capture, tag
     partially occluded), type `d` Enter to delete the most recent
     entry.

5. Type `q` Enter when you have ≥15 good captures.

6. **Solve**:

   ```bash
   python3 ~/project0/calibration/scripts/solve_handeye.py
   ```

   The script writes
   `~/project0/calibration/hand_eye_endeffector_to_camera.yaml`
   and prints AX=XB residuals. Targets:
   - rotation residual median **< 1°**
   - translation residual median **< 5 mm**

   If residuals are larger, run again with `--method PARK` or
   `--method DANIILIDIS` and compare. Persistent large residuals point
   to (a) servo precision drift between capture and FK call, (b) bad
   captures (delete and re-do), or (c) too-narrow rotational diversity.

### What "endeffector" means in the output yaml

`arm_kin/kin_srv` parses the URDF and runs KDL FK on the chain ending
at the deepest unbranched link from `base_link`. For this URDF that
chain ends at `arm5` (because both gripper paddles and the
`Gripping` link branch off `arm5` via fixed joints). So the calibrated
transform is `arm5 → camera_color_optical_frame`. To express the
camera relative to the TCP (`Gripping`), compose with the URDF's fixed
`arm5 → Gripping` transform — the difference is small (a few cm) and
constant. For Phase 2 perception math, either origin works as long as
it's used consistently.

## Phase 0.6.3 — Repo storage — in progress

Files in this directory (Mac canonical) and `~/project0/calibration/`
(robot mirror):

| File | Purpose |
|---|---|
| `camera_intrinsics_color.yaml` | RGB intrinsics, factory-calibrated |
| `camera_intrinsics_depth.yaml` | Depth intrinsics (registered to color frame) |
| `hand_eye_endeffector_to_camera.yaml` | Eye-in-hand transform (PARK, 27 captures) |
| `handeye_captures.json` | Raw 27 captures (joints + tag pose) for reproducibility |
| `handeye_captures/` | Per-capture annotated PNGs |
| `apriltag36h11_id0_80mm_300dpi.png` | Calibration target (80 mm tag, print-ready) |
| `scripts/bringup_handeye.sh` | Launches Orbbec + arm_kin/kin_srv |
| `scripts/capture_handeye.py` | Capture node with preset cycling |
| `scripts/solve_handeye.py` | AX=XB solver |

The calibrated static TF still needs to be wired into bringup so
perception nodes can use it:

- `~/project0/launch/static_tfs.launch.py` — replace identity
  placeholder with calibrated `Gripping → camera_color_optical_frame`
  transform from `hand_eye_endeffector_to_camera.yaml`. Note: the
  yaml's parent frame is "endeffector" (arm_kin's FK terminus = arm5
  per KDL chain), but physically it equals `Gripping` (fixed-joint
  child of arm5). Either parent works for downstream consumers as
  long as it's consistent.
- Future project0 perception launcher — set
  `depth_registration:=true` and pass `camera_intrinsics_color.yaml`
  to consumers that re-publish CameraInfo.
