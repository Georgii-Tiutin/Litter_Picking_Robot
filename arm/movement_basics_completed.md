# Movement Basics — Completed Work

A running record of the cuboid grasp pipeline built and validated on the robot
(Jetson `jetson@192.168.50.103`, ROS_DOMAIN_ID=30, camera = Orbbec dabai_dcw2,
model = `~/models/cuboid_v1/best.pt`). Companion to `grasp_plan.md`.

## The goal
Pick up a 6×3×3 cm cuboid whose claw can only open 6 cm and can only grab across
the 3 cm width (never the 6 cm length, never diagonal). So every grasp must
(a) put the cube at a known spot and (b) rotate the gripper to the cube's angle.

## Key poses & constants (all verified on hardware)
- **Observation / pre-grasp pose:** `[90, 45, 45, 0, 90, 0]` — cube framed for detection.
- **Descent pose:** `[90, 0, 76, 0, R, 0]` (jaws open) → close → `[90, 0, 76, 0, R, 145]`.
  - `R` = motor-5 wrist roll matched to the cube's orientation.
  - Elbow `joint3 = 76` was tuned down (from 90 → 84 → 76) to reach cube height.
  - Gripper `joint6`: **0 = open, 145 = firm clamp on 3 cm width, 180 = fully closed**.
- Gripper close value **x = 145** found empirically (firm grip on the 3 cm face).

## What we built (scripts — in repo and on robot `~/cube_tracker/`)
- **`set_pose.py j1..j6`** — publish a full 6-joint pose (`/arm6_joints`).
- **`set_joint.py id val [t]`** — move a single joint (`/arm_joint`).
- **`detect_pose.py`** — single-shot detector: YOLO ROI → **saturation segmentation**
  (shadow-immune) → `minAreaRect` → center + long-axis angle. Prints `RESULT:` and
  saves annotated + mask images. Full-frame saturation fallback when YOLO misses.
- **`detect_pose_live.py`** — continuous version, shows annotated feed on the robot
  monitor + writes `pose_out/live.jpg`.
- **`track_motor5.py`** — live servo: rolls **motor 5** to keep the jaws aligned to
  the cube's orientation. Mapping `R = 90 + SIGN·(θ−90)`, SIGN=+1 verified.
- **`track_and_grab.py`** — track_motor5 + a grab action (keys `g`/`r`/`q` on the
  robot window, or SSH file triggers `pose_out/{grab,release}.trigger`):
  descend `[90,0,76,0,R,0]` → close 145 → lift, keeping motor 5 = R.
- **`drive_base.py <cm> [speed]`** — **odom closed-loop** base move (drive until
  `/odom_raw` distance reached). Replaces unreliable timed pulses.
- **`drive_up.py`** — fixed-obs-pose approach: arm held at observation pose, base
  visual-servos the cube to a target pixel. (Superseded by the arm-tracking approach.)
- **`cube_arm_tracker.py`** (edited) — eye-in-hand approach: arm tracks head-up,
  base faces + drives in; joint2 floor lowered 60→45 so it can reach the obs tilt.
- **`center_arm.py`** — arm-only visual servo: joint1 pan + joint2 tilt keep the
  cube centered in frame (no base motion). Converged cleanly (signs verified).
- **`align_to_obs.py`** — the composite that works: **inner loop** arm-centers the
  cube; **outer loop** drives the base to unwind the arm back to `[90,45,45]`
  (rotate → j1=90, creep → j2=45). Ends at the true observation pose, cube centered.

## What works (validated end-to-end)
1. **Detection** — saturation segmentation is robust: tightly boxes the green cube
   and **ignores the adjacent gripper shadow** (which killed plain "not-floor" and
   hue thresholds). conf typically 0.7–0.98.
2. **Orientation tracking** — motor 5 follows the cube's angle live, shown on screen.
3. **Grip** — fixed close value 145 firmly clamps the 3 cm width.
4. **Approach + align** — the robot autonomously drives up to the cube and ends at
   the observation pose with the cube centered (arm-center → base-correct converges).

## Hard-won lessons (see also memory files)
- **First arm command after a fresh node is often dropped** during DDS discovery →
  resend, or run multi-step motions from ONE persistent node (`grasp_sequence.py`).
- **Base `/cmd_vel` has a big acceleration ramp** — short timed pulses barely move
  it (looks dead); a 1 s command at 0.12 m/s moved ~9 cm. Use **odom closed-loop**
  for small moves. ~1 cm precision at best, with coast overshoot.
- **`cv2.absdiff(V, scalar)` crashes** on this OpenCV — use `np.abs(V - med)`.
- **`pkill -f <name>` from SSH matches its own shell** if the launch command on the
  same line contains `<name>` — it kills the session (mystery exit 255). Run the
  kill and the launch as **separate** commands.
- The robot **dropped off the network** once mid-session (reboot); camera + nodes
  must be relaunched after.

## THE OPEN BUG (root cause found, fix identified — not yet applied)
Every centered grasp missed even with the cube perfectly centered at the true
observation pose. **Cause:** the descent does not land at image center. Measured
by placing the cube in the open jaws, lifting, and detecting: the **grasp spot is
at pixel offset (+5, −27) from image center** (~27 px ABOVE center = slightly
farther from the robot). We were centering the cube to (320,240), so the jaws
always came down ~27 px beyond it and pushed the cube on the far side.

**Fix (next step):** retarget the centering/align to the grasp-spot pixel
(≈ offset +5, −27) instead of image center. Then re-run align → grasp; the cube
should end up exactly where the jaws close.

## Immediate next steps
1. Set the align/centering target to the measured grasp spot (+5, −27), not center.
2. Re-run `align_to_obs.py` → `track_and_grab.py`, trigger grab, verify a real pick.
3. Calibrate the angle→motor-5 mapping across more orientations if grabs at extreme
   angles miss (currently anchored on θ=90 ↔ R=90, SIGN=+1).
