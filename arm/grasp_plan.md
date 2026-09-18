# Cuboid Grasp Plan

## Problem / constraints
- Block is **6 × 3 × 3 cm**; max claw separation is **6 cm**.
- Must grab across the **3 cm dimension** (the thin side). Cannot grab across the 6 cm length, and cannot grab when the block is diagonal to the gripper.
- Therefore **orientation alignment is mandatory** before every grasp.
- Assumption: the block always **lies flat** (6 × 3 footprint). A standing block (3 × 3 footprint, 6 cm tall) breaks the fixed-descent assumption — see edge cases.

## Fixed observation pose (determined by testing)
The robot drives up until its arm is at **`[90, 45, 45, 0, 90, 0]`** (joints 1–6) with the **cube dead-centre in the camera FOV**. This is the fixed pose at which the frame is snapped and the pixel→arm homography is calibrated. Gripper (joint6): `0` = open, `180` = closed.

## Approach (why it works)
Drive up to a **fixed arm pose** so distance never has to be estimated at grasp time. Calibrate the **pixel → arm-coordinate homography once at that exact pose**; then both the block's center offset and its rotation angle come out in real units for free. Standing still is what makes this cheap and repeatable.

The body parks **well back** and the **arm reaches forward** to cover the remaining gap — the body never needs to be near the cube. This keeps the protruding claw motor box clear of the cube and makes the drive-up tolerance loose (the XY homography correction handles the residual).

Detection uses the camera (RGB) only. **The depth-camera option is rejected**: most depth cameras have a ~10–28 cm minimum range, so directly overhead at grasp distance we'd likely be inside the blind zone — depth would fail exactly when needed.

## Standoff & drive-up (avoiding collision)
The drive-up sometimes gets too close, occasionally shoving the cube with the body. Root cause is momentum + control latency causing overshoot. The claw motor box protrudes **~3 cm below the claw**, so if the cube is too close it clips the cube/body during the reach. Fixes:

- **Bias to the far side (asymmetric cost).** Overshooting is catastrophic (clips/shoves the cube); stopping short is harmless (the arm reaches forward). Tune the stop threshold conservatively so the robot always errs *far*, not close.
- **Decouple body distance from grasp precision.** The drive-up only needs to park the cube inside the arm's reachable workspace *with clearance* (near edge = motor-box clearance limit, far edge = max reach). The XY homography correction (step 4) mops up the residual, so the drive-up can be loose.
- **Two-phase approach.** Drive fast until a "near" threshold, then **slow-creep** to the final stop to eliminate momentum overshoot.
- **Standoff must clear the ~3 cm motor box.** Set the minimum standoff so that when the arm reaches forward-and-down, the box passes clear of the cube's near side — never crowding it from directly above with the body close.
- **Stop on a calibrated frame target.** Since it's eye-in-hand, stop when the cube's bottom edge reaches a target pixel row R (or bounding-box height ≤ H) that corresponds to the desired standoff — not "drive until the cube looks close." Calibrate R/H once at the fixed pose.
- **Recovery behaviors.** If after stopping the cube is detected too large/too close (overshoot), **back up** to the target standoff before grasping. If the cube isn't where expected (it got nudged), re-detect and re-approach rather than grabbing blind.

## Sequence
1. **Drive up** using the detect-and-drive-up skill, with a two-phase (fast → slow-creep) approach, stopping when the arm is at the fixed observation pose **`[90, 45, 45, 0, 90, 0]`** with the cube dead-centre in the camera FOV — biased to the far side. This parks the cube well ahead of the body, inside arm reach with clearance for the protruding motor box, leaving a few cm of residual XY tolerance that step 4 corrects.
2. **Snap a frame** from the (eye-in-hand) camera.
3. **Detect the block and its pose:**
   - Use the `best.pt` YOLO model as a coarse ROI finder (says *where* the block is).
   - Inside that ROI, segment by **"not-floor"** — threshold on the uniform floor color and invert, rather than thresholding on the block's own hue. This is far more robust to glare (the problem hit before): "everything that isn't floor" is stable where "everything that is cube" blows out under glare.
   - Run `cv2.minAreaRect()` on the largest contour → `(center, (w, h), angle)`.
4. **XY is solved mechanically — no homography needed.** The drive-up centres the cube in the FOV (closed loop on the detected center pixel), so the cube's XY position is already fixed and the descent pose is constant. We do *not* move the arm in XY or convert pixels to arm coordinates. Vision only needs to answer: (a) is the cube centred? (drives the stop condition), and (b) what's its rotation angle? (step 5).
5. **Rotate motor 5** to align the gripper. The block is symmetric, so orientation is only needed mod 180° → motor 5 stays within a ±90° range (no joint-limit issue). Grab **perpendicular to the long axis**: jaws straddle the 6 cm length and close across the 3 cm width. Aligning here automatically solves the "can't grab diagonal" constraint. Mapping from image angle (`minAreaRect`) → motor 5 degrees is calibrated empirically (lay cube at a known angle, read the reported angle, command motor 5, verify). Call the aligned motor-5 value **R** (R = 90 when the cube is at the default orientation). Reuse sign/direction conventions from the cube-tracker work.
6. *(Optional)* **Re-snap and verify** alignment before descending — cheap way to catch errors.
7. **Descend to the fixed grasp pose `[90, 0, 90, 0, R, 0]`** (joints 2/3/4 lower the arm; motor 5 = R carries the orientation from step 5; gripper open). Same descent every time because the observation pose and cube XY are fixed.
8. **Close motor 6 to the fixed value x = 145.** This firmly clamps the fixed 3 cm cube width (found empirically). Final grasp pose is `[90, 0, 90, 0, R, 145]`. Power-draw/current sensing was ruled out — the `/YB_Node` driver exposes **no servo load, current, or position feedback** (arm topics are command-only, no services, `/battery` is pack voltage only). The fixed value sidesteps this entirely and is reliable because the cube width never changes. `x` determined empirically (step motor 6 until firm grip).
9. **Lift**, then **rotate motor 5 back** to the original orientation. The robot now holds the cube correctly.

## Poses (reference)
- **Observation / pre-grasp:** `[90, 45, 45, 0, 90, 0]` — cube dead-centre in FOV, gripper open.
- **Descend (aligned, open):** `[90, 0, 90, 0, R, 0]` — R = orientation-aligned motor 5.
- **Final grasp (clamped):** `[90, 0, 90, 0, R, 145]` — x = **145** (fixed close value, firm grip on the 3 cm cube width, found empirically 2026-08-01).
- Gripper (joint6): `0` = open, `180` = closed.

## Detection notes
- Prefer `cv2.minAreaRect()` over hand-rolled diagonals — it returns center, size, and angle directly.
- **Segment by SATURATION inside the YOLO ROI** (high S = colored cube; low S = grey carpet AND the gripper's shadow). This is shadow-immune and color-agnostic for a solid cube on a neutral floor — better than pure-hue (glare) or "not-floor" (which catches the shadow). Value-contrast fallback for a grey/white cube.
- **No homography / IK needed.** XY is solved by centring the cube via the drive-up. Vision delivers only (a) centred? and (b) rotation angle → motor 5 (via a simple empirical angle calibration).
- Long-axis angle computed from the longest edge of the min-area rect → unambiguous [0,180).
- The observation pose views the cube **obliquely, not top-down**, so measured aspect is foreshortened (~1.7 vs true 2.0). Fine for orientation; the angle→motor-5 calibration absorbs the perspective.

## Detection script (built & validated 2026-08-01)
`detect_pose.py` (repo copy; runs on robot at `~/cube_tracker/detect_pose.py`). Grabs one frame → YOLO ROI → saturation mask → `minAreaRect`. Validated live: conf 0.68, cube box tight, shadow correctly rejected, angle 84.6°. Prints `RESULT: ok center_px=... offset_px=... long_axis_deg=...` and saves `pose_out/pose_annotated.jpg` + `pose_mask.jpg`. Helpers: `~/cube_tracker/grab_frame.py`, `set_pose.py`, `set_joint.py`. Camera: Orbbec `dabai_dcw2` via `~/models/cuboid_v1/run_camera.sh`, topic `/camera/color/image_raw`.

## Edge cases
- **Standing block (3 × 3 footprint, 6 cm tall):** footprint is square so orientation is undefined (but grabbable any direction) — *however* it is 6 cm tall, so the fixed descent would crash into it. Either confirm the block always lies flat, or add a size check on the detected rectangle to detect the standing case and abort/adjust.

## Recommended sequence (condensed)
drive up to `[90,45,45,0,90,0]` (cube centred) → snap frame → YOLO ROI → "not-floor" contour → `minAreaRect` → rotate motor 5 to R → (optional re-snap to verify) → descend to `[90,0,90,0,R,0]` → close motor 6 to 145 → lift → rotate motor 5 back.
