# `arm-camera-calibration` — overhead camera ↔ arm calibration (2026-09-09/10)

A detour between the arm course and the sorter. The idea was a drawing robot that watches its own results through a feedback loop. The robot's camera is mounted on the arm and there is no servo feedback, so the rig used a fixed overhead camera (Logitech C505) and four AprilTag cubes marking the workspace. The drawing loop was never built. What came out of it was the **error budget for this arm**.

`vision/CALIBRATION.md` has the full result. In short:

| Source | Size |
|---|---|
| Tag detection | 0.0–0.3 px (negligible) |
| Arm repeatability | 1.54 mm rms |
| Backlash | 1.32 mm rms, 2.6 mm worst (approaching from −y) |
| Model generalisation (quad fit, leave-one-out) | 2.45 mm |

All of it is mechanical, none optical: achievable placement is about 2–2.5 mm.

## What went wrong on the way (kept in the history)
- **Tag size:** early numbers assumed the tag pattern filled the 30 mm cube face. It is ~21 mm, so every early coverage figure was ~40% too large.
- **Lens distortion:** the homography's own check showed four identical tags measuring 16.4–21.5 mm. Rather than calibrate the camera, the final approach fits commanded arm (x, y) to observed pixel directly.
- **First grid (`collect_grid.py`, `grid_data.csv`) is corrupted:** captures were taken before the arm had settled, and a fresh ROS 2 node per pose lost messages before discovery completed. `collect_grid2.py` fixed both.
- **Overfitting:** with 22 samples the cubic model looked best on training error and was the worst at leave-one-out; 50 samples (`collect_grid3.py`) settled it in favour of the quadratic.
- **`find_gripper.py`** (difference against a parked reference) fails because the arm entering the frame shifts the camera's auto-exposure; `jaw_locate.py` (toggle the jaws and difference) replaced it.

## Files — `vision/`
| File | Purpose |
|---|---|
| `capture.sh` | One settled frame from the C505, addressed by name (device indices shuffle) |
| `detect_tags.py`, `build_homography.py`, `homography.json` | Tag detection and image → workspace homography |
| `sweep_wrist.sh` | Finds the wrist angle that shows the gripper tag squarely (joint4 = 40) |
| `collect_grid.py` → `collect_grid2.py` → `collect_grid3.py` | Grid collection, v1 (broken) to v3 (50 poses, 2 captures each), data in `grid_data*.csv` |
| `fit_model.py`, `cross_validate.py`, `analyse.py`, `backlash_test.py` | Model fitting, leave-one-out, error analysis, direction-dependent offset |
| `arm_camera_model.json` | The fitted quadratic model |
| `find_gripper.py`, `jaw_locate.py`, `collect_floor.py`, `floor_grid*.csv` | Locating the jaws without a marker; floor-height calibration |
| `find_objects.py`, `map_objects.py`, `stack_plan.py` | Segmenting objects in the overhead view and planning pick/place; the start of the drawing/stacking idea, not pursued |
| `robot/move_to.py`, `set_joints.py`, `probe_joints.py`, `reach_map.py`, `why_fail.py` | Robot-side helpers (from `~/calib/`): guarded IK moves that wait for discovery and re-validate after clamping, raw joint commands, dry-run IK, and a map of the reachable envelope |

Captured images (~290 frames, 17 MB) stay local.
