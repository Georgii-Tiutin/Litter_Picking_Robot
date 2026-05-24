# `calibration+detection` — Camera Calibration & Cube Detection

Combines the camera/hand-eye **calibration** and the OpenCV **cube-detection** work that together feed into the "Cube detector v1" milestone. Branches off the base scaffolding.

## `calibration/` — Camera & Hand-Eye Calibration

Calibration for the eye-in-hand Orbbec DaBai DCW2 camera mounted on the M3PRO arm. Contains the color and depth camera intrinsics, the AprilTag (36h11) target used during calibration, the solved end-effector→camera hand-eye transform, the raw hand-eye capture data, and the scripts that capture samples (`capture_handeye.py`) and solve the calibration (`solve_handeye.py`), plus a bring-up helper.

## `perception/` — Cube Detection (Phase-0 baseline)

OpenCV color-based cube-detection — the Phase-0 perception baseline that predates the planned NVIDIA open-vocabulary stack. Holds cube detectors for blue and red cubes, HSV color-tuning tools, a scene highlighter, a robustness logger, and debug viewers, along with the HSV configuration. These are the experimental detectors developed and tuned against the live camera feed.

Continues into `cube-detector-v1`.
