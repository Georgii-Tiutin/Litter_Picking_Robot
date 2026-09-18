# Litter Picking Robot — `main`

Default branch and planning home for the autonomous litter-pickup project, built on a Yahboom ROSMASTER M3PRO with a Jetson Orin NX.

This branch deliberately holds the project **scaffolding and planning documents** rather than implementation code. It contains the hardware reference (`HARDWARE.md`), repo guidance (`CLAUDE.md`), shared launch files, and the `.claude/plans/` directory with the baseline project plan, the outdoor-grass extension plan, and the pre-work cleanup / gitflow notes.

The actual modules live on dedicated branches:

- **`calibration+detection`** — camera intrinsics, hand-eye + AprilTag calibration, OpenCV/HSV cube detectors
- **`cube-detector-v1`** — the calibration + detection milestone (cube detector v1)
- **`color-tracking`** — standalone Orin color-tracking script
- **`yolo-detector`** — YOLO11n cuboid detector: training scripts, synthetic-data generator, every training run (including the ones that did worse) and the candidate models
- **`navigation`** — LiDAR mapping / navigation plan and the arm-sweep floor-mapping scripts
- **`arm-grasping`** — cuboid pose detection, wrist alignment, drive-up and grasp scripts, with the grasp plan and the record of what worked on the robot

`PROJECT_PLAN.md` is the cuboid detection and grasping plan. `tools/` has the battery-check and arm-calibration utilities. `DATA_MANIFEST.md` lists the datasets, renders and videos that are kept locally rather than on GitHub.
