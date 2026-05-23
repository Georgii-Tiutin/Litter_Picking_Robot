# Litter Picking Robot — `main`

Default branch and planning home for the autonomous litter-pickup project, built on a Yahboom ROSMASTER M3PRO with a Jetson Orin NX.

This branch deliberately holds the project **scaffolding and planning documents** rather than implementation code. It contains the hardware reference (`HARDWARE.md`), repo guidance (`CLAUDE.md`), shared launch files, and the `.claude/plans/` directory with the baseline project plan, the outdoor-grass extension plan, and the pre-work cleanup / gitflow notes.

The actual modules live on dedicated branches:

- **`calibration`** — camera intrinsics + hand-eye calibration
- **`perception`** — OpenCV cube detectors (Phase-0 baseline)
- **`color-tracking`** — standalone Orin color-tracking script
- **`cube-detector-v1`** — historical merge of `calibration` + `perception`
