# Cuboid detector models — for robot deployment

Two candidate YOLO11n cuboid detectors. Both are single-class object detectors.
Pick one to run on the Jetson Orin NX (or try both and compare on-device).

| File | What it is |
|---|---|
| `cuboid_v3_clean.pt` | YOLO11n, **real images only**, trained on label-cleaned data. |
| `cuboid_twostage_rooms.pt` | YOLO11n, **two-stage**: pretrained on realistic synthetic "room" renders, then fine-tuned on real. |

## Model spec (both)
- **Framework:** Ultralytics YOLO (`.pt`, PyTorch). Architecture **YOLO11n** (~2.6M params).
- **Classes:** 1 → `0: cuboid`.
- **Trained image size:** 640.
- **Task:** axis-aligned bounding-box detection (not OBB, not segmentation).

## How to run
```bash
# PyTorch (any machine)
yolo predict model=cuboid_v3_clean.pt source=0 imgsz=640      # source=0 = camera

# On the Jetson Orin NX — export to TensorRT ON THE JETSON for real-time speed:
yolo export model=cuboid_v3_clean.pt format=engine            # -> cuboid_v3_clean.engine
yolo predict model=cuboid_v3_clean.engine source=<camera/topic>
```
(TensorRT export must run on the Jetson itself, not on the Mac.)

## How these fit the robot pipeline (see ../PROJECT_PLAN.md)
- The detector only outputs **2D boxes of "cuboid"**. It does NOT measure size or colour.
- **Graspability / size:** a detected cuboid is only a valid target if its narrowest real
  dimension ≤ ~5.5 cm (6 cm claw). Enforce this with a **depth size-check** at grasp time
  (depth camera + known camera geometry) — the detector is deliberately biased toward
  small-in-scene cuboids but cannot measure cm from RGB alone.
- **Colour:** read separately via HSV inside the detected box (not a learned class).
- **Grasp pose:** top-down only → need (x, y) from ground-plane raycast/depth, yaw from the
  top-face rectangle, z from table height + cube height.

## Honest status / caveats (as of 2026-07-02)
- Both models **fixed the black-bin false positive** that the earlier baseline had
  (mainly thanks to label cleaning).
- **Which model is better is NOT yet decided.** The current validation set uses
  model-assisted (pseudo) labels, which makes cross-model mAP comparison circular.
  A human-verified val set is still needed to rank them fairly. Treat both as candidates.
- Trained/validated on **iPhone footage** (`IMG_*`, 1920×1080), not the robot's Dabai DCW2
  camera — expect some domain gap on the real robot; a short on-robot capture + fine-tune
  will likely help.
