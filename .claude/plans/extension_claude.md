# Extension — Generalized litter pickup from real grass

This extends `plan_claude.md`. Phases 0–7 deliver pickup of cuboids/cylinders in a controlled environment. This Extension covers the leap to a real outdoor lawn with real litter (bottles, cans, wrappers, etc.). Grass breaks several assumptions in the baseline plan, so it gets its own decomposition.

## Up-front hardware caveat
**Mecanum wheels are hostile to grass** — the small rollers jam on blades and lose traction. Decide early:
- Restrict operation to short / manicured grass, or
- Swap to a differential / skid-steer base, or
- Accept mecanum as a hard performance ceiling.

Resolve this before investing in E2.

## Decomposition

### E1 — Outdoor perception robustness
- **Failure modes to solve**: green-on-green camouflage, partial occlusion by blades, direct sun + harsh shadows, wind-induced motion, real litter (bottles, cans, wrappers) ≠ clean primitives
- Re-prompt **NanoOWL** with real classes: `"bottle", "can", "plastic wrapper", "cigarette butt", "paper"`
- **NanoSAM** mask refinement to ignore intersecting grass blades
- HDR / auto-exposure in the camera pipeline; lock white balance per session
- GPU **depth completion** (Isaac ROS ESS or a small UNet in TensorRT) to fix noisy RGBD on grass
- Multi-frame detection fusion (objects are static, viewpoint isn't) to suppress wind-induced false positives

### E2 — Traversability & outdoor SLAM
- **Semantic costmap**: GPU segmentation (SegFormer / BiSeNet via TensorRT) labels `grass / path / obstacle / litter` → feed into **Nvblox** as a traversability layer
- 2D LiDAR returns demoted to advisory only — grass blades fill its costmap with phantom obstacles
- **cuVSLAM** (visual-inertial) as primary localization; loosen feature thresholds for low-texture lawn
- Optional **GPS fusion** via `robot_localization` EKF for global drift correction over larger areas
- Coverage planner: bounded lawn-mower pattern (geofence by GPS or visual fiducials at corners)

### E3 — Grasping through grass
- **Pre-grasp pose** hovering above the object, then slow vertical descent through the canopy
- **Force / current feedback** on gripper close (read STM32 motor current via Micro-ROS) — distinguishes "grasped object" vs "grasped grass"
- Retry policy: shake / twist / re-approach when blades wedge in the gripper
- **cuRobo** with inflated ground-plane tolerance — grass is compliant, hard collision checks will falsely abort plans
- Gripper-design honest assessment: a 2-finger parallel gripper struggles with floppy wrappers. Consider a passive scoop or suction adapter for true generality. This is a mechanical, not software, problem.

### E4 — Synthetic data + sim2real (the NVIDIA-stack force multiplier)
- **Isaac Sim** scene: procedural grass mesh (PBR shader, wind animation), HDRI sun, randomized lawn varieties
- Drop in **3D-scanned real litter** (or asset-store models) at varied poses, partial burial
- **Replicator** for domain randomization → millions of auto-labeled RGB + depth + mask + pose frames
- Use this to:
  - Fine-tune / distill a smaller detector for on-device speed
  - Validate FoundationPose on cluttered grass before field trials
  - Train the traversability segmentation model for E2
- This is the single biggest payoff of the NVIDIA stack for grass — real labeled outdoor data is expensive; synthetic is free.

### E5 — Outdoor mission layer
- Extend the BehaviorTree from Phase 5 with: `CoverageScan`, `ReturnToBin`, `LowBatteryReturn`, `WeatherAbort`
- Stationary bin location (AprilTag-marked) or on-board hopper that the robot empties manually
- Battery-aware planning: budget travel-to-dock distance against SOC

### E6 — Field hardening
- **Thermal**: Orin NX throttles in direct sun — characterize under full pipeline load, add active cooling or shade if needed
- **Ingress**: dev-kit Orin carrier is not weatherproof — minimum splash protection
- **Safety**: hardware E-stop, watchdog on perception/control loops, geofence enforced in software
- **Operating envelope**: gate on weather (no rain), light level (no dusk), grass height (capped)

### E7 — Field validation loop
- Metrics: pickups / hour, false-grasp rate, time-to-first-detection, grass-vs-litter confusion matrix
- Every field failure → logged RGBD clip → replayed in Isaac Sim → fix → re-deploy
- This loop is the project's actual engine once E1–E6 are functional.

## Critical path through the Extension
**E4 → E1 → E3.**
Synthetic data unblocks perception training; perception unblocks grasping; navigation (E2) and mission (E5) are comparatively standard once perception is solid. E6 / E7 are operational, not research.
