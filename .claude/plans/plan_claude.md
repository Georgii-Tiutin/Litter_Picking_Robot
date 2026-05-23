# Project Plan — Autonomous Litter Pickup (Primitives Baseline)

End goal: a robot that picks up litter, generalized for the development phase as cuboids and cylinders of varied size and color. This plan covers the controlled-environment baseline. The outdoor/grass leap is in `extension_claude.md`.

The decomposition is built around the Jetson Orin NX (70 TOPS) and uses the NVIDIA stack (Isaac ROS, TensorRT, Isaac Sim, NanoOWL/SAM, FoundationPose, cuRobo) so the GPU isn't idle.

## Hardware baseline (assumed — verify before Phase 0)
- **Compute**: Jetson Orin NX 16GB (70 TOPS) — main controller
- **Low-level**: STM32H743 (motors, IMU, servos, OLED) via Micro-ROS
- **Chassis**: Mecanum (omnidirectional)
- **Sensors**: RGBD depth camera, 2D LiDAR, wheel encoders + IMU
- **Manipulation**: 6-DoF arm + gripper
- **Current state**: "Cube detector v1" exists (latest commit), likely OpenCV color-based

## Task decomposition

### Phase 0 — Foundation
- Lock JetPack version and verify Isaac ROS compatibility matrix
- Establish baselines: current detector FPS, end-to-end pick latency, arm pose accuracy
- Bring up Isaac Sim scene with the M3Pro URDF + a flat ground + scattered cuboids/cylinders → regression harness for every later phase

### Phase 1 — Generalized perception (replace color-cube detector)
- **NanoOWL** (open-vocab detection, TensorRT) — query strings like "bottle", "can", "wrapper", "cube", "cylinder". Handles varied color/size without retraining.
- **NanoSAM** — instance masks for each detection (needed for pose + grasp)
- All models compiled to TensorRT engines (`.plan` files), FP16
- Throughput target: ≥15 Hz at camera resolution on Orin NX
- *GPU load: ~1 SM cluster*

### Phase 2 — 6-DoF pose of novel objects
- **FoundationPose** (Isaac ROS Pose Estimation) — model-free 6-DoF pose from RGBD + mask. Crucial because litter shapes/textures aren't known a priori; avoids per-object training.
- Fallback for pure primitives: fit OBB to masked point cloud (cuboid = principal axes; cylinder = RANSAC cylinder)
- Publish object poses on TF in the robot base frame

### Phase 3 — Grasp planning + arm motion
- Grasp synthesis: rule-based for cuboid (top face center, gripper aligned to short edge) and cylinder (side wrap perpendicular to axis). Keep it simple before reaching for learned grasping.
- **cuRobo** (NVIDIA, CUDA-accelerated motion planning) instead of vanilla MoveIt — ~50ms plans vs seconds, collision-aware, runs on GPU
- Validate every grasp in Isaac Sim before letting it touch hardware

### Phase 4 — Navigation (indoor / structured)
- **Isaac ROS cuVSLAM** (stereo/RGBD visual-inertial SLAM) as primary localization
- **Isaac ROS Nvblox** — GPU 3D reconstruction → 2D costmap for Nav2
- Nav2 with Nvblox costmap layer for obstacle avoidance
- Approach controller: stop at a pose that places the target inside the arm's workspace

### Phase 5 — Mission / behavior layer
- BehaviorTree.CPP: `Search → Detect → Approach → Refine pose → Grasp → Verify → Deposit → Resume`
- Recovery branches: missed grasp, lost detection, navigation timeout
- Bin pose either fixed (known location) or AprilTag-tracked

### Phase 6 — Sim-to-real with Isaac Sim / Isaac Lab
- Domain randomization in Isaac Sim: lighting, primitive appearance, clutter, camera noise
- Replay logged real-world failures in sim to debug perception/grasp regressions
- If learned grasping comes later, **Isaac Lab** is where to train it

### Phase 7 — Indoor integration
- Thermal + power profile on Orin NX under full pipeline load
- End-to-end runs on real hardware in a controlled space
- Failure mode dataset → loops back into Phases 1/2

## Concurrent GPU utilization
A correctly built pipeline runs simultaneously on the Orin:
NanoOWL + NanoSAM + FoundationPose + cuVSLAM + Nvblox + cuRobo.
That actually saturates the 70 TOPS rather than letting one OpenCV thread own everything.

## Critical path
Phase 1 (perception) → Phase 2 (pose) → Phase 3 (grasp). Navigation and mission layers are comparatively standard ROS2 work once perception/manipulation are solid.
