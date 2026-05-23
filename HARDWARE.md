# HARDWARE.md — ROSMASTER M3PRO

Hardware inventory and assumptions for the cube pick-and-place project.
All facts below were captured live from the robot over SSH on 2026-04-08
unless marked **[unverified]**.

## Robot

- **Platform:** Yahboom ROSMASTER M3PRO
- **Hostname:** `yahboom`
- **SSH:** `ssh -i ~/.ssh/id_ed25519 jetson@192.168.50.103` (MHL net),
  fallback `jetson@192.168.8.88` (ROSMASTER WiFi)

## Compute

- **Main controller:** NVIDIA Jetson Orin NX Engineering Reference
  Developer Kit Super (from `/proc/device-tree/model`)
- **Memory:** 8 GB LPDDR5 (7.4 GiB visible) → Orin NX 8 GB SKU
- **Storage:** 1 TB NVMe (`/dev/nvme0n1p1`, 915 G, 80 G used)
- **OS:** Ubuntu 22.04.5 LTS (jammy), aarch64
- **L4T:** R36.4.7 (JetPack 6.x), kernel 5.15.148-tegra
- **ROS 2 distro:** Humble Hawksbill (`/opt/ros/humble`)

## Arm

- **DOF:** **5 DOF arm + 1 DOF parallel gripper = 6 servos total**
  (corrected 2026-04-08 after URDF + arm_driver inspection). The plan
  previously stated "6 DOF"; this is wrong. URDF has 5 revolute arm
  joints (`arm1_Joint`..`arm5_Joint`) plus `rlink1_Joint` (gripper).
  `arm_driver.py` drives all 6 servos via
  `set_uart_servo_angle_array`. `ArmJoints.msg` `joint6` is the gripper.
- **Kinematic consequence:** 5-DOF cannot reach arbitrary 6D end-effector
  poses. Phase 5 grasp planning must constrain to vertical-approach
  grasps aligned with the cube's top face.
- **Description:** `yahboomcar_ws/src/yahboom_M3Pro_description/urdf/M3Pro.urdf`
- **Kinematics:** custom `arm_kin` package with `libarmkin.so` in `~/`,
  `kin_srv` service node
- **Gripper:** 2-finger parallel jaw (URDF links `llink1/2`, `rlink1/2`,
  with `llink3`/`rlink3` as auxiliary linkages)
- **TCP / end-effector frame:** URDF link `Gripping` (fixed child of
  `arm5`). There is no literal `end_effector` frame.
- **Driver:** `arm_driver` (Python node `arm_driver.py`)
- **Servo bus:** `/dev/ttyUSB0` (direct serial to arm servo controller).
  **Not** the STM32 micro-ROS pipe. Corrected 2026-04-08.
- **Per-joint feedback:** servo position commands are int16 per joint in
  `ArmJoints.msg`; **effort / current feedback not confirmed** —
  **[unverified]**, must test by echoing the arm driver's state topic
  before relying on it for grasp confirmation (Phase 5.6).
- **Max reach / payload / gripper opening width:** **[unverified]** — not
  found in URDF/config; pull from Yahboom M3PRO datasheet or measure.
  Placeholder until confirmed: ~25–30 cm reach, ~200 g payload,
  ~3–4 cm gripper opening (typical for this class).
- **URDF quirk:** joint `arm4_Joiint` (double `i`) is a typo propagated
  through MoveIt config. Not fixed (would break SRDF). Code referencing
  this joint by name must use the typo spelling. See `tf_tree_spec.md`.

## Perception

- **Primary (only) camera:** Orbbec DaBai DCW2 (RGB-D)
  - RGB USB ID `2bc5:0561`, depth sensor USB ID `2bc5:06a0`
  - Driver: `OrbbecSDK_ROS2` (built in `yahboomcar_ws`)
  - Canonical launch: `orbbec_camera/launch/dabai_dcw2.launch.py`
  - Video devices: `/dev/video0`, `/dev/video1`
  - Resolution / FOV / depth range: **[unverified]** — pull from Orbbec
    DaBai DCW2 datasheet before tuning detection.
- **Mounting — EYE IN HAND (confirmed by user 2026-04-08):** the Orbbec
  is physically mounted on `arm4` near the wrist. The URDF models this
  via `DCW2_Joint` (fixed). Camera pose is not fixed relative to
  `base_link`; pose estimation must use live TF lookups against the
  arm chain. Hand-eye calibration (task 0.6) determines accuracy.
- **Second camera:** NONE. The URDF contains a stale `Camera` link
  (with its own mesh `Camera.STL`) mounted on `base_link` via
  `Camera_Joint`. No physical camera corresponds to it. The link is
  kept in the URDF only because the MoveIt SRDF references it in
  disable-collisions entries; removing would break MoveIt. Ignore in
  all perception code.
- **Note:** `ros-humble-realsense2-camera` is installed but no RealSense
  device is connected. Ignore for this project.

## LiDAR

**Status: CONFIRMED 2026-04-08** — two physical scanning LiDARs.

- **Count:** 2 units, each with **180° field of view**.
- **Placement (user-confirmed by visual inspection):**
  - Unit 1: **front-right** of the chassis, looking forward
  - Unit 2: **back-left** of the chassis, looking rearward
  - Diagonal opposition → combined coverage = full 360°
- **Model:** **LDROBOT LD-Mini Plus ×2** per the Yahboom login banner
  (`RADAR: Tminl-plus*2`). Exact driver TBD — likely `ldlidar_stl_ros2`
  or similar; no driver package currently present in either workspace,
  which means the driver is either launched from a Docker image, from
  `~/mircoROS_agent`, from the chassis STM32 over micro-ROS, or via a
  script not yet discovered. Needs runtime trace when LiDAR is first
  activated.
- **Topics:**
  - Each unit publishes `/scan0` or `/scan1` (`sensor_msgs/LaserScan`)
  - Merged by `ira_laser_tools` → `/scan_multi` (frame: `base_link`)
  - Filtered by `yahboom_laser_filter`
  - Launch: `yahboom_M3Pro_laser/launch/laser_driver.launch.py` starts
    the merge + filter (but NOT the physical drivers themselves)
- **Merge config:** range 0.05–4.0 m, 1° angular resolution, 360°
  merged output (see
  `M3Pro_ws/src/M3Pro_core/ira_laser_tools/config/laserscan_merge.yaml`)
- **Range cap of 4.0 m** is the merge config's limit, not a sensor
  limitation. LD-Mini Plus native range is 8–12 m; the 4 m cap is a
  project choice, likely for indoor nav reliability. Can be raised
  in the merge yaml if needed.

**Still unknown:**
- Which physical serial port each LiDAR uses. The two CH340 / CP2104
  USB-serial devices we see (`/dev/mic`, `/dev/myserial`) are:
  - `/dev/myserial` (CP2104) → STM32 control board (confirmed via
    `~/start_agent.sh`, used for chassis micro-ROS at 2 Mbaud)
  - `/dev/mic` (CH340) → udev rule labels this "mic" but could
    plausibly be one LiDAR. The second LiDAR's serial path is
    unknown. May be on an additional USB-serial device not enumerated
    until the laser driver launches. Deferred to when we actually
    bring up the LiDAR stack (Phase 3 work).
- Exact LiDAR driver package (not installed/built yet).

## Low-level control board

- **MCU:** STM32 (H743 per Yahboom docs — **[unverified on this unit]**)
- **Link:** USB-serial CP2104 at `/dev/myserial`, 2,000,000 baud
- **Protocol:** micro-ROS (agent launched via `~/start_agent.sh`,
  `~/mircoROS_agent/install`)

## Network

- **Primary interface:** `wlP1p1s0` (WiFi), `192.168.50.103/24` (MHL net)
- **Fallback:** ROSMASTER AP at `192.168.8.88`
- **DDS:** FastDDS with custom profile at `~/fastdds_all_interfaces.xml`
  (UDPv4 on all interfaces, default participant)
- **ROS_DOMAIN_ID:** `30` (from `start_agent.sh`)
- **Docker:** present (`docker0`, `br-*` bridges active)

## Existing ROS 2 workspaces on the robot

- `~/yahboomcar_ws/src` — arm stack, Orbbec SDK, description, YOLOv8,
  laser driver, mediapipe, M3Pro_config / demo / MoveIt
- `~/M3Pro_ws/src` — core libs (ira_laser_tools, imu_tools,
  slam_gmapping, yahboom_laser_filter), navigation, slam_mapping,
  calibration, patrol, multi-brains, large model integration
- `~/mircoROS_agent` — micro-ROS agent for STM32 link
- `~/uros_ws` — micro-ROS workspace

Reuse these where sensible instead of rewriting drivers.

## Open items to close before Phase 1 starts

- [x] ~~Confirm LiDAR presence and model~~ — **2 × LD-Mini Plus 180° LiDARs, diagonal opposition (front-right forward + back-left rear), merged to 360°**
- [ ] Confirm arm max reach, payload, gripper opening width from datasheet
- [ ] Confirm whether arm servos report effort/current (critical for
      Phase 5.6 grasp confirmation)
- [ ] Confirm Orbbec DaBai DCW2 depth range, FOV, min range
- [ ] Confirm STM32 MCU variant (H743 assumed)
- [ ] Identify which serial port each LiDAR uses (deferred to Phase 3 bringup)
- [ ] Locate / install the actual LiDAR driver package (not present in current workspaces)

## Resolved items (log)

- 2026-04-08: Arm DOF corrected from "6" to "5 + gripper" after
  reading `arm_driver.py` and URDF.
- 2026-04-08: Arm serial bus corrected to `/dev/ttyUSB0` (not the
  STM32 micro-ROS pipe).
- 2026-04-08: Orbbec DaBai DCW2 confirmed as eye-in-hand on `arm4`
  (user visual confirmation).
- 2026-04-08: Second camera (URDF `Camera` link on base) confirmed
  absent; link kept in URDF only to avoid breaking MoveIt SRDF.
- 2026-04-08: LiDAR count and placement confirmed — **2 × LD-Mini Plus
  180° units**, front-right forward-facing + back-left rear-facing,
  merged to 360° `/scan_multi`. Two sub-items remain open (serial port
  assignment, driver package location) but are deferred to Phase 3.
