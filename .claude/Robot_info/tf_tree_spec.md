# TF Tree Specification — Project0

Canonical runtime TF tree for the cube pick-and-place project.
Derived from `yahboom_M3Pro_description/urdf/M3Pro.urdf` (ground truth via
`check_urdf`, 2026-04-08) plus runtime publishers listed in the right
column below.

## Target tree

```
map                                   (slam_toolbox / slam_mapping pkg, runtime)
 └── odom                             (chassis odom node via micro-ROS from STM32, runtime)
      └── base_link                   (URDF root)
           ├── arm1                   (URDF, revolute, arm1_Joint)
           │    └── arm2              (URDF, revolute, arm2_Joint)
           │         └── arm3         (URDF, revolute, arm3_Joint)
           │              └── arm4    (URDF, revolute, arm4_Joiint [sic])
           │                   ├── DCW2              (URDF, fixed, DCW2_Joint)
           │                   │    └── camera_link  (static, project0/launch/static_tfs.launch.py)
           │                   │         ├── camera_color_optical_frame  (orbbec_camera driver)
           │                   │         └── camera_depth_optical_frame  (orbbec_camera driver)
           │                   └── arm5             (URDF, revolute, arm5_Joint)
           │                        ├── Gripping    (URDF, fixed, Gripping_Joint) -- TCP
           │                        ├── rlink1 → rlink2   (URDF, right finger)
           │                        ├── llink1 → llink2   (URDF, left finger)
           │                        └── rlink3, llink3    (URDF, aux finger linkages)
           ├── Camera                 (URDF leftover, no physical camera — IGNORE)
           ├── arm_base_Link          (URDF orphan, not in kinematic chain — IGNORE)
           ├── lwheel1, lwheel2       (URDF, revolute wheel joints)
           └── rwheel1, rwheel2       (URDF, revolute wheel joints)
```

## Frame name conventions used by this project

- **`base_link`** — URDF root, the chassis reference frame.
- **`end_effector` (conceptual) = URDF link `Gripping`** — the tool-center
  point used for grasp pose commands. There is no literal frame called
  `end_effector`; code should target `Gripping` directly.
- **`camera_link`** — root of the camera driver frame tree. NOT a URDF
  link; published by the `dcw2_to_camera_link` static transform
  publisher (see "Static transforms" below).
- **Optical frames** (`camera_color_optical_frame`,
  `camera_depth_optical_frame`) — published by the `orbbec_camera` ROS
  driver. Use these (not `camera_link`) when consuming raw image pixels,
  because they follow the ROS optical-frame convention (Z forward, X
  right, Y down).

## Runtime publishers required for a complete tree

| Transform(s)                              | Source node                                  | Launch file                                                                 |
|-------------------------------------------|-----------------------------------------------|-----------------------------------------------------------------------------|
| URDF internals (base_link → arm chain)    | `robot_state_publisher`                       | `yahboom_M3Pro_description/launch/display_launch.py`                        |
| Arm joint states                          | `arm_driver` (publishes `/joint_states`)      | run `arm_driver.py` directly                                                |
| `odom → base_link`                        | chassis odom node via micro-ROS / STM32       | `~/start_agent.sh` + chassis bringup (M3Pro_core)                           |
| `map → odom`                              | SLAM stack                                    | `slam_mapping` package (Phase 3 work)                                       |
| `DCW2 → camera_link` (identity placeholder) | `static_transform_publisher`                 | `~/project0/launch/static_tfs.launch.py` (on robot, project-local)          |
| `camera_link → camera_*_optical_frame`    | `orbbec_camera` driver                        | `orbbec_camera/launch/dabai_dcw2.launch.py` (or via `camera_arm_kin.launch.py`) |

## Dropped / ignored frames (documented so future work doesn't re-discover)

- **`arm_base`** — was in the plan's target tree but not in the kinematic
  chain. The arm is parented directly to `base_link` via `arm1_Joint`.
  An orphan `arm_base_Link` exists in the URDF but the arm does not go
  through it. DROPPED from the target tree on 2026-04-08.
- **URDF link `Camera`** — separate base-mounted revolute link with its
  own mesh. Physically absent on the robot. Kept in URDF because the
  MoveIt SRDF (`M3Pro_config/config/M3Pro.srdf`) references it in
  disable-collisions entries; removing it would break MoveIt. IGNORE in
  perception code; do not consume TFs from this link.
- **URDF link `arm_base_Link`** — same story. Kept because SRDF
  references it. IGNORE.

## URDF quirks (known, not fixed)

- **`arm4_Joiint`** (double `i`) — typo in the URDF. Propagated
  consistently through:
  - `M3Pro_config/config/M3Pro.ros2_control.xacro`
  - `M3Pro_config/config/ros2_controllers.yaml`
  - `M3Pro_config/config/joint_limits.yaml`
  - `M3Pro_config/config/initial_positions.yaml`
  - `M3Pro_config/config/M3Pro.srdf`
  - `M3Pro_config/config/moveit_controllers.yaml`
  Fixing in only the URDF would break MoveIt. Fixing everywhere is a
  multi-file change with non-zero risk. Left as-is. Any code that needs
  the joint name must spell it **`arm4_Joiint`**.
- **`arm4_Joint` / `arm4_Joiint`** confusion — if grep'ping code, use
  both spellings.

## Arm DOF clarification

- **5 DOF arm (arm1..arm5) + 1 DOF parallel gripper (rlink1) = 6 servos total.**
- `arm_driver.py` drives all 6 servos via `set_uart_servo_angle_array`
  over `/dev/ttyUSB0` (not via STM32 micro-ROS).
- `ArmJoint`/`ArmJoints` messages use `joint1..joint6` where `joint6` is
  the gripper.
- **Kinematic consequence:** 5-DOF cannot reach arbitrary 6D end-effector
  poses. Grasp planning in Phase 5 must constrain to reachable yaw/pitch
  and align grasps with the cube's top face (vertical approach).

## Eye-in-hand camera

- The Orbbec DaBai DCW2 is **physically mounted on `arm4`** (confirmed
  by user, 2026-04-08). This is an eye-in-hand configuration.
- **Consequences for perception code:**
  - The camera pose is not fixed in `base_link`. Every detection must
    be transformed into `base_link` using a live TF lookup at the
    detection's timestamp (use `tf2_ros.Buffer.lookup_transform`).
  - Hand-eye calibration (plan task 0.6) directly determines pose
    estimation accuracy. The URDF `DCW2_Joint` pose is nominal, not
    calibrated.
  - To detect the cube, the arm must be in a pose where the camera sees
    the ground. Define a canonical "observation pose" in Phase 5.
  - Once a grasp starts, the camera will move with the arm — cannot
    re-observe the cube during descent. Either freeze the target pose
    from the last good observation or use a fixed base camera (not
    available). Freeze-and-go is the plan.

## Static transforms added by this project

| Name                    | Parent | Child        | Values                | Source file                                     |
|-------------------------|--------|--------------|-----------------------|-------------------------------------------------|
| `dcw2_to_camera_link`   | `DCW2` | `camera_link`| identity (placeholder)| `~/project0/launch/static_tfs.launch.py` (robot) |

The identity value is a placeholder. Replace with calibrated values from
hand-eye calibration (Phase 0.6) output. Until then, pose estimation has
an unknown constant offset from the true camera optical center.

## How to bring up a complete TF tree

On the robot:

```bash
# 1. Chassis micro-ROS agent (publishes odom → base_link)
~/start_agent.sh

# 2. URDF + robot_state_publisher + joint_state_publisher (URDF internals)
ros2 launch yahboom_M3Pro_description display_launch.py

# 3. Arm driver (publishes joint_states for the arm servos)
ros2 run arm_driver arm_driver

# 4. Orbbec camera driver (publishes camera_link → optical frames + images)
ros2 launch orbbec_camera dabai_dcw2.launch.py

# 5. Project static TF bridges (DCW2 → camera_link)
ros2 launch ~/project0/launch/static_tfs.launch.py
```

Verify with:

```bash
ros2 run tf2_tools view_frames   # dumps frames.pdf
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

The second command should succeed and show a transform that changes as
the arm moves.
