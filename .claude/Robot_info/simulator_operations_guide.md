# M3PRO Simulator Operations Guide

Complete operational reference for running the Ignition Fortress
simulation of the ROSMASTER M3PRO on the Jetson Orin NX. Covers
launch procedures, available interfaces, known quirks, and
troubleshooting.

---

## Quick Start

```bash
cd ~/m3pro_gazebo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0          # SAFETY: isolate from real robot (domain 30)
ros2 launch m3pro_gazebo sim.launch.py rviz:=false
```

**SAFETY WARNING:** The real robot runs on `ROS_DOMAIN_ID=30`. If you
forget `export ROS_DOMAIN_ID=0`, `/cmd_vel` commands will reach the
real chassis motors and the robot WILL move physically. This happened
on 2026-04-14. Always verify domain before publishing motion commands.

## Launch Arguments

| Arg | Default | Purpose |
|---|---|---|
| `world` | `tabletop.sdf` | World file. Also: `empty.sdf` |
| `use_sim` | `true` | xacro toggle (sim plugins included) |
| `rviz` | `true` | Start RViz. **Set `false` to avoid OOM crash** |
| `entity_name` | `m3pro` | Gazebo model name |

**Critical:** always use `rviz:=false` on the 8 GB Jetson. Gazebo GUI
+ camera rendering + RViz exceeds RAM and causes a hard crash. Launch
RViz separately if needed:
```bash
export ROS_DOMAIN_ID=0
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

## What the Launch File Does (in order)

1. `UnsetEnvironmentVariable(FASTRTPS_DEFAULT_PROFILES_FILE)` — the
   Yahboom profile breaks local DDS; this removes it automatically
2. `SetEnvironmentVariable(IGN_GAZEBO_RESOURCE_PATH)` — so Gazebo
   finds `package://m3pro_gazebo/meshes/*`
3. Starts Ignition Gazebo Fortress with the selected world (`-r -v 3`)
4. Starts `ros_gz_bridge` with `config/ros_gz_bridge.yaml`
5. Spawns the robot: `xacro` → `robot_state_publisher` → `ros_gz_sim create`
6. Spawns controllers in sequence:
   `joint_state_broadcaster` → `arm_controller` → `gripper_controller`
7. Optionally starts RViz

## Available ROS 2 Topics

### Motion (kinematic base via VelocityControl)

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | publish to move | Linear x/y + angular z. Kinematic — no wheel physics |
| `/odom` | `nav_msgs/Odometry` | subscribe | From GZ `OdometryPublisher`, bridged to ROS |

### Arm + Gripper (ros2_control)

| Topic / Action | Type | Notes |
|---|---|---|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` action | 5 joints: `arm1_Joint`, `arm2_Joint`, `arm3_Joint`, `arm4_Joiint`, `arm5_Joint` |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` action | 1 joint: `rlink1_Joint`. `llink1_Joint` mimics automatically |
| `/joint_states` | `sensor_msgs/JointState` | Arm + gripper joints (NOT wheels) |

**IMPORTANT:** `arm4_Joiint` has a double-i typo. This is intentional
— the MoveIt SRDF on the real robot uses this spelling. Always use
the typo in code.

### Camera (eye-in-hand, bridged from GZ)

| Topic | Type | Notes |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB, 640×480 @ ~19 Hz |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth, 640×480 @ ~18 Hz |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | Frame: `DCW2` |

Topic names match the real Orbbec DaBai DCW2 driver — downstream
perception code is sim/real agnostic.

### Utility

| Topic | Type | Notes |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Sim clock. Use `use_sim_time: true` on all nodes |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Full URDF tree + odom→base_link |

## TF Tree (runtime)

```
odom
 └── base_link
      ├── arm1 → arm2 → arm3 → arm4 → arm5 → Gripping (TCP)
      │                         └── DCW2 (camera)
      ├── Camera (stale URDF link, no physical device — ignore)
      ├── arm_base_Link (orphan — ignore)
      ├── lwheel1, lwheel2  (static only, no joint_states)
      └── rwheel1, rwheel2  (static only, no joint_states)
```

Note: `map → odom` is NOT published in sim by default. That comes
from SLAM (Phase 3 work). For sim, `odom` is the root frame.

## Example Commands

### Move the robot forward
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```

### Spin the robot in place
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.5}}" -r 10
```

### Move the arm to a pose
```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [arm1_Joint, arm2_Joint, arm3_Joint, arm4_Joiint, arm5_Joint],
      points: [{positions: [0.5, 0.3, -0.3, 0.2, -0.2], time_from_start: {sec: 2}}]
    }
  }"
```

### Close the gripper
```bash
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [rlink1_Joint],
      points: [{positions: [0.5], time_from_start: {sec: 1}}]
    }
  }"
```

### Teleport the robot (bypass physics)
```bash
ign service -s /world/tabletop/set_pose \
  --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean \
  --timeout 2000 \
  --req 'name: "m3pro", position: {x: 0.5, y: 0.0, z: 0.05}'
```

## Architecture

### Gazebo Plugins (all in a single merged `<gazebo>` block in xacro)

| Plugin | Purpose |
|---|---|
| `ign_ros2_control::IgnitionROS2ControlPlugin` | Loads ros2_control for arm + gripper |
| `ignition::gazebo::systems::VelocityControl` | Kinematic base motion from `/cmd_vel` |
| `ignition::gazebo::systems::OdometryPublisher` | Publishes `/odom` and `odom→base_link` TF |
| `ignition::gazebo::systems::JointStatePublisher` | Publishes wheel joint states to GZ (visual animation) |

**Critical:** all model-level plugins MUST be in a single `<gazebo>`
block (no `reference` attribute). Fortress's URDF→SDF converter drops
subsequent unreferenced `<gazebo>` blocks. This cost several hours
of debugging.

### World Plugins (in `.sdf` files)

| Plugin | Purpose |
|---|---|
| `Physics` | Physics engine (Bullet) |
| `SceneBroadcaster` | GUI + client scene data |
| `UserCommands` | Runtime spawn/delete |
| `Sensors` (ogre2) | Camera rendering |
| `Contact` | Contact events (future use) |

### ros_gz_bridge (config/ros_gz_bridge.yaml)

Bridges these GZ topics to ROS:
- `/clock` GZ→ROS
- `/cmd_vel` ROS→GZ (for VelocityControl)
- `/odom` GZ→ROS
- `/camera/image` → `/camera/color/image_raw`
- `/camera/depth_image` → `/camera/depth/image_raw`
- `/camera/camera_info` → `/camera/color/camera_info`

### Physics & Collision

- **Engine:** Bullet (DART has friction issues with URDF models)
- **Wheel collisions:** spheres (radius 0.04m) — NOT meshes
- **Base collision:** raised box at z=0.08 (bottom at z=0.04) — NOT
  the original STL mesh which sat on the ground and prevented wheels
  from bearing weight
- **Ground friction:** explicit `<surface><friction>` in world SDF
  (mu=100 for both ODE and Bullet)
- **Joint damping:** 0.5 on continuous gripper linkages, 0.1 on arm
  revolute joints, 0.5 on Camera_Joint

### File Layout on the Jetson

```
~/m3pro_gazebo_ws/
├── src/m3pro_gazebo/
│   ├── urdf/m3pro.xacro           # Single-source URDF, use_sim toggle
│   ├── worlds/
│   │   ├── empty.sdf              # Smoke test world
│   │   └── tabletop.sdf           # Cube + bin world
│   ├── launch/
│   │   ├── sim.launch.py           # Top-level bringup
│   │   └── spawn_robot.launch.py   # Spawn-only (included by sim)
│   ├── config/
│   │   ├── arm_controllers.yaml    # ros2_control controller config
│   │   └── ros_gz_bridge.yaml      # Topic bridge config
│   └── meshes/                     # 20 STL files (copy of Yahboom meshes)
├── build/
└── install/
```

## Known Limitations

1. **Base motion is kinematic** — VelocityControl moves the model
   directly. No wheel-ground physics, no slip, no skid. The robot
   slides rather than rolls. Wheels may not animate in sync with motion.
2. **Wheels not in RViz TF tree** — wheel joints are not in
   ros2_control, so `joint_state_broadcaster` doesn't publish them.
3. **Gripper grasping is unreliable** — parallel gripper physics for
   small objects in Gazebo is notoriously poor. Grasp confirmation
   must be validated on real hardware.
4. **Camera noise model doesn't match real Orbbec** — sim depth is
   clean; real depth has noise/dropouts. Perception tuning against
   sim depth won't transfer 1:1.
5. **Servo effort not modelled** — arm joints are position-controlled
   only. No effort/current feedback in sim.
6. **Mecanum strafe not modelled** — VelocityControl does support
   linear.y, but there's no mecanum kinematics. Forward/back/rotate
   only for nav dev.
7. **`NvMapMemAllocInternalTagged` GPU memory warnings** — appear
   during camera rendering on the Jetson. Benign unless they precede
   a crash (which they do if RViz is also running).
8. **Stale `Camera` link visible in Gazebo** — this URDF link has
   no physical counterpart. Kept because MoveIt SRDF references it.

## Troubleshooting

### Robot doesn't spawn / "awaiting /robot_description"
- Check `FASTRTPS_DEFAULT_PROFILES_FILE` is unset (launch should
  handle this, but verify with `echo $FASTRTPS_DEFAULT_PROFILES_FILE`)
- Check `ROS_DOMAIN_ID` matches between your terminal and the launch

### Can't see sim nodes from SSH
- Match the domain: `export ROS_DOMAIN_ID=0`
- Do NOT set `FASTRTPS_DEFAULT_PROFILES_FILE`

### Jetson crashes / screen goes black
- You're running too much. Kill RViz. Use `rviz:=false`.
- Check `free -h` before launching — need ~4 GB free.

### Gripper links spinning
- Check that `<dynamics damping="0.5" friction="0.1"/>` is present
  on all continuous joints in the xacro.

### Real robot moves during sim
- **STOP IMMEDIATELY.** You're on domain 30.
- `Ctrl+C` the launch.
- `export ROS_DOMAIN_ID=0` and relaunch.

## Rebuilding After Changes

```bash
cd ~/m3pro_gazebo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select m3pro_gazebo
source install/setup.bash   # re-source after build
```

Then `Ctrl+C` and relaunch. Gazebo must be restarted for URDF/world
changes to take effect — there is no hot-reload.

## Differences Between Sim and Real Robot

| Aspect | Sim | Real |
|---|---|---|
| ROS_DOMAIN_ID | 0 | 30 |
| Base motion | Kinematic (VelocityControl) | Mecanum (micro-ROS → STM32) |
| `/cmd_vel` source | `ros_gz_bridge` → GZ | Direct to `/cmd_vel` topic |
| Arm driver | `gz_ros2_control` | `arm_driver.py` → `/dev/ttyUSB0` |
| Camera driver | GZ sensor + bridge | `orbbec_camera` driver |
| Camera topics | Same names (`/camera/color/image_raw`, etc.) | Same names |
| Arm action topics | Same names | Same names (via MoveIt or direct) |
| Wheel joint_states | NOT published | Published by micro-ROS |
| LiDAR | NOT simulated (yet) | 2 × LD-Mini Plus → `/scan0`, `/scan1` |
| Physics engine | Bullet (Fortress) | Reality |
| FASTRTPS profile | Unset by launch | Set by Yahboom `.bashrc` |
