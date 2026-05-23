# System Architecture — Project0

ROS 2 node layout, message contracts, orchestration framework, and
global state machine for the cube pick-and-place project.

## Guiding principles

1. **Reuse before rewriting.** Every component already shipping in
   `~/yahboomcar_ws` or `~/M3Pro_ws` that does a piece of the job is
   wrapped, not replaced. Project-specific code lives under
   `~/project0/` and a matching `project0_ws` ROS 2 workspace.
2. **One orchestrator, many workers.** Perception / arm / nav / logger
   nodes are dumb executors. The orchestrator owns all sequencing and
   failure handling. Workers never know about the state machine.
3. **Standard message types wherever possible** — `vision_msgs`,
   `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `moveit_msgs`.
   Project-specific messages only where no standard fits.
4. **Eye-in-hand discipline** — every pose produced by perception is
   stamped with the exact TF snapshot used to derive it. Downstream
   consumers must not assume the camera is still there.

## Node inventory

### Reused upstream nodes (already on the robot)

| Node | Package | Purpose | Comms touched by project0 |
|---|---|---|---|
| `robot_state_publisher` | `robot_state_publisher` | Publishes URDF TF tree | `/tf_static`, `/tf` |
| `joint_state_publisher` | `joint_state_publisher` | Joint state for URDF | `/joint_states` |
| `arm_driver_node` | `arm_driver` (Yahboom) | Sends servo commands to `/dev/ttyUSB0` | subscribes `/TargetAngle` (`arm_msgs/ArmJoint` or `ArmJoints`) |
| `kin_ik_fk` | `arm_kin` (Yahboom) | IK / FK service | service (name TBD after inspection, wrapped by `arm_control`) |
| `orbbec_camera_node` | `orbbec_camera` | RGB-D driver for DaBai DCW2 | publishes `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`, `/tf` (camera_link → optical frames) |
| `dcw2_to_camera_link` | `tf2_ros` static | Bridges URDF `DCW2` link to driver `camera_link` | `/tf_static` (identity placeholder until 0.6) |
| `micro_ros_agent` | `micro_ros_agent` | STM32 chassis link over `/dev/myserial` | `/odom`, `/cmd_vel`, wheel IMU topics |
| Nav2 stack | `nav2_*` | Global + local planning, AMCL, costmaps | `nav2_msgs/action/NavigateToPose` action |
| `slam_toolbox` or `slam_mapping` | slam stack | Map building (offline) + localization (online) | `/map`, `map → odom` TF |

### New project0 nodes

| Node | Lang | Purpose |
|---|---|---|
| `perception_node` | Python | Detects the cube in RGB. Publishes 2D detection + RGB overlay image for debugging. Stateless wrt trials. |
| `pose_estimator_node` | Python | Consumes detection + depth + camera_info + TF. Produces a stamped `PoseStamped` of the cube in `base_link`. Includes temporal filter. |
| `grasp_planner_node` | Python | Given a cube pose, computes a reachable pre-grasp / grasp / retreat trajectory. Wraps `kin_srv` and enforces the vertical-approach constraint (5-DOF limitation). |
| `arm_control_node` | Python | Thin executor on top of `arm_driver_node` and `grasp_planner_node`. Exposes actions: `MoveToPose`, `ExecuteGrasp`, `OpenGripper`, `CloseGripper`. Provides grasp confirmation. |
| `bin_provider_node` | Python | Loads saved bin pose from config OR detects bin via AprilTag. Publishes stamped bin pose on demand (service). |
| `orchestrator_node` | C++ | BehaviorTree.CPP root. Owns the state machine. Calls all workers via action/service clients. |
| `trial_logger_node` | Python | Subscribes to orchestrator lifecycle events + perception overlay + effort trace + TF. Writes per-trial directory per the `success_metrics.md` logging contract. |

**Total new nodes: 7.** Keep the count small; resist the urge to add a
separate node for every verb.

## Message contracts

Preference order: standard ROS 2 messages → project-specific messages
only when needed.

### Standard messages used as-is

| Topic / service / action | Type | Direction |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | orbbec → perception |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | orbbec → pose_estimator |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | orbbec → perception, pose_estimator |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | everyone |
| `/joint_states` | `sensor_msgs/JointState` | arm_driver → everyone |
| `/odom` | `nav_msgs/Odometry` | micro-ROS → Nav2 |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → micro-ROS |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM → Nav2, RViz |
| `NavigateToPose` action | `nav2_msgs/action/NavigateToPose` | orchestrator → Nav2 |

### Project0 topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/project0/cube/detection` | `vision_msgs/Detection2DArray` | perception → pose_estimator, logger | bbox + confidence + stamp |
| `/project0/cube/overlay` | `sensor_msgs/Image` | perception → logger, RViz | debug RGB with bbox drawn |
| `/project0/cube/pose` | `geometry_msgs/PoseStamped` | pose_estimator → orchestrator, logger | in `base_link`, filtered |
| `/project0/bin/pose` | `geometry_msgs/PoseStamped` | bin_provider → orchestrator | in `map` frame |
| `/project0/gripper/effort` | `std_msgs/Float32` | arm_control → logger | most recent effort reading (if available) |
| `/project0/trial/state` | `std_msgs/String` | orchestrator → logger | "phase_entered:pickup", etc. |

### Project0 services

| Service | Request / Response | Provider | Caller |
|---|---|---|---|
| `/project0/bin/get_pose` | `-` / `PoseStamped` | bin_provider | orchestrator |
| `/project0/grasp/plan` | `PoseStamped` / `trajectory_msgs/JointTrajectory` | grasp_planner | arm_control |

### Project0 actions

Actions are used for any worker call that can take > 1 s and where
the orchestrator needs feedback / cancellation.

| Action | Goal / Feedback / Result | Provider |
|---|---|---|
| `/project0/observe` | `-` / phase / `PoseStamped` (cube in base_link) | orchestrator-internal wrapping of perception + pose_estimator |
| `/project0/arm/move_to_pose` | `PoseStamped` / progress / success | arm_control |
| `/project0/arm/execute_grasp` | `PoseStamped` (cube pose) / phase / success + gripper verdict | arm_control |
| `/project0/arm/release` | `-` / - / success | arm_control |
| `/project0/arm/go_to_observation_pose` | `-` / - / success | arm_control |
| `/project0/arm/go_to_transport_pose` | `-` / - / success | arm_control |
| `NavigateToPose` (Nav2) | as standard | Nav2 |

### Project-specific .msg / .action files

Only create these if a standard type doesn't fit. Target: **zero** new
custom messages, **two** new actions:

- `project0_msgs/action/ExecuteGrasp.action`
- `project0_msgs/action/Observe.action`

Both are thin wrappers around standard types. Defer their creation
until they are actually needed (probably during Phase 5 and Phase 1).

## Orchestration framework choice

**Choice: BehaviorTree.CPP (v4, upstream of Nav2).**

Reasons:

| Criterion | BehaviorTree.CPP | SMACH |
|---|---|---|
| ROS 2 native | ✅ first-class | ⚠️ community ports only |
| Nav2 integration | ✅ Nav2 uses it internally | ❌ |
| Maintained | ✅ active | ⚠️ minimal |
| Failure recovery patterns | ✅ fallbacks, retries, reactive | ⚠️ harder to express |
| Visual debugging | ✅ Groot2 | ❌ |
| C++ required | ⚠️ yes for tree authoring | ✅ Python |
| Existing use on robot | — `behaviortree_cpp` is pulled in by Nav2 already | — not present |

The only downside (C++ for tree nodes) is mitigated because most of
the work happens in Python nodes the tree just calls via actions.
The tree itself will be ~100 lines of XML + a small C++ action-client
adapter per worker.

**Tree XML** lives at `~/project0/bt/main.xml`. C++ adapters live in
`project0_ws/src/project0_orchestrator/`.

## Global state machine

High-level phases and transitions. Each box is a BT subtree.

```
                            ┌──────────────┐
                            │    BOOT      │
                            │ init, TF,    │
                            │ home arm,    │
                            │ load bin     │
                            └──────┬───────┘
                                   │ OK
                                   ▼
                            ┌──────────────┐
              ┌────────────>│  OBSERVE     │
              │             │ go to obs    │
              │             │ pose, detect │
              │             │ + estimate   │
              │             └──────┬───────┘
              │                    │ cube_pose
              │                    ▼
              │             ┌──────────────┐
              │             │ NAV_TO_CUBE  │
              │             │ Nav2 to      │
              │             │ approach pose│
              │             └──────┬───────┘
              │                    │ arrived
              │                    ▼
              │             ┌──────────────┐
              │             │   PICKUP     │
              │             │ plan+execute │
              │             │ grasp, confirm│
              │             └──────┬───────┘
              │                    │ grasped
              │                    ▼
              │             ┌──────────────┐
              │             │ NAV_TO_BIN   │
              │             │ transport    │
              │             │ pose, Nav2,  │
              │             │ effort mon.  │
              │             └──────┬───────┘
              │                    │ arrived + retained
              │                    ▼
              │             ┌──────────────┐
              │             │    PLACE     │
              │             │ over bin,    │
              │             │ release      │
              │             └──────┬───────┘
              │                    │ released
              │                    ▼
              │             ┌──────────────┐
              │             │    VERIFY    │
              │             │ obs pose,    │
              │             │ re-image,    │
              │             │ decide       │
              │             └──────┬───────┘
              │                    │
              │              ┌─────┴──────┐
              │              ▼            ▼
              │        success=S/A    fail=B/F
              │              │            │
              │              ▼            ▼
              │         ┌────────┐  ┌────────┐
              │         │ LOG +  │  │ LOG +  │
              │         │ IDLE   │  │ RECOVER│
              │         └────────┘  └────┬───┘
              │                          │
              └──────────────────────────┘ next trial
```

### Failure transitions (fallbacks)

Each phase has a specific failure policy. Encoded in BT as
`ReactiveFallback` or `RetryUntilSuccessful` decorators.

| Phase | Failure condition | Action |
|---|---|---|
| BOOT | any subsystem down | abort, human intervention |
| OBSERVE | detection failed 3× in 5 s | retreat 5 cm, retry. After 2 retreats → RECOVER |
| OBSERVE | pose filter not converged in 2 s | retry detection once → RECOVER |
| NAV_TO_CUBE | Nav2 reports failure | Nav2 recovery behaviors → retry once → RECOVER |
| NAV_TO_CUBE | approach pose unreachable (IK check fails) | re-observe → pick a different standoff → retry once → RECOVER |
| PICKUP | IK infeasible | adjust base pose by 2 cm + retry once → RECOVER |
| PICKUP | grasp confirm fails (effort mismatch) | open gripper, lift 3 cm, re-observe, retry once → RECOVER |
| NAV_TO_BIN | effort drops during transit → cube lost | stop, go to OBSERVE (search) → RECOVER if still lost |
| NAV_TO_BIN | Nav2 failure | recovery → retry → RECOVER |
| PLACE | collision predicted | raise release height 1 cm → retry → RECOVER |
| VERIFY | cube not visible in bin | mark trial as B (failure), log, return to IDLE |
| Any | wall-clock budget exceeded (`success_metrics.md`) | abort phase, RECOVER |
| Any | e-stop / unexpected exception | safe stop (freeze arm, cancel Nav2), human intervention |

### RECOVER subtree

Common recovery routine triggered by any phase fallback:

1. Open gripper (drop whatever we're holding — safer than arm stall).
2. Move arm to **transport pose** (tucked, known-safe).
3. Cancel any active Nav2 goals.
4. Request fresh localization.
5. Return to OBSERVE phase.
6. If RECOVER itself fires twice in one trial → abort trial, log F,
   wait for human.

## Launch composition

One top-level bringup launches everything; subsystems can also be
launched individually for debugging. Proposed top-level:
`~/project0/launch/project0_bringup.launch.py`

```
project0_bringup
├── micro_ros_agent                  (chassis link)
├── robot_state_publisher + JSP      (URDF)
├── orbbec_camera dabai_dcw2         (camera driver)
├── static_tfs.launch.py             (DCW2 → camera_link)
├── arm_driver_node                  (arm servo bus)
├── kin_ik_fk (arm_kin)              (IK/FK service)
├── slam_toolbox localization        (map → odom)
├── nav2 bringup                     (planning stack)
├── project0 perception_node
├── project0 pose_estimator_node
├── project0 grasp_planner_node
├── project0 arm_control_node
├── project0 bin_provider_node
├── project0 trial_logger_node
└── project0 orchestrator_node       (last — waits for the rest)
```

Lifecycle management for project0 nodes can be added later (Nav2 uses
LifecycleNode) — for now use regular nodes and rely on the
orchestrator's readiness checks in BOOT.

## What is explicitly NOT in the architecture

To avoid over-engineering:

- No microservices over HTTP. Everything is ROS 2.
- No custom DDS profiles beyond the one already at
  `~/fastdds_all_interfaces.xml`.
- No lifecycle nodes for project0 workers (yet).
- No separate node per phase — the orchestrator is a single BT node.
- No database. Trial logs are flat files on disk.
- No web dashboard. RViz + Groot2 + `ros2 topic echo` are enough.

## Open questions

- **Effort/current reading from arm servos:** required by `arm_control`
  for grasp confirmation. If unavailable (0.1 open item), fall back to
  a "gripper closed past expected width + cube still visible in
  overlay" check. Design allows either source; flip in config.
- **Bin detection vs saved pose:** `bin_provider_node` starts as
  "load from config", AprilTag detection is a v2 upgrade.
- **C++ vs Python for orchestrator:** leaning C++ for BT.CPP fluency;
  can switch to `py_trees_ros` if C++ friction is too high. Revisit
  after first orchestrator prototype.
