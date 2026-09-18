# Nav_Plan — LiDAR Mapping + Collision-Free Navigation (ROSMASTER M3PRO)

## Context
Two goals for this robot:
1. **Map the environment with the two LiDARs and navigate any point → any point while producing a
   map and avoiding obstacles.** (Part 1 — the focus of this plan.)
2. *(Deferred, documented only)* Use an arm sweeping pattern to build a full 3D visualization of the
   room. **Ignored for now** — noted at the bottom, not planned in detail.

**Immediate near-term scope:** the robot is sitting on a box on the desk with its **wheels in the
air** (parked there after nearly driving off the edge). So **no driving yet.** The first concrete
step is to **merge the two LiDAR scans and visualize the combined output** to confirm the sensors and
the 360° merge are healthy before any motion.

Key finding: we do **not** build SLAM/Nav from scratch. Yahboom ships a complete, matched stack on
the robot (`slam_mapping` = SLAM, `M3Pro_navigation` = Nav2), installed in `~/M3Pro_ws` +
`~/yahboomcar_ws`. Both lidars already publish live (`/scan0`, `/scan1`) via micro-ROS. This work is
**bring-up, config, tuning, and a small goal-sending interface** on that stack.

## Hardware / topic facts (verified live 2026-07-12)
- Bring-up: `sh ~/start_agent.sh` (micro-ROS, `/dev/myserial`, `ROS_DOMAIN_ID=30`). Already running on boot.
  Publishes `/cmd_vel`, `/odom_raw`, `/imu/data_raw`, `/scan0`, `/scan1`, `/battery`, `/beep`.
- Source workspaces to see the nav packages: `source ~/M3Pro_ws/install/setup.bash` and
  `source ~/yahboomcar_ws/install/setup.bash` (base `/opt/ros/humble` alone does NOT show them).
- LiDARs: 2× 360° LDROBOT T-mini Plus, ~7 Hz, each reports **angle 0 → 4.712 rad (270° arc)**,
  angle_increment ≈ 0.00944, range 0.05–12 m. Mounted diagonally (translation only, rpy=0):
  - `/scan0` → frame `laser0_frame`, base_link offset `(-0.11617, +0.09156, +0.1253)` (back-left)
  - `/scan1` → frame `laser1_frame`, base_link offset `(+0.10766, -0.09078, +0.1253)` (front-right)
- Merge: `ira_laser_tools` `laserscan_multi_merger` reprojects both into `base_link` → **`/scan_multi`**
  (+ `/merged_cloud`). Config `laserscan_merge.yaml`: 360°, 1° increment, range 0.05–4.0 m.
  `merge_multi.launch.py` also starts URDF `robot_state_publisher` (base_link→laserN_frame TF).

## STEP 1 (DONE 2026-07-12) — Merge + visualize the two LiDARs, no motion
- Tool: `Computer_vision/navigation/lidar_viz.py` — self-contained rclpy node; subscribes to
  `/scan0`+`/scan1`, reprojects each into base_link via the static offsets above (replicating the
  vendor merger), renders a top-down PNG (each lidar color-coded, range rings, heading arrow).
- Result: both lidars publish ~7 Hz; ~834 merged points; red(/scan0) & blue(/scan1) overlap exactly
  on shared surfaces → merge registration correct. No `/cmd_vel` published.
- Run: `python3 /tmp/lidar_viz.py --out /tmp/lidar_viz.png --seconds 2.5 --max-range 4.0` on the robot
  (after sourcing ROS + both workspaces), then `scp` the PNG back.

## STEP 2 (LATER, needs wheels on the floor) — Map the room (SLAM)
- `ros2 launch slam_mapping slam_toolbox.launch.py` (merge + filter + IMU + EKF + slam_toolbox).
- Drive slowly with `ros2 run yahboomcar_ctrl yahboom_keyboard`; cover walls + close loops.
- `ros2 launch slam_mapping save_map.launch.py` → save `room.pgm`/`room.yaml`; copy to `M3Pro_navigation/map/`.

## STEP 3 (LATER) — Navigate point-to-point with obstacle avoidance (Nav2)
- `ros2 launch M3Pro_navigation navigation2.launch.py map:=<room.yaml>` (Nav2 + AMCL + costmaps).
- Set initial pose (RViz "2D Pose Estimate" or `/initialpose`), then send goals via `/navigate_to_pose`.
- Costmaps fuse live `/scan_multi` → avoids mapped walls AND newly-seen obstacles.

## STEP 4 (LATER) — Small goal-sending interface (the code we own)
- `nav_goto.py x y [yaw]`: a `NavigateToPose` action client (frame `map`) reporting success/failure;
  optional named waypoints in YAML. Reference: vendor `laserscan_to_point_publisher/app_send_goal.py`.

## Deferred (Part 2, NOT planned here)
- Arm-sweep 3D reconstruction of the room. Recorded so it isn't lost; to be planned separately later.

## Verification
- Step 1 ✅: `ros2 topic hz /scan0 /scan1` ≈7 Hz; rendered PNG shows coherent partial-room outline with
  both lidars contributing and overlapping on shared surfaces.
- Later: saved `room.pgm` recognizable in RViz; A→B→C navigation arrives within tolerance and re-plans
  around an unmapped box placed in its path.
