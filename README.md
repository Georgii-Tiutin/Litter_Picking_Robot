# `navigation` — LiDAR mapping and arm-sweep floor mapping

Work towards mapping the room with the M3PRO's two LiDARs and navigating point to point while avoiding obstacles.

- `navigation/Nav_Plan.md` — the plan: LiDAR mapping and collision-free navigation (part 1), with the arm-sweep 3D room scan noted as deferred (part 2)
- `navigation/lidar_viz.py` — merges `/scan0` and `/scan1` into `base_link` and renders a top-down PNG
- `navigation/floor_sweep_map.py` — sweeps the arm left-right and stitches the eye-in-hand depth camera into a `/floor_map` point cloud (runs on the robot)
- `navigation/arm_state_pub.py` — publishes `/joint_states` at the commanded arm pose so the RViz robot model matches the real (open-loop) arm
- `navigation/lidar.rviz`, `navigation/floor_map.rviz` — RViz configs for the two views
