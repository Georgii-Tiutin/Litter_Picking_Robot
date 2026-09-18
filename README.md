# `arm-grasping` — cuboid pose detection and grasping

Scripts for picking up a 6 × 3 × 3 cm cuboid with the ROSMASTER M3PRO arm. The claw opens only 6 cm, so it must grip across the 3 cm width, which means lining up the wrist with the cuboid before every grasp. They run on the robot (ROS 2, `ROS_DOMAIN_ID=30`), using the YOLO baseline from `yolo-detector` for the region of interest.

`arm/grasp_plan.md` is the plan; `arm/movement_basics_completed.md` records what was built, what worked, and what didn't.

| Script | What it does | Status |
|---|---|---|
| `detect_pose.py` / `detect_pose_live.py` | YOLO ROI → saturation segmentation → `minAreaRect`: cuboid centre and angle (single shot / live on the robot screen) | Works; ignores the gripper shadow |
| `track_motor5.py` | Rolls the wrist (motor 5) to follow the cuboid's angle live | Works |
| `track_and_grab.py` | `track_motor5` plus descend → close at 145 → lift | Grasps **missed**: see below |
| `grasp_sequence.py` | Whole grasp from one persistent node (fresh nodes drop the first arm command) | Works |
| `drive_base.py` | Closed-loop base move using odometry | Works, ~1 cm precision |
| `diag_base.py` | Base motion diagnostic | Diagnostic |
| `drive_up.py` | Arm held at the observation pose while the base servos to the cuboid | Superseded by `align_to_obs.py` |
| `center_arm.py` | Arm-only visual servo keeping the cuboid centred | Works |
| `cube_arm_tracker.py`, `run_cube_move.sh`, `stop_cube.sh` | Earlier eye-in-hand tracker: arm follows the cuboid, base faces it and drives in; launch and safe-stop wrappers | Superseded by `center_arm.py` / `align_to_obs.py` |
| `set_pose.py`, `set_joint.py` | Send a full 6-joint pose (`/arm6_joints`) or move one joint (`/arm_joint`) | Utility |
| `grab_frame.py` | Save one camera frame to disk | Utility |
| `align_to_obs.py` | Arm centres the cuboid, base unwinds the arm back to the observation pose `[90,45,45,0,90,0]` | Works |

## The open failure
Every grasp missed even with the cuboid perfectly centred. The jaws land about (+5, −27) px from the image centre, so they came down just beyond the cuboid and pushed it away. The fix (centre on that grasp spot instead of the image centre) was identified but not applied before the reboot.
