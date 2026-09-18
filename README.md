# `manual-courses` — working through the Yahboom manual (after the reboot)

On 2026-08-26 the project restarted with two rules: work through the ROSMASTER M3 Pro course material in order, and reuse the code that ships on the robot instead of rewriting it. This branch holds what came out of that: the robot configuration changes, and every edit made to Yahboom's own demo code while running the courses.

The vendor code itself is not copied here. Each change is stored as a **unified diff against the original file** (the robot keeps the originals as `*.orig-backup` / `*.factory-backup`), so the branch shows exactly what was changed and nothing else. To apply one on the robot: `patch -p1` from the package's source directory.

The full reasoning for every change is in the project journal (`A_new_beginning.md` in the separate `rosmaster-docs-md` repo); the entry for each is named below.

## Robot configuration — `robot_config_backups/`

| File | What it is |
|---|---|
| `joy_control.desktop` | Gamepad autostart entry, moved out of `~/.config/autostart` so the joystick nodes stop holding the chassis and arm topics during autonomous work (2026-08-26, "Disabled gamepad autostart") |
| `offset_value.yaml.bak-20260826` | Arm camera offsets as they were **before** the 2026-08-26 offset calibration; the new values landed within ~1 mm of these |
| `README.md` | Where the files came from and how to restore them |

## Chassis course — `vendor_patches/calibration`, `patrol`, `M3Pro_demo/follow_line`, `config_robot`

| Patch | Why | Result |
|---|---|---|
| `calibration/calibrate_angular.patch` | `first_angle` started at 0, so the first tick of every test added the robot's absolute heading (up to ±180°) to the accumulator; the Ctrl-C handler called `cmd_vel()` and never stopped the robot; `None` from a tf timeout crashed the timer | Overshoot became a constant 58° ± 3° and the calibration converged (2026-09-02) |
| `calibration/calibrate_linear.patch` | Unguarded `None` from `get_position()` stopped the node starting; overshooting the target made it **reverse** at full speed, giving results in two modes 24 cm apart | Converged to 1.1, **later shown to be wrong** (see `config_robot`) (2026-09-03) |
| `patrol/patrol.patch` | `x_start` only reset after a *successful* run, so one overrun made the next run drive off; `last_angle` went stale between corners, so corners 2–4 stopped ~30° early. The first fix (`turn_angle == 0.0` guard) made the robot spin 260° unbounded; replaced with a self-clearing `spin_fresh` flag | Square with four even corners; circle closure 10 cm (2026-09-05) |
| `M3Pro_demo/follow_line.patch` | `/follow_line_key` topic so the line follower can be started and stopped without a keyboard riding on the robot; the stop also clears the buzzer, which the vendor code left sounding | 82 s continuous run; the end-of-line stop is still unreliable (one thread per frame racing on `/cmd_vel`) (2026-09-04) |
| `config_robot/config_robot.patch` | Records the final chassis scale values written to the MCU | `ros_scale_line` **reverted 1.1 → 1.0** (the 11 cm was deceleration coast, not odometry error); `ros_scale_angular` **1.06** after 1.13 over-corrected and 1.04 did no better (2026-09-05) |

## LiDAR course — `M3Pro_demo/apriltag_transport_V2`, `grasp_transport`, `color_transport`

| Patch | Why | Result |
|---|---|---|
| `apriltag_transport_V2.patch` | Three nodes the demo depends on are not on this image, so it never searched, never returned home and never released. Publishes the stored origin to `/goal_pose` and calls `unload()` on arrival. Includes the fix for a flag added during this work that nothing ever cleared | Navigate → find tag → grip → carry home → release, end to end (2026-09-07, section 15) |
| `grasp_transport.patch` | Grasp stopped 1–1.5 cm short (the stock −0.015 m margin) and the cube slipped out of the jaws | Margin +0.003 m, closure 115 → 139 |
| `color_transport.patch` | `/target_color` topic for colour selection without a keyboard; automatic unload after a grasp, because the node that normally triggers it is missing | No journal entry for this section; change described in the patch comments |

## Depth camera course — `yahboom_M3Pro_DepthCam/Edge_Detection`, `M3Pro_demo/estimate_volume`

| Patch | Why | Result |
|---|---|---|
| `Edge_Detection.patch` (install copy) | Spacebar start replaced with a 3 s auto-start and an `/edge_key` topic | Stopped at a real box edge; also found to be blind to walls by design and to treat invalid depth as "safe" (2026-09-07, section 6) |
| `estimate_volume.patch` | `/measure_now` topic to trigger a measurement remotely | Height correct (3 cm); volume maths broken (always 0.0) and **left stock by choice** (section 4) |

## Arm & 3D gripping course — `M3Pro_demo/*`

Every one of these adds a topic in place of a keypress, because the robot has no keyboard once it is on the floor. Same convention throughout: an `Int16` topic carrying the key code `cv2.waitKey` would have returned.

| Patch | Section | Notes |
|---|---|---|
| `apriltag_detect.patch` | 3 — AprilTag ID sorting | `/apriltag_key`; also `quad_decimate` 2.0 → 1.0 (full-resolution tag detection, changed 2026-09-11). Grip for the 3×3×6 cuboid found to need 142, set over the existing `/set_joint6` topic without code changes |
| `apriltag_list.patch` | 4 — height-abnormality sorting | `/apriltag_key` |
| `color_recognize.patch` | 6 — colour block sorting | `/target_color`, `/color_key`. Found it grips across an arbitrary contour edge, so a 3×6 block fails half the time |
| `color_list.patch` | 7 — height sorting, colour blocks | Same topics. The first version of the patch set only one of the two flags the spacebar sets, so nothing triggered; fixed |
| `green_colorHSV.patch` | 7 | Green preset required V ≥ 205; the blocks measured 71–105 in this room. Replaced with a range measured from camera pixels |
| `color_follow.patch` | 8 — tracking a colour block | `/target_color`. Works with the 4 cm cube only |
| `shape_recognize.patch` | 9 — shape sorting | `/target_shape`, `/shape_key`. "Shape sorting" drops every shape in the same place (`pos.id` is never set) |
| `grasp.patch` | 10 — KCF tracking | Adds the `/set_joint6` hook its sibling node has; the default drop pose opened the gripper at the start of the traverse, so every object fell. Fixed with an explicit `else` branch |

Section 11 (gesture sorting) ran unmodified.

## What was lost

Three helper scripts named in the journal lived in `/tmp` on the robot and were wiped by later reboots, so they are not here: `spin_test.py` (the angular calibration harness), `fl.sh` (remote control for `follow_line`) and `nav_status_bridge.py` (the stand-in for the missing `get_nav2_status_V2` node in section 15).
