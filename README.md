# `sorter` — sweep-and-collect mission (2026-09-10 to 09-16)

The project goal after the reboot: the robot finds cuboids around a room, picks each one up and places it in a collection area. The task as specified:

1. map the room (reuse a map if one exists)
2. build a sweep pattern for each room
3. find the rough centre of the room and a small placement area there
4. detect cubes with `best.pt` + the depth camera and grab the nearest one
5. navigate to the centre and **place** (not drop) the cube
6. return to where the chase started and carry on sweeping

The demo cubes carry no AprilTags and their colour is unknown, so both vendor grasp pipelines are ruled out: `best.pt` + depth is the only perception path. The commit history follows the build day by day. Where an earlier version of a file survived (as a `.pre-*` backup), it is committed first and replaced in a later commit, so `git log -p sorter/<file>` shows how it changed.

## Status at 2026-09-16

| Step | State |
|---|---|
| 1. Map | Done: `explore.py` frontier mapper; map in `maps/office_2026-09-12.*` |
| 2. Sweep pattern | Done for the first map (`map_analysis.py`, `rooms.json`); the patrol uses waypoints from the recorded map instead |
| 3. Centre / drop zone | Done: `find_dropzone.py`, reachable-checked |
| 4. Detect + grasp | Detection and tracking work (5 confirmed for 5 cuboids). **Grasping is not solved**: the jaws arrive aligned and centred but nudge the 3 cm cuboid. Grip height is the suspect, not yet measured |
| 5. Carry + place | Proven once end to end (cube #1, 2026-09-13); `place_cube.py` places gently |
| 6. Resume sweep | Not started |

**Correction to earlier figures:** the 75% grasp rate reported on 2026-09-15 is not valid. The success check counted "cube no longer visible" as a grasp, and cubes pushed out of the near camera view counted as held. Fixed on 2026-09-16; the real rate has not been re-measured.

## Main scripts — `sorter/`

| Script | Purpose |
|---|---|
| `explore.py` | Autonomous frontier-exploration mapper (cartographer + Nav2), with stuck detection, blacklist and battery guard |
| `estop.py` | Stop that cancels Nav2 as well as the client, then measures that the robot has actually stopped |
| `nav_camera_tf.py` | Publishes the depth camera's transform so its point cloud can feed the costmap |
| `view_map.launch.py`, `localise.py`, `check_localisation.py` | Show the recorded map, localise globally by rotating in place, check the scan against the map |
| `scan_restamp.py` | Replacement laser filter that clamps future-stamped scans (the clock drifts with uptime) |
| `cube_patrol.py` | Patrol waypoints, 360° look at each, detect with `best.pt` + depth, place cubes by floor projection |
| `cube_tracker.py`, `test_cube_tracker.py` | Multi-object association (Hungarian assignment, tentative/confirmed tracks); tests run without the robot |
| `cube_markers.py`, `read_markers.py` | Durable RViz markers for found cubes, colour-coded by evidence |
| `find_dropzone.py` | Clearest reachable 15 × 15 cm of known floor |
| `stationary_pickup.py` | The grasp: pitch search validated by IK→FK, wrist rule, span guard |
| `servo_grasp.py` | Visual servo (forward + strafe) until the cube is inside the arm's reach zone, then grasp |
| `collect_cubes.py` | The full mission: approach, grasp, carry, place, with per-phase costmap settings |
| `place_cube.py` | Put a held cube down gently and return to the navigation pose |

Supporting and diagnostic scripts:
- **Search and grasp:** `detect_cuboids.py`, `scan_room.py`, `chase.py`, `cuboid_grasp.py`, `grasp_depth.py`, `grasp_hold.py`, `grab_in_place.py`.
- **Wrist and position sweeps:** `sweep_j5.py`, `sweep_pos.py`, `calib_j5.py`.
- **Costmap investigation (2026-09-12):** `why_stuck.py`, `dump_costmap.py`, `floor_hist.py`, `tf_parents.py`, `attribute.py`, `decay.py`, `cubes_in_costmap.py`.
- **Checks:** `check_beanbag.py`, `verify_camera_tf.py`, `tf_survives.py`, `stamp_chain.py`, `dryrun.py`, `why_nothing.py`, `cube_spread_test.py` (written, not yet run), `vision_view.py`, `robot_view.rviz`.

`SORTER_DESIGN.md` and `MISSION.md` are the design notes written at the start.

## Nav2 configuration — `sorter/nav2_params/`

Diffs against the vendor `M3Pro_navigation/param/yahboom_M3Pro.yaml`, in the order they were applied:

1. `1-depth-camera` adds the depth point cloud as an obstacle source (the file was also re-serialised, so most of this diff is quoting and blank lines).
2. `2-voxel-fix` disables `voxel_layer` on both costmaps. It ran alongside `obstacle_layer` and filled the costmap with lethal cells it could not clear.
3. `3-cubes-and-amcl` sets `min_obstacle_height` 0.03 → 0.02 so a 3 cm cuboid registers, `raytrace_min_range` 0.45, and AMCL `transform_tolerance` 0.5.

The launch file loads a different params file by default (`yahboom_M3Pro_carto.yaml`), so pass `params_file:=` explicitly.

## Data — `maps/`, `logs/`

- `maps/office_2026-09-12.*`: the recorded map; `free_thresh` 0.25 → 0.1 so unknown cells stay unknown.
- `maps/cubes_*.txt`: cube lists from each patrol and collection run.
- `logs/`: patrol, collection, servo-grasp and restamp logs from the runs described in the commits.

Camera frames and grasp photos stay local.
