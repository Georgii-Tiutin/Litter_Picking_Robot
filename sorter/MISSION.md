# Sweep-and-collect mission

## Task
1. Map the room — **done**, cartographer map from 2026-09-05 reused.
2. Per-room boustrophedon sweep — **done**, `rooms.json`.
3. Room centre + 10x10 cm placement zone — **done**, deepest-point centre.
4. Detect cuboids (`cuboid_best_baseline.pt`, YOLO11n) -> grasp nearest first.
5. Navigate to centre, **place** (not drop) inside the zone.
6. Return to the interrupted sweep pose and continue.

## State machine

    SWEEP ---detect---> CHASE ---in reach---> GRASP ---held---> DELIVER
      ^                   |                     |                  |
      |                   | lost                | failed x3        | placed
      |                   v                     v                  v
      +----------------- RESUME <------------------------------- RETURN

- **SWEEP**   drive lane waypoints from `rooms.json`; YOLO runs continuously
- **CHASE**   record `resume_pose`; approach the nearest detection
- **GRASP**   align, close to 142, lift, verify by re-looking at the floor
- **DELIVER** Nav2 to room centre, servo to the centre tag, place in the next free slot
- **RETURN**  Nav2 back to `resume_pose`, continue the lane that was interrupted

## Why a tag at the centre
Nav2 goal tolerance (0.10-0.25 m) is larger than the 10 cm zone, so navigation alone
cannot satisfy the spec. One AprilTag cube at the room centre gives a visual reference:
navigate roughly, then servo on the tag. The arm's 2-2.5 mm accuracy only matters once
the base is positioned by something better than odometry.

## Placement slots
Zone is 100 x 100 mm; cuboids are 30 mm. A 3 x 3 grid at 33 mm pitch fills it.
Slots are filled in a fixed order and tracked, so blocks are placed beside each other
rather than onto each other.

## Constraints carried from the course
- Arm floor reach: **x 160-250 mm, y +/-90 mm** from base. Chassis is the coarse
  positioner; drive until the block is inside that patch.
- Grip **across the short side** (`minAreaRect`); reject if outside 12-55 mm.
- joint6 = **142** holds a 30 mm block; 135 slips.
- Depth must be a **median over a patch**, never the centre pixel.
- Never leave `apriltag_detect` running - it republishes the arm pose every frame and
  silently overrides arm commands.
- IK -> FK round trip, clamp, then **re-validate**; wait for subscriber discovery.

## Place, not drop
Descend to grasp height (block centre), open jaws to 30, then retract vertically.
No opening at height - that is the vendor behaviour and it bounces blocks on carpet.

## Detection — built and validated 2026-09-10

`~/calib/detect_cuboids.py` on the robot: YOLO11n (`cuboid_best_baseline.pt`) gated by a
RANSAC floor plane fitted to the depth image.

**The model alone is not usable.** On the living-room view it fired at 0.68–0.75
confidence on *mantelpiece decorations* — the domain gap the model README predicted
(trained on iPhone footage, not the Dabai). Confidence thresholds cannot separate these.

**Geometric gating does.** A cuboid must sit on the floor, so each detection's 3D point is
tested for height above the fitted plane. Validated against known points:

| image point | distance | height above floor |
|---|---|---|
| floor, near | 0.71 m | **+0.002 m** |
| floor, mid | 1.21 m | +0.030 m |
| fireplace mantel | 2.07 m | +0.947 m — rejected |
| bookshelf | 2.11 m | +0.557 m — rejected |

Accept window 5–120 mm above the plane, 0.15–2.0 m range. Floor-plane inlier fraction
0.86–0.98. Depth is a **median over a 9x9 patch**, never a single pixel.

Camera pitch matters: at `joint2=120` (60° down) the camera sees only carpet a few
hundred mm ahead. **`joint2=170` is the search pose** — it sees across the room.
