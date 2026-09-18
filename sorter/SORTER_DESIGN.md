# Cuboid sorter — semi-closed-loop design

Goal (due 2026-09-26): the robot roams, finds cuboids, navigates to each avoiding
obstacles, grasps it, carries it to a bin area, and drops it in the sub-area for its
colour.

"Semi-closed loop" here means **three loops running at different rates**, only the inner
two of which run continuously on the robot. The slowest loop — the one that corrects
systematic error — runs occasionally at a fixed station, because the instrument that
measures it cannot travel.

---

## The fact that drives the whole design

**The arm reaches the floor only in a 102 x 177 mm patch, at x = 160–250 mm, y = ±90 mm
from its base.** Measured 2026-09-09, 25 poses, 4.1 mm residual.

The limit is not distance but the *combination* of low and far: reaching down and out
drives joint2 negative, it clamps at 0, and the target is missed by 16–39 mm.

Consequence: **the chassis is the coarse positioner, the arm is only the fine one.** The
robot must drive until the cuboid lies inside that patch. Everything below follows from
this.

---

## Loop 1 — approach (inner, ~10 Hz, onboard only)

Onboard camera detects the cuboid; chassis drives until the block sits in the reach patch.

- Sensor: Dabai colour + depth
- Target: the vendor's own 215–225 mm window already lands inside the patch
- Already exists: `color_recognize` does exactly this
- **Fix required:** depth must be a **median over a patch**, not the single centre pixel.
  Every demo in the course (edge detection, colour sorting, KCF, apriltag) fails the same
  way on one bad pixel, and the manual even documents "rotate the block to get valid depth"
  as a user workaround.

## Loop 2 — grasp and verify (middle, per object, onboard only)

Act, then check, then retry. This is the loop the vendor code entirely lacks.

1. Compute grip axis from `minAreaRect` **short side**; reject if outside 12–55 mm
   (section 6: the vendor aligns to an arbitrary contour edge and fails on 3x6 blocks).
2. Close to **joint6 = 142** (measured: 135 slips on 3x3x6, 142 holds).
3. Lift 40 mm, then **re-look at the floor where the block was**.
   - gone -> held, proceed
   - still there -> grasp failed, re-approach (max 3 tries, then skip and log)
4. There is **no servo feedback on this robot** (`/joint_states` does not exist), so vision
   is the only possible confirmation. This step is what converts a silent failure into a
   retry.

## Loop 3 — deliver (outer, per object, Nav2)

Navigate to the bin area for the block's colour, drop, verify, resume search.

- Nav2 handles obstacle avoidance; maps already built (cartographer preferred)
- Bin areas are named poses on the map: `red`, `yellow+blue`, `rest`
- After the drop, confirm the gripper is empty by looking down before resuming

## Loop 0 — calibration station (slowest, occasional, overhead camera)

The overhead C505 cannot travel with the robot, so it becomes a **fixed station** the robot
returns to — after N objects, or after repeated grasp failures.

At the station, with the four corner tags defining the world frame:

1. Robot parks at a known pose (its own tag or a marked spot).
2. Commanded arm pose vs observed gripper pixel gives the **current** offset.
   Gripper is located by **toggling the jaws and differencing** — the only method that
   survives the C505's auto-exposure, which shifts 30 grey levels when the arm enters frame.
3. Fit the correction; write it to a file the onboard grasp node reads.
4. Re-measure the HSV thresholds for the day's lighting while parked
   (factory green demands V >= 205; this room measures 71–105 — green and yellow can
   *never* be detected as shipped).

This is the "semi" in semi-closed: the robot runs open-loop against a **calibration it
trusts**, and that calibration is refreshed periodically against ground truth rather than
continuously.

---

## Why this shape and not full autonomy

The measured error budget says where effort pays:

| source | magnitude | fix |
|---|---|---|
| tag/vision detection | 0.0–0.3 px | nothing needed — the camera is not the problem |
| arm repeatability | 1.54 mm | irreducible |
| backlash (direction-dependent) | 1.3 mm rms, 2.6 mm worst | always approach from one direction |
| model generalisation | 2.45 mm | more calibration samples |
| **vendor grasp offset** | **22–32 mm** | **loop 0 — this is the whole prize** |

The systematic offset is an order of magnitude larger than everything else, it is
*constant*, and it is only measurable with an external instrument. So: measure it rarely,
apply it always.

---

## Build order for the remaining time

1. **Median-patch depth** in the colour grasp path (loop 1) — small change, removes the
   most common failure in the entire course.
2. **Grasp verify + retry** (loop 2) — the single biggest reliability win.
3. **Bin poses + delivery** (loop 3) — composes existing Nav2 waypoint work.
4. **Search behaviour** — rotate and scan; reuse the apriltag rotation-search already
   patched in chapter 15.
5. **Station calibration** (loop 0) — everything needed already exists in `vision/`.

## Standing hazards (all observed, all cost time already)

- `apriltag_detect` republishes the arm pose every frame and **silently overrides external
  arm commands** — never leave it running alongside custom motion.
- `kill <pid>` on a `ros2 run` leaves the node child alive, re-parented to init. Verify with
  `ps -eo pid,ppid,cmd` and kill the child.
- Publishing from a freshly created node loses messages until discovery matches the
  subscriber — wait on `get_subscription_count()`.
- IK never reports failure. Always **IK -> FK round-trip**, and **re-validate after
  clamping**, not before.
- Battery: alarm at 10.5 V; working range 12.0–12.6 V.
