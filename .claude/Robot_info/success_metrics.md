# Success Metrics & Timeout Budgets — Project0

Initial targets for the cube pick-and-place project. Numbers are
working defaults, not contracts — refine as real data comes in.

## Overall success criterion

- **End-to-end success rate: ≥ 90% over 10 consecutive trials**
  from a reset starting pose.
- A "success" = cube is detected, transported, and released such that
  it remains fully inside the bin when the robot returns to the
  observation pose and re-images the scene (Phase 8 check).
- Trials are run back-to-back with the same starting layout (cube
  within a defined pickup zone, bin in a fixed location) unless a
  variation is explicitly called out.

### Graded success tiers (for intermediate milestones)

| Tier | Meaning | Target |
|---|---|---|
| S | Cube in bin, robot back at observation pose, no human intervention | ≥ 90% |
| A | Cube in bin, but cycle required a recovery (re-plan, re-grasp) | counted as success for overall metric, logged separately |
| B | Cube not in bin but no collision / damage | counted as failure, logged with failure-reason tag |
| F | Collision, dropped cube off-table, arm stall, e-stop, timeout | counted as failure, highest priority to investigate |

## Per-phase metrics

### Phase 1 — Object detection

| Metric | Target | How measured |
|---|---|---|
| True positive rate at observation pose | ≥ 98% | 50 frames with cube present, count detected |
| False positive rate | ≤ 1% | 50 frames without cube, count spurious detections |
| Detection latency (per frame) | ≤ 50 ms | median over 100 frames |
| Bbox IoU vs hand-labeled ground truth | ≥ 0.75 | 20 labeled frames |

Notes: metric applies at the **observation pose** with the arm held
still. Moving-arm detection is not a requirement — the plan is
freeze-and-go (see `Keep_in_mind`).

### Phase 2 — Position estimation

| Metric | Target | How measured |
|---|---|---|
| Cube 3D position error in `base_link` (X, Y) | ≤ 10 mm after 0.6 calibration | place cube at 5 measured ground-truth positions, record estimate, report RMSE |
| Cube 3D position error in `base_link` (Z) | ≤ 5 mm | same |
| Yaw estimate error (for grasp alignment) | ≤ 10° | same |
| Temporal jitter at rest (pose std-dev) | ≤ 2 mm | 30 static frames |

**Important:** these targets assume Phase 0.6 hand-eye calibration has
been completed. Before 0.6, pose has an unknown constant offset and
cannot be validated against ground truth.

### Phase 3 — Environment mapping

| Metric | Target | How measured |
|---|---|---|
| Map coverage of operational area | 100% of pickup zone + bin + path between | visual inspection in RViz |
| AMCL localization RMSE vs ground truth | ≤ 5 cm position, ≤ 5° heading | drive to 3 marked poses, compare |
| Bin pose stability (AprilTag or saved) across runs | ≤ 2 cm drift | 5 runs |

### Phase 4 — Navigation to cube

| Metric | Target | How measured |
|---|---|---|
| Arrive-at-approach-pose success | ≥ 95% | 20 runs |
| Final position error vs commanded approach pose | ≤ 5 cm | same |
| Final heading error | ≤ 10° | same |
| Obstacle avoidance (unplanned obstacle injected) | no collision in ≥ 95% of 10 trials | injected obstacle trials |

### Phase 5 — Pickup

| Metric | Target | How measured |
|---|---|---|
| IK-reachable grasp pose found | ≥ 98% | per trial, log MoveIt result |
| Grasp success (cube lifted and retained to transport pose) | ≥ 90% | 20 trials |
| Grasp confirmation (effort or vision check agrees with physical state) | ≥ 95% correct | compare sensor verdict to human observation |
| Drop rate during retreat | ≤ 5% | 20 trials |

### Phase 6 — Navigation to bin

| Metric | Target | How measured |
|---|---|---|
| Arrive-at-bin success | ≥ 95% | 20 runs |
| Cube retained during transit | ≥ 98% | effort monitor + post-run inspection |

### Phase 7 — Placement

| Metric | Target | How measured |
|---|---|---|
| Release pose reached without collision with bin rim | ≥ 98% | 20 trials |
| Cube lands inside bin footprint | ≥ 95% | 20 trials |

### Phase 8 — Verification

| Metric | Target | How measured |
|---|---|---|
| Verification check agrees with human observer | ≥ 98% | 20 trials, compare Claude's verdict to reality |
| Debug image + log saved per trial | 100% | filesystem check |

## Timeout budgets

Per-phase wall-clock budgets from phase entry to phase exit. On
timeout, the orchestrator aborts the phase and transitions to the
failure-recovery state.

| Phase | Budget | Rationale |
|---|---|---|
| 1 — Detection (at observation pose) | 3 s | RGB-D processing + tracking settle |
| 2 — Position estimation (filtering) | 2 s | after detection, running filter to convergence |
| 3 — (one-time, offline) map build | N/A | done before trials |
| 4 — Navigation to cube (approach pose) | 45 s | 2 m nav + approach + align |
| 5 — Pickup (plan + execute) | 20 s | IK + pre-grasp + descent + close + lift |
| 6 — Navigation to bin | 45 s | symmetric with phase 4 |
| 7 — Placement | 15 s | approach + descend + release + retreat |
| 8 — Verification (return to observation + image) | 15 s | nav back + image + decision |
| **End-to-end total** | **≤ 150 s (2.5 min) per cycle** | sum of above with small margin |

Trial loop target: 10 cycles in ≤ 30 minutes including inter-trial
reset.

## Data to log per trial

For every trial, regardless of success, persist to disk:

- Trial ID, timestamp, cube start pose (from detection), bin pose
- Per-phase entry/exit timestamps and outcomes
- Final verdict (S / A / B / F) and failure-reason tag if not S
- Observation-pose RGB image (post-detection overlay)
- Gripper effort trace during pickup and transport
- TF snapshot at the moment of grasp
- Any MoveIt planning failures (error codes + joint states)

Log directory convention: `~/project0/trials/<YYYY-MM-DD>/<trial-id>/`
on the robot. Pull to Mac after each session.

## Known biases that will affect early metrics

- **Before Phase 0.6 (hand-eye calibration):** Phase 2 position error
  will have an unknown constant offset. Do not validate Phase 2
  numbers against ground truth until 0.6 lands. Phase 1 (detection)
  and Phases 3–4 (navigation in the map) are unaffected.
- **Before LiDAR question is resolved (0.1 open item):** Phase 3
  mapping quality depends on whether the scan source is a physical
  LiDAR or depth-camera-derived. If depth-derived, range is capped
  at ~4 m and the map will be noisier — lower the AMCL target to
  ≤ 10 cm RMSE in that case.
- **5-DOF arm constraint:** grasp pose space is restricted to
  vertical-approach poses. Phase 5 "IK-reachable" target assumes this
  constraint is encoded in the grasp planner; without it, the
  reachability rate will look much worse than reality.

## What these metrics are NOT

- Not a contract with a customer. They are engineering targets to
  detect regression and gate phase transitions.
- Not final. Expect to revise after the first 10 real trials — the
  first pass of real numbers always surfaces unrealistic targets.
- Not exhaustive. Safety-critical checks (e-stop latency, power draw)
  are out of scope for this project.
