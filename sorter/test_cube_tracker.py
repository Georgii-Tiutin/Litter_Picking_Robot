#!/usr/bin/env python3
"""Tests for cube_tracker. Runs anywhere - no numpy, no scipy, no robot."""
import sys, math, random, itertools
sys.path.insert(0, "/Users/georgiitiutin/ProjectsRoot/Robot_AI/sorter")
from cube_tracker import hungarian, INF, Detection, CubeTracker, CubeTrack

fails = []
def check(name, cond, detail=""):
    print("   %-58s %s"%(name, "PASS" if cond else "FAIL  "+detail))
    if not cond: fails.append(name)

def brute(cost):
    """Exhaustive optimal assignment, for verifying the Hungarian on small matrices."""
    R, C = len(cost), len(cost[0])
    best, bestsel = INF, None
    for k in range(min(R, C), -1, -1):
        for rows in itertools.combinations(range(R), k):
            for cols in itertools.permutations(range(C), k):
                tot = 0.0; ok = True
                for r, c in zip(rows, cols):
                    if cost[r][c] == INF: ok = False; break
                    tot += cost[r][c]
                if not ok: continue
                # prefer more assignments, then lower cost
                key = (-k, tot)
                if bestsel is None or key < bestsel[0]:
                    bestsel = (key, list(zip(rows, cols))); best = tot
        if bestsel: break
    return bestsel[1] if bestsel else []

print("\n--- Hungarian correctness vs brute force ---")
random.seed(7)
worst = 0.0; bad = 0
for trial in range(300):
    R = random.randint(1, 5); C = random.randint(1, 5)
    cost = [[(INF if random.random() < 0.15 else round(random.uniform(0, 1), 3))
             for _ in range(C)] for _ in range(R)]
    h = hungarian(cost)
    hb = brute(cost)
    ch = sum(cost[r][c] for r, c in h)
    cb = sum(cost[r][c] for r, c in hb)
    if len(h) != len(hb) or abs(ch - cb) > 1e-6:
        bad += 1
        if bad <= 2: print("      mismatch: cost=%s\n        hung=%s (%.3f)\n        brute=%s (%.3f)"%(cost,h,ch,hb,cb))
    worst = max(worst, abs(ch - cb))
check("300 random matrices match brute-force optimum", bad == 0, "%d mismatched"%bad)

print("\n--- one-to-one constraint ---")
# two detections, ONE track: only one may match
cost = [[0.05], [0.07]]
h = hungarian(cost)
check("2 detections cannot both claim 1 track", len(h) == 1, str(h))

print("\n--- the scenario from the proposal ---")
# Det1: 7cm from A, 18cm from B.  Det2: 21cm from A, 5cm from B.
cost = [[0.07, 0.18], [0.21, 0.05]]
h = dict(hungarian(cost))
check("Det1->A and Det2->B", h.get(0) == 0 and h.get(1) == 1, str(h))
# greedy-by-smallest would also get this one; now a case where greedy FAILS:
# Det1 is nearest to A, but assigning it there strands Det2 with no legal match.
cost = [[0.10, 0.12], [0.11, INF]]
h = dict(hungarian(cost))
check("greedy would strand a detection; Hungarian does not",
      h.get(0) == 1 and h.get(1) == 0, str(h))

print("\n--- two real cubes 15 cm apart, seen from 4 directions with 8 cm noise ---")
CubeTrack._next_id = itertools.count(1)
random.seed(3)
A, B = (0.0, 0.0), (0.15, 0.0)
tr = CubeTracker()
for view in range(4):
    rx, ry = math.cos(view*1.6)*0.6, math.sin(view*1.6)*0.6
    bias = (random.uniform(-0.08, 0.08), random.uniform(-0.08, 0.08))  # per-view pose error
    dets = [Detection(A[0]+bias[0], A[1]+bias[1], conf=0.9, rng=0.6, stamp=view),
            Detection(B[0]+bias[0], B[1]+bias[1], conf=0.9, rng=0.6, stamp=view)]
    tr.update(dets, robot_xy=(rx, ry), stamp=view)
check("still exactly 2 tracks", len(tr.tracks) == 2, "%d tracks: %s"%(len(tr.tracks), tr.tracks))
check("both confirmed", len(tr.confirmed()) == 2, str(tr.confirmed()))
if len(tr.tracks) == 2:
    sep = math.hypot(tr.tracks[0].x-tr.tracks[1].x, tr.tracks[0].y-tr.tracks[1].y)
    check("separation recovered near 0.15 m (got %.3f)"%sep, 0.10 < sep < 0.20)

print("\n--- ONE cube, 20 cm of viewpoint noise (the old merge radius would split it) ---")
CubeTrack._next_id = itertools.count(1)
random.seed(11)
tr = CubeTracker()
for view in range(6):
    rx, ry = math.cos(view*1.1)*0.7, math.sin(view*1.1)*0.7
    d = Detection(random.uniform(-0.10, 0.10), random.uniform(-0.10, 0.10),
                  conf=0.9, rng=0.7, stamp=view)
    tr.update([d], robot_xy=(rx, ry), stamp=view)
check("stays a single track", len(tr.tracks) == 1, "%d tracks"%len(tr.tracks))
check("confirmed after repeat views", len(tr.confirmed()) == 1, str(tr.tracks))

print("\n--- a one-off false positive ---")
CubeTrack._next_id = itertools.count(1)
tr = CubeTracker()
tr.update([Detection(2.0, 2.0, conf=0.3, rng=1.5, stamp=0.0)], robot_xy=(0, 0), stamp=0.0)
check("created only as tentative", tr.tracks[0].state == "tentative")
check("not reported as confirmed", len(tr.confirmed()) == 0)
tr.update([], robot_xy=(0, 0), stamp=200.0)
check("expires when never seen again", len(tr.tracks) == 0, str(tr.tracks))

print("\n--- gating: a detection far from everything starts its own track ---")
CubeTrack._next_id = itertools.count(1)
tr = CubeTracker()
tr.update([Detection(0, 0, rng=0.5, stamp=0)], robot_xy=(0, 0), stamp=0)
tr.update([Detection(1.5, 1.5, rng=0.5, stamp=1)], robot_xy=(0, 0), stamp=1)
check("2 separate tracks beyond the gate", len(tr.tracks) == 2, str(tr.tracks))

print("\n--- outlier rejection (the GPT scenario: one bad depth reading) ---")
CubeTrack._next_id = itertools.count(1)
tr = CubeTracker()
# settle a track with consistent observations around (2.10, 1.30)
for k,(x,y) in enumerate([(2.10,1.30),(2.12,1.31),(2.09,1.28),(2.11,1.32),(2.10,1.29)]):
    tr.update([Detection(x,y,conf=0.9,rng=0.6,stamp=k,n_px=300,depth_spread=0.005)],
              robot_xy=(k*0.3,0.0), stamp=k)
before = (tr.tracks[0].x, tr.tracks[0].y)
check("settled to ~(2.10,1.30)", abs(before[0]-2.10)<0.03 and abs(before[1]-1.30)<0.03,
      str(before))
# The reading from the discussion, (2.42,1.51), is 0.37 m away - OUTSIDE the 0.30 m gate.
# Gating alone rejects it, and it becomes a tentative track that expires unless corroborated.
tr.update([Detection(2.42,1.51,conf=0.9,rng=0.6,stamp=9,n_px=12,depth_spread=0.09)],
          robot_xy=(2.0,0.0), stamp=9)
after = (tr.tracks[0].x, tr.tracks[0].y)
moved = math.hypot(after[0]-before[0], after[1]-before[1])
check("far outlier did not drag the estimate (moved %.3f m)"%moved, moved < 0.02, str(after))
check("far outlier is only tentative, never confirmed",
      len(tr.confirmed()) == 1 and tr.tracks[-1].state == "tentative", str(tr.tracks))

# Now the case outlier rejection actually exists for: INSIDE the gate, but a jump far larger
# than this settled track's own scatter (residuals ~0.02 m, so 0.25 m is implausible).
# Isolate the in-gate outlier case on a FRESH tracker: the previous block left a tentative
# track at (2.42,1.51), and a probe at (2.38,1.30) is nearer to THAT than to the settled one,
# so Hungarian quite correctly assigned it there instead. One track, one probe, no ambiguity.
CubeTrack._next_id = itertools.count(1)
tr2 = CubeTracker()
for k,(x,y) in enumerate([(2.10,1.30),(2.12,1.31),(2.09,1.28),(2.11,1.32),(2.10,1.29)]):
    tr2.update([Detection(x,y,conf=0.9,rng=0.6,stamp=k,n_px=300,depth_spread=0.005)],
               robot_xy=(k*0.3,0.0), stamp=k)
# threshold is OUTLIER_K * max(FLOOR, typical residual) = 2.5 * 0.10 = 0.25 m, gate is 0.30 m,
# so only a jump between those two bounds exercises this path at all.
inside = Detection(2.10+0.28, 1.30, conf=0.9, rng=0.6, stamp=10, n_px=15, depth_spread=0.08)
check("flagged as an outlier by the track itself", tr2.tracks[0].is_outlier(inside))
b2 = (tr2.tracks[0].x, tr2.tracks[0].y)
tr2.update([inside], robot_xy=(2.1,0.0), stamp=10)
a2 = (tr2.tracks[0].x, tr2.tracks[0].y)
m2 = math.hypot(a2[0]-b2[0], a2[1]-b2[1])
check("in-gate outlier did not move the estimate (%.4f m)"%m2, m2 < 0.001, str(a2))
check("and did not become a new phantom track", len(tr2.tracks) == 1, str(tr2.tracks))
check("rejection counted", getattr(tr2,"rejected",0) == 1, str(getattr(tr2,"rejected",None)))

print("\n--- depth quality changes the weight ---")
good = Detection(0,0,conf=0.9,rng=0.5,n_px=400,depth_spread=0.003)
poor = Detection(0,0,conf=0.3,rng=1.6,n_px=11,depth_spread=0.12)
check("a close, dense, confident detection outweighs a far, sparse, noisy one (%.2f vs %.2f)"
      %(good.weight(),poor.weight()), good.weight() > 5*poor.weight())

print("\n--- YOLO splits ONE cube into two boxes in one frame ---")
CubeTrack._next_id = itertools.count(1)
tr = CubeTracker()
# frame 1: a duplicated box for a single cube, 0.15 m apart -> forced into two tracks
tr.update([Detection(0.00,0.00,conf=0.9,rng=0.6,stamp=0),
           Detection(0.15,0.00,conf=0.8,rng=0.6,stamp=0)], robot_xy=(0,0), stamp=0)
check("one-to-one forces 2 tracks initially", len(tr.tracks)==2, str(tr.tracks))
# later frames see only ONE box each - never both together
for k in range(1,5):
    tr.update([Detection(0.07,0.00,conf=0.9,rng=0.6,stamp=k)],
              robot_xy=(k*0.3,0.0), stamp=k)
check("never-co-observed pair gets merged back to 1", len(tr.tracks)==1, str(tr.tracks))

print("\n--- but two REAL cubes seen together are never merged ---")
CubeTrack._next_id = itertools.count(1)
tr = CubeTracker()
for k in range(5):
    # both cubes visible in the SAME frame every time - that is the evidence of two objects
    tr.update([Detection(0.00,0.00,conf=0.9,rng=0.6,stamp=k),
               Detection(0.15,0.00,conf=0.9,rng=0.6,stamp=k)],
              robot_xy=(k*0.3,0.0), stamp=k)
check("co-observed cubes 0.15 m apart stay separate", len(tr.tracks)==2, str(tr.tracks))
check("both confirmed", len(tr.confirmed())==2, str(tr.confirmed()))

print("\n%s"%("ALL TESTS PASSED" if not fails else "FAILURES: %s"%fails))
sys.exit(1 if fails else 0)
