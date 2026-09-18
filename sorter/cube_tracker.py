#!/usr/bin/env python3
"""Multi-object data association for cube detections.

WHY THIS EXISTS
---------------
The first version merged a detection into any existing cube within 30 cm. That rule cannot
distinguish "the same cube, re-measured from a new angle" from "a second cube standing 15 cm
from the first" - and on 2026-09-13 it produced 21 candidates for 7 physical cuboids.

Merging greedily also breaks a constraint the camera hands us for free: if YOLO reports TWO
boxes in ONE image, those are definitively two different physical objects. A greedy nearest
merge can fold both into the same track and silently destroy that information.

So: gate by distance, then solve a ONE-TO-ONE assignment between this frame's detections and
the existing tracks with the Hungarian algorithm. Distance becomes a gate, not a merge rule.

No numpy or scipy: the matrices are tiny (tens of entries) and a dependency-free module can be
unit-tested on any machine, including when the robot is unreachable.
"""
import math, itertools

GATE_M        = 0.30    # a detection may only match a track within this distance
OUTLIER_K     = 2.5     # reject a detection further than this many typical residuals from a
                        # settled track, so one bad depth reading cannot drag a good estimate.
OUTLIER_MIN_N = 4       # ...only once a track has enough observations to have an opinion.
OUTLIER_FLOOR = 0.10    # m; never reject inside this, or noise alone would freeze a track.
#
# HONEST NOTE ON THESE NUMBERS. With FLOOR 0.10 and K 2.5 the rejection threshold is 0.25 m,
# only just inside the 0.30 m gate - so this mechanism is very nearly dormant today. That is
# deliberate and it is a statement about the CURRENT measurement accuracy, not a safe default:
# repeat observations of one stationary cube are measured 15-25 cm apart, so a 25 cm jump is
# not yet distinguishable from ordinary noise. Rejecting harder would throw away legitimate
# observations. Once the spread is reduced (capture-time TF, camera extrinsics, depth
# clustering), tighten K - the mechanism will then earn its place. Tuning it aggressively now
# would be using the tracker to hide a localisation error rather than to resolve ambiguity.
CONFIRM_OBS   = 3       # sightings needed before a tentative track becomes confirmed
CONFIRM_VIEWS = 2       # ...from at least this many distinct robot positions
DROP_TENTATIVE_AFTER = 90.0   # seconds; a tentative track never seen again is discarded
VIEW_SEPARATION = 0.25  # robot positions this far apart count as different viewpoints
COOBS_MERGE_M   = 0.22  # two nearby tracks may be one cube that YOLO split into two boxes
COOBS_TOLERANCE = 1     # ...but only if they were seen TOGETHER at most this many times.
                        # A duplicated box co-occurs exactly once and then never again; two
                        # real cubes co-occur in nearly every frame that sees either of them.
COOBS_MIN_N     = 3     # and only once one of them has enough observations to judge

INF = float("inf")


def hungarian(cost):
    """Minimum-cost one-to-one assignment for a rectangular cost matrix.

    cost[i][j] is the cost of assigning row i to column j; use INF to forbid a pair.
    Returns a list of (row, col) pairs. Rows or columns may be left unassigned when the
    matrix is not square or when every remaining option is forbidden.

    Implementation is the O(n^3) shortest-augmenting-path (Jonker-Volgenant style) form,
    padded to square. n is small here, so clarity beats cleverness.
    """
    if not cost or not cost[0]: return []
    n_rows, n_cols = len(cost), len(cost[0])
    n = max(n_rows, n_cols)
    BIG = 1e9
    # pad to square; padding entries are expensive but finite so a solution always exists
    a = [[(cost[i][j] if i < n_rows and j < n_cols and cost[i][j] != INF else BIG)
          for j in range(n)] for i in range(n)]

    u = [0.0]*(n+1); v = [0.0]*(n+1); p = [0]*(n+1); way = [0]*(n+1)
    for i in range(1, n+1):
        p[0] = i; j0 = 0
        minv = [INF]*(n+1); used = [False]*(n+1)
        while True:
            used[j0] = True
            i0 = p[j0]; delta = INF; j1 = 0
            for j in range(1, n+1):
                if used[j]: continue
                cur = a[i0-1][j-1] - u[i0] - v[j]
                if cur < minv[j]: minv[j] = cur; way[j] = j0
                if minv[j] < delta: delta = minv[j]; j1 = j
            for j in range(n+1):
                if used[j]: u[p[j]] += delta; v[j] -= delta
                else: minv[j] -= delta
            j0 = j1
            if p[j0] == 0: break
        while True:
            j1 = way[j0]; p[j0] = p[j1]; j0 = j1
            if j0 == 0: break

    out = []
    for j in range(1, n+1):
        i = p[j]
        if 1 <= i <= n_rows and 1 <= j <= n_cols:
            if cost[i-1][j-1] != INF and cost[i-1][j-1] < BIG:
                out.append((i-1, j-1))
    return sorted(out)


class Detection:
    """One YOLO box, carrying everything needed to reason about it later.

    n_px and depth_spread describe how trustworthy the DEPTH was: a box backed by hundreds
    of consistent pixels deserves more weight than one backed by nine scattered ones.
    """
    def __init__(self, x, y, z=0.03, conf=0.0, rng=1.0, stamp=0.0, bbox=None, cls="cube",
                 n_px=0, depth_spread=0.0):
        self.x, self.y, self.z = x, y, z
        self.conf, self.rng, self.stamp, self.bbox, self.cls = conf, rng, stamp, bbox, cls
        self.n_px, self.depth_spread = n_px, depth_spread

    def weight(self):
        """Higher = more trustworthy. Combines the factors we can actually measure."""
        w = 1.0/max(0.25, self.rng*self.rng)          # near views localise far better
        if self.n_px:                                  # more supporting pixels = steadier
            w *= min(2.0, 0.5 + self.n_px/200.0)
        if self.depth_spread > 0:                      # a noisy depth patch is less certain
            w *= 1.0/(1.0 + self.depth_spread/0.02)
        w *= 0.5 + min(1.0, self.conf)                 # and YOLO's own confidence
        return max(1e-6, w)

    def __repr__(self):
        return "Det(%.3f,%.3f c%.2f r%.2f w%.2f)"%(self.x,self.y,self.conf,self.rng,self.weight())


class CubeTrack:
    """A persistent belief about one physical cube."""
    _next_id = itertools.count(1)
    def __init__(self, det, robot_xy):
        self.id = next(CubeTrack._next_id)
        self.x, self.y, self.z = det.x, det.y, det.z
        self.conf = det.conf
        self.n = 1
        self.w = det.weight()
        self.resid = []                            # recent |detection - estimate|, for outliers
        self.last_seen = det.stamp
        self.views = [robot_xy] if robot_xy else []
        self.state = "tentative"
        self.spread = 0.0                          # running max disagreement, diagnostic
        self.frames = set()                        # frame ids this track was seen in

    def typical_resid(self):
        if len(self.resid) < 2: return None
        m = sorted(self.resid)[len(self.resid)//2]
        return max(OUTLIER_FLOOR, m)

    def is_outlier(self, det):
        """Would this detection be an implausible jump for a track that has settled down?"""
        if self.n < OUTLIER_MIN_N: return False
        tr = self.typical_resid()
        if tr is None: return False
        return math.hypot(det.x-self.x, det.y-self.y) > OUTLIER_K*tr

    def update(self, det, robot_xy, frame_id=None):
        if frame_id is not None: self.frames.add(frame_id)
        d = math.hypot(det.x-self.x, det.y-self.y)
        self.spread = max(self.spread, d)
        self.resid.append(d); self.resid = self.resid[-12:]
        wt = det.weight()
        tot = self.w + wt; f = wt/tot
        self.x += (det.x-self.x)*f
        self.y += (det.y-self.y)*f
        self.z += (det.z-self.z)*f
        self.w = tot
        self.n += 1
        self.conf = max(self.conf, det.conf)
        self.last_seen = det.stamp
        if robot_xy and not any(math.hypot(robot_xy[0]-v[0], robot_xy[1]-v[1]) < VIEW_SEPARATION
                                for v in self.views):
            self.views.append(robot_xy)
        if self.state == "tentative" and self.n >= CONFIRM_OBS and len(self.views) >= CONFIRM_VIEWS:
            self.state = "confirmed"

    def __repr__(self):
        return "Track#%d(%.3f,%.3f n=%d views=%d %s)"%(
            self.id, self.x, self.y, self.n, len(self.views), self.state)


class CubeTracker:
    def __init__(self, gate=GATE_M):
        self.gate = gate
        self.tracks = []
        self.frame_no = 0
        self.rejected = 0

    def _merge_never_coobserved(self):
        """Undo splits caused by YOLO emitting two boxes for one cube.

        The one-to-one constraint is correct in general - two boxes in one image really are
        two objects - but it means a single duplicated box becomes two PERMANENT tracks. On
        2026-09-15 that produced tracks 0.149 m apart, well inside the 0.30 m gate, which
        association could never afterwards reconcile.

        The discriminator is how OFTEN two tracks appear together. A duplicated box co-occurs
        in exactly one frame and afterwards only one of the pair is ever seen; two real cubes
        co-occur in nearly every frame that sees either. So merge only when the pair is close,
        was seen together at most COOBS_TOLERANCE times, and one of them has enough
        observations to be judged. Repeatedly co-observed tracks are never merged however
        close they are - that is exactly the "two real cubes 15 cm apart" case this design
        exists to protect.
        """
        merged = True
        while merged:
            merged = False
            for i in range(len(self.tracks)):
                for j in range(i+1, len(self.tracks)):
                    a, b = self.tracks[i], self.tracks[j]
                    # How often were these two seen SIMULTANEOUSLY? Repeatedly => two real
                    # objects, leave them alone. Once => almost certainly a duplicated box.
                    if len(a.frames & b.frames) > COOBS_TOLERANCE:
                        continue
                    if max(a.n, b.n) < COOBS_MIN_N:
                        continue                     # too early to tell them apart
                    if math.hypot(a.x-b.x, a.y-b.y) > COOBS_MERGE_M:
                        continue
                    tot = a.w + b.w; f = b.w/tot
                    a.x += (b.x-a.x)*f; a.y += (b.y-a.y)*f; a.z += (b.z-a.z)*f
                    a.w = tot; a.n += b.n
                    a.conf = max(a.conf, b.conf)
                    a.frames |= b.frames
                    for v in b.views:
                        if not any(math.hypot(v[0]-u[0], v[1]-u[1]) < VIEW_SEPARATION
                                   for u in a.views):
                            a.views.append(v)
                    a.last_seen = max(a.last_seen, b.last_seen)
                    if a.n >= CONFIRM_OBS and len(a.views) >= CONFIRM_VIEWS:
                        a.state = "confirmed"
                    self.tracks.pop(j); merged = True; break
                if merged: break

    def update(self, detections, robot_xy=None, stamp=0.0):
        """Associate a WHOLE FRAME of detections against all tracks at once.

        Doing the frame as a set is the point: two boxes in one image are two different
        physical cubes, so they must not both be assigned to the same track.
        """
        self.frame_no += 1
        fid = self.frame_no
        for d in detections:
            if not d.stamp: d.stamp = stamp
        if not self.tracks:
            for d in detections:
                t = CubeTrack(d, robot_xy); t.frames.add(fid); self.tracks.append(t)
            return self._expire(stamp)

        cost = [[(math.hypot(d.x-t.x, d.y-t.y)
                  if math.hypot(d.x-t.x, d.y-t.y) <= self.gate else INF)
                 for t in self.tracks] for d in detections]
        pairs = hungarian(cost) if detections else []
        matched_d = set(i for i, _ in pairs)
        self.rejected = 0
        for di, ti in pairs:
            t = self.tracks[ti]
            if t.is_outlier(detections[di]):
                # Matched, but an implausible jump for a settled track. Do NOT let it move
                # the estimate, and do NOT spawn a new track either - that would turn one
                # bad depth reading into a permanent phantom cube.
                self.rejected += 1
                matched_d.add(di)
                continue
            t.update(detections[di], robot_xy, frame_id=fid)
        for i, d in enumerate(detections):
            if i not in matched_d:
                t = CubeTrack(d, robot_xy); t.frames.add(fid); self.tracks.append(t)
        self._merge_never_coobserved()
        return self._expire(stamp)

    def _expire(self, now):
        keep = []
        for t in self.tracks:
            if t.state == "tentative" and now - t.last_seen > DROP_TENTATIVE_AFTER and t.n < CONFIRM_OBS:
                continue
            keep.append(t)
        self.tracks = keep
        return self.tracks

    def confirmed(self):
        return [t for t in self.tracks if t.state == "confirmed"]
