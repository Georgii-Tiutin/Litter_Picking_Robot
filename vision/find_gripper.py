#!/usr/bin/env python
"""Locate the gripper in the overhead view by differencing against a parked reference.
The gripper is the extremity of the changed region, i.e. the changed pixel farthest
from the arm's shoulder."""
import sys, numpy as np, cv2
SHOULDER = (575, 372)          # arm turret centre in overhead pixels

def gripper_px(ref_path, cur_path, dbg=None):
    ref = cv2.imread(ref_path).astype(np.int16)
    cur = cv2.imread(cur_path).astype(np.int16)
    d = np.abs(cur - ref).sum(axis=2).astype(np.uint8)
    _, m = cv2.threshold(d, 45, 255, cv2.THRESH_BINARY)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9,9), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1: return None, 0
    # keep components of reasonable size, take the one reaching farthest from the shoulder
    best, bestd = None, -1
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 300: continue
        ys, xs = np.where(lab == i)
        dist = np.hypot(xs - SHOULDER[0], ys - SHOULDER[1])
        j = int(np.argmax(dist))
        if dist[j] > bestd: bestd, best = dist[j], (float(xs[j]), float(ys[j]), i)
    if best is None: return None, 0
    x, y, comp = best
    # refine: mean of the 40 farthest pixels of that component (tip of the claw)
    ys, xs = np.where(lab == comp)
    dist = np.hypot(xs - SHOULDER[0], ys - SHOULDER[1])
    k = np.argsort(dist)[-40:]
    tip = (float(xs[k].mean()), float(ys[k].mean()))
    if dbg:
        v = cv2.imread(cur_path); cv2.circle(v, (int(tip[0]), int(tip[1])), 9, (0,0,255), 2)
        cv2.circle(v, SHOULDER, 6, (255,0,0), -1); cv2.imwrite(dbg, v)
    return tip, int(stats[comp, cv2.CC_STAT_AREA])

if __name__ == "__main__":
    t, a = gripper_px(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv)>3 else None)
    print(f"gripper px {t}  area {a}" if t else "gripper NOT found")
