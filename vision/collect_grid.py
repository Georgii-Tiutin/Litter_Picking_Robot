#!/usr/bin/env python
"""Drive the arm over a grid, observe the gripper tag overhead, log commanded vs observed.

Writes grid_data.csv:  x_mm,y_mm,z_mm,u_px,v_px,square,margin,status
"""
import subprocess, sys, time, csv, numpy as np, cv2
from pupil_apriltags import Detector

CORNERS = [(1182,45),(74,52),(76,651),(1185,656)]   # static corner tags
PITCH   = 0.349          # rad - keeps the tag square to the camera
J6      = 142            # hold the cube
Z       = 0.274          # m
REMOTE  = ("export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; "
           "source ~/yahboomcar_ws/install/setup.bash; python3 ~/calib/move_to.py")

det = Detector(families="tag36h11", nthreads=4, quad_decimate=1.0, refine_edges=1)

def move(x, y):
    cmd = f"{REMOTE} {x:.4f} {y:.4f} {Z:.4f} {PITCH} {J6} 1600"
    r = subprocess.run(["ssh","robot",cmd], capture_output=True, text=True, timeout=90)
    return ("SENT" in r.stdout), r.stdout.strip().splitlines()[-1] if r.stdout else "no output"

def observe(path):
    subprocess.run(["./capture.sh", path], capture_output=True, timeout=90)
    g = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
    best = None
    for t in det.detect(g):
        if min(np.hypot(t.center[0]-cx, t.center[1]-cy) for cx,cy in CORNERS) > 120:
            best = t
    if best is None:
        return None
    c = best.corners
    s = [float(np.linalg.norm(c[i]-c[(i+1)%4])) for i in range(4)]
    return best.center[0], best.center[1], min(s)/max(s), best.decision_margin

xs = [0.16, 0.19, 0.22, 0.25, 0.28]
ys = [-0.10, -0.05, 0.0, 0.05, 0.10]

rows, n, ok = [], 0, 0
for x in xs:
    for y in ys:
        n += 1
        sent, msg = move(x, y)
        if not sent:
            print(f"[{n:2}] ({x:.2f},{y:+.2f}) SKIP - {msg}")
            rows.append([x*1000, y*1000, Z*1000, "", "", "", "", "unreachable"])
            continue
        time.sleep(1.2)
        obs = observe(f"grid_{n:02d}.jpg")
        if obs is None:
            print(f"[{n:2}] ({x:.2f},{y:+.2f}) tag not seen")
            rows.append([x*1000, y*1000, Z*1000, "", "", "", "", "not_seen"])
            continue
        u, v, sq, mg = obs
        ok += 1
        print(f"[{n:2}] ({x:.2f},{y:+.2f}) -> px({u:7.1f},{v:6.1f})  sq {sq:.2f}  margin {mg:.0f}")
        rows.append([x*1000, y*1000, Z*1000, round(u,2), round(v,2), round(sq,3), round(mg,1), "ok"])

with open("grid_data.csv","w",newline="") as f:
    w = csv.writer(f); w.writerow(["x_mm","y_mm","z_mm","u_px","v_px","square","margin","status"])
    w.writerows(rows)
print(f"\n{ok}/{n} samples usable -> grid_data.csv")
