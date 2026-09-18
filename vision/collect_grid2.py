#!/usr/bin/env python
"""Grid collection v2: proper settling + interleaved repeatability probes."""
import subprocess, time, csv, numpy as np, cv2
from pupil_apriltags import Detector

CORNERS=[(1182,45),(74,52),(76,651),(1185,656)]
PITCH, J6, Z = 0.349, 142, 0.274
MOVE_MS, SETTLE = 2000, 2.5
REMOTE=("export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; "
        "source ~/yahboomcar_ws/install/setup.bash; python3 ~/calib/move_to.py")
det=Detector(families="tag36h11",nthreads=4,quad_decimate=1.0,refine_edges=1)

def move(x,y):
    r=subprocess.run(["ssh","robot",f"{REMOTE} {x:.4f} {y:.4f} {Z} {PITCH} {J6} {MOVE_MS}"],
                     capture_output=True,text=True,timeout=90)
    return "SENT" in r.stdout

def observe(path):
    subprocess.run(["./capture.sh",path],capture_output=True,timeout=90)
    g=cv2.cvtColor(cv2.imread(path),cv2.COLOR_BGR2GRAY)
    for t in det.detect(g):
        if min(np.hypot(t.center[0]-cx,t.center[1]-cy) for cx,cy in CORNERS)>120:
            c=t.corners; s=[float(np.linalg.norm(c[i]-c[(i+1)%4])) for i in range(4)]
            return t.center[0],t.center[1],min(s)/max(s),t.decision_margin
    return None

def sample(x,y,tag,rows,n):
    if not move(x,y):
        print(f"  {tag} ({x:.2f},{y:+.2f}) unreachable"); return
    time.sleep(SETTLE)
    o=observe(f"g2_{n:03d}.jpg")
    if o is None:
        print(f"  {tag} ({x:.2f},{y:+.2f}) tag not seen"); return
    u,v,sq,mg=o
    print(f"  {tag} ({x:.2f},{y:+.2f}) -> px({u:7.1f},{v:6.1f}) sq {sq:.2f}")
    rows.append([tag,x*1000,y*1000,round(u,2),round(v,2),round(sq,3),round(mg,1)])

rows=[]; n=0
REPEAT=(0.22,0.0)
print("grid:")
for x in [0.16,0.20,0.22,0.25,0.28]:
    for y in [-0.08,-0.04,0.0,0.04,0.08]:
        n+=1; sample(x,y,"grid",rows,n)
    n+=1; sample(*REPEAT,"repeat",rows,n)      # probe after each row

with open("grid_data2.csv","w",newline="") as f:
    w=csv.writer(f); w.writerow(["kind","x_mm","y_mm","u_px","v_px","square","margin"]); w.writerows(rows)

rep=np.array([[r[3],r[4]] for r in rows if r[0]=="repeat"])
if len(rep)>1:
    d=rep-rep.mean(axis=0)
    print(f"\nrepeatability over {len(rep)} visits to {REPEAT}:")
    print(f"  spread u {np.ptp(rep[:,0]):.1f} px, v {np.ptp(rep[:,1]):.1f} px")
    print(f"  rms {np.sqrt((d**2).sum(axis=1).mean()):.2f} px")
print(f"\n{len([r for r in rows if r[0]=='grid'])} grid points -> grid_data2.csv")
