#!/usr/bin/env python
"""Dense grid, 2 captures averaged per pose."""
import subprocess, time, csv, numpy as np, cv2
from pupil_apriltags import Detector
CORNERS=[(1182,45),(74,52),(76,651),(1185,656)]
PITCH,J6,Z=0.349,142,0.274
REMOTE=("export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; "
        "source ~/yahboomcar_ws/install/setup.bash; python3 ~/calib/move_to.py")
det=Detector(families="tag36h11",nthreads=4,quad_decimate=1.0,refine_edges=1)

def move(x,y):
    r=subprocess.run(["ssh","robot",f"{REMOTE} {x:.4f} {y:.4f} {Z} {PITCH} {J6} 1800"],
                     capture_output=True,text=True,timeout=90)
    return "SENT" in r.stdout

def observe(path):
    subprocess.run(["./capture.sh",path,"14"],capture_output=True,timeout=90)
    g=cv2.cvtColor(cv2.imread(path),cv2.COLOR_BGR2GRAY)
    for t in det.detect(g):
        if min(np.hypot(t.center[0]-cx,t.center[1]-cy) for cx,cy in CORNERS)>120:
            c=t.corners; s=[float(np.linalg.norm(c[i]-c[(i+1)%4])) for i in range(4)]
            return t.center[0],t.center[1],min(s)/max(s)
    return None

rows=[];n=0;t0=time.time()
xs=[0.16,0.18,0.20,0.22,0.24,0.26,0.28,0.30]
ys=[-0.09,-0.06,-0.03,0.0,0.03,0.06,0.09]
for x in xs:
    for y in ys:
        n+=1
        if not move(x,y):
            print(f"[{n:3}] ({x:.2f},{y:+.2f}) unreachable"); continue
        time.sleep(2.0)
        obs=[observe(f"g3_{n:03d}_{k}.jpg") for k in (0,1)]
        obs=[o for o in obs if o]
        if not obs:
            print(f"[{n:3}] ({x:.2f},{y:+.2f}) not seen"); continue
        u=float(np.mean([o[0] for o in obs])); v=float(np.mean([o[1] for o in obs]))
        sq=float(np.mean([o[2] for o in obs]))
        spread=float(np.hypot(obs[0][0]-obs[-1][0],obs[0][1]-obs[-1][1])) if len(obs)>1 else 0.0
        rows.append(["grid",x*1000,y*1000,round(u,2),round(v,2),round(sq,3),round(spread,2)])
        print(f"[{n:3}] ({x:.2f},{y:+.2f}) -> px({u:7.1f},{v:6.1f}) sq {sq:.2f} spread {spread:.1f}px")
    # repeatability probe each row
    if move(0.22,0.0):
        time.sleep(2.0); o=observe(f"g3_rep_{len(rows)}.jpg")
        if o: rows.append(["repeat",220.0,0.0,round(o[0],2),round(o[1],2),round(o[2],3),0.0])

with open("grid_data3.csv","w",newline="") as f:
    w=csv.writer(f); w.writerow(["kind","x_mm","y_mm","u_px","v_px","square","spread_px"]); w.writerows(rows)
g=[r for r in rows if r[0]=="grid"]
print(f"\n{len(g)} grid points in {(time.time()-t0)/60:.1f} min -> grid_data3.csv")
