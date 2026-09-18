#!/usr/bin/env python
"""Approach one target from four directions; measure direction-dependent offset."""
import subprocess, time, numpy as np, cv2
from pupil_apriltags import Detector
CORNERS=[(1182,45),(74,52),(76,651),(1185,656)]
PITCH,J6,Z=0.349,142,0.274
REMOTE=("export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; "
        "source ~/yahboomcar_ws/install/setup.bash; python3 ~/calib/move_to.py")
det=Detector(families="tag36h11",nthreads=4,quad_decimate=1.0,refine_edges=1)
def move(x,y,t=1600):
    return "SENT" in subprocess.run(["ssh","robot",f"{REMOTE} {x:.4f} {y:.4f} {Z} {PITCH} {J6} {t}"],
        capture_output=True,text=True,timeout=90).stdout
def observe(p):
    subprocess.run(["./capture.sh",p,"14"],capture_output=True,timeout=90)
    g=cv2.cvtColor(cv2.imread(p),cv2.COLOR_BGR2GRAY)
    for t in det.detect(g):
        if min(np.hypot(t.center[0]-cx,t.center[1]-cy) for cx,cy in CORNERS)>120:
            return np.array([t.center[0],t.center[1]])
    return None

TARGET=(0.24,0.0)
APPROACH={"from -y":(0.24,-0.07),"from +y":(0.24,+0.07),
          "from -x":(0.17,0.00),"from +x":(0.30,0.00)}
SCALE=1.547
obs={}
for rep in range(2):
    for name,(ax,ay) in APPROACH.items():
        move(ax,ay); time.sleep(1.8)
        move(*TARGET); time.sleep(2.0)
        o=observe(f"bl_{name.replace(' ','')}_{rep}.jpg")
        if o is not None:
            obs.setdefault(name,[]).append(o)
            print(f"rep{rep} {name:8s} -> px({o[0]:7.1f},{o[1]:6.1f})")
print()
means={k:np.mean(v,axis=0) for k,v in obs.items() if v}
allp=np.array(list(means.values()))
print("mean landing point per approach direction:")
for k,m in means.items():
    print(f"  {k:8s} px({m[0]:7.1f},{m[1]:6.1f})")
d=allp-allp.mean(axis=0)
print(f"\nspread across directions: {np.sqrt((d**2).sum(axis=1)).max()/SCALE:.2f} mm max, "
      f"{np.sqrt((d**2).sum(axis=1).mean())/SCALE:.2f} mm rms")
