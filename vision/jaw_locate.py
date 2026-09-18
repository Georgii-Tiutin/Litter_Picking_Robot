#!/usr/bin/env python
"""Locate the gripper jaws by toggling j6 and differencing. Returns (u,v)."""
import subprocess, time, numpy as np, cv2
R=("export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; "
   "source ~/yahboomcar_ws/install/setup.bash;")
def _move(x,y,z,pitch,j6,t=1500):
    r=subprocess.run(["ssh","robot",f"{R} python3 ~/calib/move_to.py {x:.4f} {y:.4f} {z:.4f} {pitch} {j6} {t}"],
                     capture_output=True,text=True,timeout=90)
    return "SENT" in r.stdout
def _cap(p):
    subprocess.run(["./capture.sh",p,"12"],capture_output=True,timeout=90)
    return cv2.cvtColor(cv2.imread(p),cv2.COLOR_BGR2GRAY).astype(np.float32)
def locate(x,y,z,pitch,tag=""):
    """Move to pose, toggle jaws, return jaw pixel or None."""
    if not _move(x,y,z,pitch,30,1700): return None
    time.sleep(2.0); a=_cap(f"jl_{tag}_o.jpg")
    if not _move(x,y,z,pitch,142,1100): return None
    time.sleep(1.6); b=_cap(f"jl_{tag}_c.jpg")
    d=np.abs(a-b); m=(d>40).astype(np.uint8)
    m=cv2.morphologyEx(m,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    n,lab,st,cen=cv2.connectedComponentsWithStats(m,8)
    if n<=1: return None
    i=max(range(1,n),key=lambda k:st[k,cv2.CC_STAT_AREA])
    return float(cen[i][0]), float(cen[i][1]), int(st[i,cv2.CC_STAT_AREA]), float(d.mean())
if __name__=="__main__":
    import sys
    for (x,y) in [(0.16,0.0),(0.16,0.06),(0.16,-0.06),(0.18,0.0)]:
        r=locate(x,y,0.05,1.5708,f"{x}_{y}")
        print(f"({x:.2f},{y:+.2f}) -> {('px(%7.1f,%6.1f) area %4d  mean|d| %.1f'%r) if r else 'FAILED'}")
