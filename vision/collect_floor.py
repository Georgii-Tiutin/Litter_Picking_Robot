#!/usr/bin/env python
"""Floor-level calibration: pixel <-> arm(x,y) at grasp height, via jaw-toggle detection."""
import csv, time, numpy as np
from jaw_locate import locate
Z, PITCH = 0.06, 1.5708
rows=[]
for x in [0.16,0.19,0.22,0.25,0.28]:
    for y in [-0.09,-0.06,-0.03,0.0,0.03,0.06,0.09]:
        r=locate(x,y,Z,PITCH,f"f{int(x*100)}_{int(y*100)}")
        if r is None:
            print(f"({x:.2f},{y:+.2f}) unreachable/not found"); continue
        u,v,area,md=r
        if area<120:
            print(f"({x:.2f},{y:+.2f}) weak detection area={area}, skipped"); continue
        rows.append([x*1000,y*1000,round(u,2),round(v,2),area])
        print(f"({x:.2f},{y:+.2f}) -> px({u:7.1f},{v:6.1f}) area {area}")
with open("floor_grid06.csv","w",newline="") as f:
    w=csv.writer(f); w.writerow(["x_mm","y_mm","u_px","v_px","area"]); w.writerows(rows)
print(f"\n{len(rows)} usable samples -> floor_grid06.csv")
