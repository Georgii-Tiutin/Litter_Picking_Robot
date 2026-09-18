#!/usr/bin/env python
"""Fit commanded arm (x,y) <-> observed pixel (u,v); compare model families."""
import csv, numpy as np, json

rows=[r for r in csv.DictReader(open("grid_data2.csv")) if r["kind"]=="grid"]
X=np.array([[float(r["x_mm"]),float(r["y_mm"])] for r in rows])
P=np.array([[float(r["u_px"]),float(r["v_px"])] for r in rows])
n=len(X); print(f"{n} grid samples\n")

def design(XY, kind):
    x,y=XY[:,0],XY[:,1]; o=np.ones_like(x)
    if kind=="affine":    return np.column_stack([o,x,y])
    if kind=="quad":      return np.column_stack([o,x,y,x*x,x*y,y*y])
    if kind=="cubic":     return np.column_stack([o,x,y,x*x,x*y,y*y,x**3,x*x*y,x*y*y,y**3])

def fit(A,B,kind):
    M=design(A,kind); C,_,_,_=np.linalg.lstsq(M,B,rcond=None)
    R=B-M@C; return C, np.sqrt((R**2).sum(axis=1)), R

print(f"{'model':<8}{'params':>8}{'rms_px':>9}{'max_px':>9}{'rms_mm':>9}{'max_mm':>9}")
best=None
for kind in ("affine","quad","cubic"):
    C,res,_=fit(X,P,kind)
    # local scale: mean |d(px)/d(mm)| from the affine part
    Ca,_,_=fit(X,P,"affine")
    J=Ca[1:3]                       # 2x2 jacobian px per mm
    s=np.sqrt(abs(np.linalg.det(J)))
    print(f"{kind:<8}{design(X,kind).shape[1]*2:>8}{res.mean():>9.2f}{res.max():>9.2f}"
          f"{res.mean()/s:>9.2f}{res.max()/s:>9.2f}")
    if best is None or res.mean()<best[1]: best=(kind,res.mean(),C)

# inverse map: pixel -> commanded mm (what the drawing loop needs)
print()
for kind in ("affine","quad","cubic"):
    C,res,_=fit(P,X,kind)
    print(f"inverse {kind:<6} rms {res.mean():6.2f} mm   max {res.max():6.2f} mm")

kind,_,C=best
Cinv,resinv,_=fit(P,X,"quad")
json.dump({"forward_kind":kind,"forward_coeffs":C.tolist(),
           "inverse_kind":"quad","inverse_coeffs":Cinv.tolist(),
           "n_samples":n,"pitch_rad":0.349,"z_m":0.274,"joint6":142},
          open("arm_camera_model.json","w"),indent=2)
print(f"\nsaved arm_camera_model.json (forward={kind}, inverse=quad)")
