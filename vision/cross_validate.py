#!/usr/bin/env python
"""Leave-one-out CV: which model actually generalises?"""
import csv, numpy as np

rows=[r for r in csv.DictReader(open("grid_data2.csv")) if r["kind"]=="grid"]
X=np.array([[float(r["x_mm"]),float(r["y_mm"])] for r in rows])
P=np.array([[float(r["u_px"]),float(r["v_px"])] for r in rows])
rep=np.array([[float(r["u_px"]),float(r["v_px"])] for r in csv.DictReader(open("grid_data2.csv")) if r["kind"]=="repeat"])

def design(XY,kind):
    x,y=XY[:,0],XY[:,1]; o=np.ones_like(x)
    return {"affine":np.column_stack([o,x,y]),
            "quad":  np.column_stack([o,x,y,x*x,x*y,y*y]),
            "cubic": np.column_stack([o,x,y,x*x,x*y,y*y,x**3,x*x*y,x*y*y,y**3])}[kind]

# inverse direction: pixel -> mm, which is what the drawing loop uses
A,B=P,X
print(f"leave-one-out CV, pixel -> mm  ({len(A)} samples)\n")
print(f"{'model':<8}{'k':>4}{'train_mm':>10}{'LOO_mm':>9}{'LOO_max':>10}")
for kind in ("affine","quad","cubic"):
    M=design(A,kind); k=M.shape[1]
    C,_,_,_=np.linalg.lstsq(M,B,rcond=None)
    train=np.sqrt((((B-M@C)**2).sum(axis=1))).mean()
    errs=[]
    for i in range(len(A)):
        m=np.ones(len(A),bool); m[i]=False
        Ci,_,_,_=np.linalg.lstsq(design(A[m],kind),B[m],rcond=None)
        pred=design(A[~m],kind)@Ci
        errs.append(float(np.sqrt(((B[~m]-pred)**2).sum())))
    errs=np.array(errs)
    print(f"{kind:<8}{k:>4}{train:>10.2f}{errs.mean():>9.2f}{errs.max():>10.2f}")

# noise floor expressed in mm using the affine jacobian
Ca,_,_,_=np.linalg.lstsq(design(X,"affine"),P,rcond=None)
s=np.sqrt(abs(np.linalg.det(Ca[1:3])))
d=rep-rep.mean(axis=0)
print(f"\nhardware noise floor (5 repeat visits): "
      f"{np.sqrt((d**2).sum(axis=1).mean())/s:.2f} mm rms")
