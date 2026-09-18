#!/usr/bin/env python
import csv, sys, numpy as np, json
f=sys.argv[1] if len(sys.argv)>1 else "grid_data3.csv"
R=list(csv.DictReader(open(f)))
G=[r for r in R if r["kind"]=="grid"]; Rep=[r for r in R if r["kind"]=="repeat"]
X=np.array([[float(r["x_mm"]),float(r["y_mm"])] for r in G])
P=np.array([[float(r["u_px"]),float(r["v_px"])] for r in G])
rep=np.array([[float(r["u_px"]),float(r["v_px"])] for r in Rep])
print(f"{len(G)} grid samples, {len(rep)} repeat probes\n")

def design(A,k):
    x,y=A[:,0],A[:,1]; o=np.ones_like(x)
    return {"affine":np.column_stack([o,x,y]),
            "quad":np.column_stack([o,x,y,x*x,x*y,y*y]),
            "cubic":np.column_stack([o,x,y,x*x,x*y,y*y,x**3,x*x*y,x*y*y,y**3])}[k]

A,B=P,X    # pixel -> mm, the direction the drawing loop needs
print(f"{'model':<8}{'k':>4}{'train':>9}{'LOO':>8}{'LOO_max':>9}   (mm)")
results={}
for k in ("affine","quad","cubic"):
    M=design(A,k); C,_,_,_=np.linalg.lstsq(M,B,rcond=None)
    tr=np.sqrt((((B-M@C)**2).sum(axis=1))).mean()
    e=[]
    for i in range(len(A)):
        m=np.ones(len(A),bool); m[i]=False
        Ci,_,_,_=np.linalg.lstsq(design(A[m],k),B[m],rcond=None)
        e.append(float(np.sqrt(((B[~m]-design(A[~m],k)@Ci)**2).sum())))
    e=np.array(e); results[k]=(e.mean(),C)
    print(f"{k:<8}{M.shape[1]:>4}{tr:>9.2f}{e.mean():>8.2f}{e.max():>9.2f}")

Ca,_,_,_=np.linalg.lstsq(design(X,"affine"),P,rcond=None)
s=np.sqrt(abs(np.linalg.det(Ca[1:3])))
d=rep-rep.mean(axis=0)
floor=np.sqrt((d**2).sum(axis=1).mean())/s
print(f"\nnoise floor from {len(rep)} repeats: {floor:.2f} mm    scale {s:.3f} px/mm")

best=min(results,key=lambda k:results[k][0])
print(f"best generalising model: {best}  ({results[best][0]:.2f} mm)")
json.dump({"kind":best,"coeffs":results[best][1].tolist(),"n":len(G),
           "loo_mm":results[best][0],"noise_floor_mm":float(floor),
           "pitch_rad":0.349,"z_m":0.274,"joint6":142,
           "px_per_mm":float(s)}, open("arm_camera_model.json","w"), indent=2)
print("saved arm_camera_model.json")
