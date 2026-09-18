#!/usr/bin/env python
"""Segment objects on the carpet from the overhead view, map to arm coords."""
import csv, sys, numpy as np, cv2
CORNERS=[(1182,45),(74,52),(76,651),(1185,656)]

# floor-level model: pixel -> arm mm
R=list(csv.DictReader(open("floor_grid.csv")))
X=np.array([[float(r["x_mm"]),float(r["y_mm"])] for r in R])
P=np.array([[float(r["u_px"]),float(r["v_px"])] for r in R])
def dz(A):
    u,v=A[:,0],A[:,1]; o=np.ones_like(u)
    return np.column_stack([o,u,v,u*v])
C,_,_,_=np.linalg.lstsq(dz(P),X,rcond=None)
res=X-dz(P)@C
print(f"floor model: {len(R)} pts, fit residual {np.sqrt((res**2).sum(axis=1)).mean():.2f} mm\n")
def px2arm(u,v): return (dz(np.array([[u,v]]))@C)[0]

img=cv2.imread(sys.argv[1] if len(sys.argv)>1 else "scene_01.jpg")
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
H,S,V=hsv[:,:,0].astype(int),hsv[:,:,1].astype(int),hsv[:,:,2].astype(int)
# carpet: low saturation mid value. objects: saturated OR distinctly bright/dark
mask=((S>70)|(V<60)).astype(np.uint8)*255
mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
n,lab,st,cen=cv2.connectedComponentsWithStats(mask,8)
print(f"{'#':>3} {'centroid':>16} {'area':>6} {'w x h':>9}  {'hue':>4} {'arm x,y (mm)':>16}  reach")
out=[]
for i in range(1,n):
    a=st[i,cv2.CC_STAT_AREA]
    if a<250 or a>60000: continue
    cx,cy=cen[i]
    if min(np.hypot(cx-qx,cy-qy) for qx,qy in CORNERS)<60: continue   # corner tags excluded
    if cx<640: continue                                              # robot body side
    w,h=st[i,cv2.CC_STAT_WIDTH],st[i,cv2.CC_STAT_HEIGHT]
    hue=int(np.median(H[lab==i])); sat=int(np.median(S[lab==i]))
    ax,ay=px2arm(cx,cy)
    ok = (135<=ax<=185) and (-95<=ay<=95)
    print(f"{i:>3} ({cx:7.1f},{cy:6.1f}) {a:>6} {w:>4}x{h:<4} {hue:>4} ({ax:7.1f},{ay:+7.1f})  {'YES' if ok else 'no'}")
    out.append((i,cx,cy,a,w,h,hue,sat,ax,ay,ok))
print(f"\n{sum(1 for o in out if o[10])} of {len(out)} objects inside the graspable strip")
vis=img.copy()
for o in out:
    c=(0,255,0) if o[10] else (0,0,255)
    cv2.circle(vis,(int(o[1]),int(o[2])),14,c,2)
    cv2.putText(vis,str(o[0]),(int(o[1])+16,int(o[2])),cv2.FONT_HERSHEY_SIMPLEX,0.5,c,2)
cv2.imwrite("objects_detected.jpg",vis)
