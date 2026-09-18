#!/usr/bin/env python
import csv, numpy as np, cv2
CORNERS=[(1182,45),(74,52),(76,651),(1185,656)]
R=[r for r in csv.DictReader(open("floor_grid06.csv"))]
X=np.array([[float(r["x_mm"]),float(r["y_mm"])] for r in R])
P=np.array([[float(r["u_px"]),float(r["v_px"])] for r in R])
A=np.array([float(r["area"]) for r in R])
def dz(Q):
    u,v=Q[:,0],Q[:,1]; o=np.ones_like(u)
    return np.column_stack([o,u,v,u*v])
keep=np.ones(len(X),bool)
for it in range(4):                       # iterative outlier rejection
    C,_,_,_=np.linalg.lstsq(dz(P[keep]),X[keep],rcond=None)
    res=np.sqrt(((X-dz(P)@C)**2).sum(axis=1))
    thr=max(6.0, np.percentile(res[keep],75)*2)
    new=res<thr
    if (new==keep).all(): break
    keep=new
C,_,_,_=np.linalg.lstsq(dz(P[keep]),X[keep],rcond=None)
res=np.sqrt(((X-dz(P)@C)**2).sum(axis=1))
print(f"floor model z=0.06: {keep.sum()}/{len(X)} inliers, residual {res[keep].mean():.2f} mm "
      f"(dropped {(~keep).sum()} outliers)")
umin,umax=P[keep][:,0].min(),P[keep][:,0].max()
vmin,vmax=P[keep][:,1].min(),P[keep][:,1].max()
print(f"calibrated pixel region: u {umin:.0f}-{umax:.0f}, v {vmin:.0f}-{vmax:.0f}\n")
def px2arm(u,v): return (dz(np.array([[float(u),float(v)]]))@C)[0]

img=cv2.imread("scene_clean.jpg"); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
S,V=hsv[:,:,1].astype(int),hsv[:,:,2].astype(int)
mask=(((S>70)|(V<70))*255).astype(np.uint8)
mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
mask[:, :700]=0                                    # robot body + arm live on the left
n,lab,st,cen=cv2.connectedComponentsWithStats(mask,8)
print(f"{'#':>3}{'centroid':>17}{'short':>7}{'long':>7}{'ang':>6}   {'arm x,y mm':>17}  status")
objs=[]
for i in range(1,n):
    a=st[i,cv2.CC_STAT_AREA]
    if a<300 or a>40000: continue
    cx,cy=cen[i]
    if min(np.hypot(cx-qx,cy-qy) for qx,qy in CORNERS)<70: continue
    pts=cv2.findNonZero((lab==i).astype(np.uint8))
    (rx,ry),(w,h),ang=cv2.minAreaRect(pts)
    PXMM=1.35
    short,long_=min(w,h)/PXMM,max(w,h)/PXMM
    ax,ay=px2arm(cx,cy)
    inbox = (umin-25<=cx<=umax+25) and (vmin-25<=cy<=vmax+25)
    grip  = 12<=short<=55
    stat = "REACH+GRIP" if (inbox and grip) else ("out of reach" if not inbox else "too wide")
    print(f"{i:>3}({cx:7.1f},{cy:6.1f}){short:7.1f}{long_:7.1f}{ang:6.0f}   ({ax:7.1f},{ay:+7.1f})  {stat}")
    objs.append(dict(i=i,cx=cx,cy=cy,short=short,long=long_,ang=ang,ax=ax,ay=ay,ok=inbox and grip))
print(f"\n{sum(o['ok'] for o in objs)} of {len(objs)} objects are both reachable and grippable")
vis=img.copy()
cv2.rectangle(vis,(int(umin),int(vmin)),(int(umax),int(vmax)),(255,255,0),2)
for o in objs:
    c=(0,255,0) if o["ok"] else (0,0,255)
    cv2.circle(vis,(int(o["cx"]),int(o["cy"])),16,c,2)
    cv2.putText(vis,f"{o['i']}",(int(o["cx"])+18,int(o["cy"])),cv2.FONT_HERSHEY_SIMPLEX,0.6,c,2)
cv2.imwrite("objects_mapped.jpg",vis)
