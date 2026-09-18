#!/usr/bin/env python3
"""Calibrate jaw angle -> block orientation by PLACING, not grabbing.

Placing leaves the block aligned to the jaw axis, so (j5_place, short_ang_measured)
pairs give the mapping directly, with no failed grasps needed.
"""
import sys, math, time, csv, numpy as np, rclpy
sys.path.insert(0,"/home/jetson/calib")
import grab_in_place as G

PLACE_J5=[50,80,110,140,170]
BOOTSTRAP=[None,45,70,95,120,145,170]     # None = vendor formula

def observe(g):
    g.send_joints(G.OBS,1700); g.spin(0.8)
    s=g.fk(G.OBS[:5]); g.CurEndPos=[s.x,s.y,s.z,s.roll,s.pitch,s.yaw]
    return g.detect(frames=5)

def plan(g,x,y,h):
    gz=max(0.010,h*0.5); above=h+G.CLEAR
    for p in G.PITCHES:
        ja,_=g.solve(x,y,above,p); jg,_=g.solve(x,y,gz,p)
        if ja and jg:
            if jg[3]>90: ja[3]=min(ja[3],90); jg[3]=min(jg[3],90)
            return ja,jg
    return None,None

def vendor_j5(gj,j1):
    if gj<0:
        v=abs(gj); v=(180-j1+v-90) if abs(gj)<90 else (v-j1+90)
    elif gj>0:
        v=180-abs(gj); v=(v-j1) if gj<90 else (v-(j1-90))
    else: return 90
    while v>135: v-=90
    while v<45:  v+=90
    return int(round(v))

def try_grip(g,det,j5):
    f=det["fine"]; b=f["base"]
    ja,jg=plan(g,b[0],b[1],f["h"])
    if ja is None: return False,None,"unreachable"
    if j5 is None:
        gj=math.degrees(math.atan2(f["ivx"],f["ivy"])); j5=vendor_j5(gj,jg[0])
    ja=ja[:4]+[j5]; jg=jg[:4]+[j5]
    g.send_joints(ja+[G.GRIP_OPEN],1500)
    g.send_joints(jg+[G.GRIP_OPEN],1300)
    g.send_joints(jg+[G.GRIP_CLOSE],1000)
    time.sleep(0.7)
    g.send_joints(ja+[G.GRIP_CLOSE],1300)
    still=observe(g)
    if still is None: return True,(ja,jg,j5),"held"
    d=math.hypot(still["fine"]["base"][0]-b[0],still["fine"]["base"][1]-b[1])*1000
    return False,None,"moved %.0f mm"%d

def place_with(g,ja,jg,j5):
    ja=ja[:4]+[j5]; jg=jg[:4]+[j5]
    g.send_joints(ja+[G.GRIP_CLOSE],1400)
    g.send_joints(jg+[G.GRIP_CLOSE],1200)
    g.send_joints(jg+[G.GRIP_OPEN],1000)
    g.send_joints(ja+[G.GRIP_OPEN],1200)

def main():
    rclpy.init(); g=G.Grab()
    t0=time.time()
    while time.time()-t0<15 and (g.rgb is None or g.depth is None): rclpy.spin_once(g,timeout_sec=0.3)
    g.kin.wait_for_service(timeout_sec=10.0)

    print("--- bootstrap: find any j5 that grips ---")
    held=None
    for j5 in BOOTSTRAP:
        det=observe(g)
        if det is None: print("   no block visible"); return
        f=det["fine"]
        ok,info,msg=try_grip(g,det,j5)
        print("   j5=%-6s short_ang %5.1f  dist %.0f mm -> %s"
              %(str(j5),f["short_ang"],math.hypot(f["base"][0],f["base"][1])*1000,
                "GRIP" if ok else msg))
        if ok: held=info; break
    if held is None: print("could not grip at any j5 - stopping"); return
    ja,jg,_=held

    print("\n--- calibration: place at j5, measure resulting orientation ---")
    print("%6s %12s %10s %8s"%("j5","short_ang","long_mm","dist_mm"))
    rows=[]
    for p5 in PLACE_J5:
        place_with(g,ja,jg,p5)
        det=observe(g)
        if det is None: print("%6d   lost sight after placing"%p5); break
        f=det["fine"]
        print("%6d %12.1f %10.0f %8.0f"%(p5,f["short_ang"],f["long_mm"],
                                          math.hypot(f["base"][0],f["base"][1])*1000))
        rows.append([p5,round(f["short_ang"],1),round(f["long_mm"]),round(f["short_mm"]),
                     round(f["base"][0],4),round(f["base"][1],4),jg[0]])
        ok,info,msg=try_grip(g,det,p5)     # same angle it was placed with -> aligned
        if not ok:
            print("       re-grip at same j5 failed (%s) - stopping"%msg); break
        ja,jg,_=info
    with open("/home/jetson/calib/j5_map.csv","w",newline="") as fh:
        w=csv.writer(fh); w.writerow(["j5_place","short_ang_base","long_mm","short_mm","x","y","servo_j1"])
        w.writerows(rows)
    if len(rows)>=3:
        J=np.array([r[0] for r in rows],float); A=np.array([r[1] for r in rows],float)
        A=np.unwrap(np.deg2rad(A*2))/2      # 180-deg ambiguity
        A=np.rad2deg(A)
        m,c=np.polyfit(J,A,1)
        pred=m*J+c
        print("\nfit: short_ang = %.3f * j5 + %.1f   (residual rms %.1f deg)"
              %(m,c,float(np.sqrt(((A-pred)**2).mean()))))
        print("inverse (what to command): j5 = (short_ang - %.1f) / %.3f"%(c,m))
    g.send_joints(G.OBS,1700)

if __name__=="__main__": main()
