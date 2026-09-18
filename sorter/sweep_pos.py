#!/usr/bin/env python3
"""Position sweep: the robot places the block at each target, then tries to grasp it.

Orientation is left to self-align (the j5 sweep showed the wrist does not matter);
this varies POSITION across the reach envelope and finds where grasps start failing.
Also logs commanded-vs-detected placement, which measures placement accuracy.
"""
import sys, math, time, csv, numpy as np, rclpy
sys.path.insert(0,"/home/jetson/calib")
import grab_in_place as G

TARGETS=[(0.17,-0.05),(0.17,0.00),(0.17,0.05),
         (0.21,-0.05),(0.21,0.00),(0.21,0.05),
         (0.25,-0.05),(0.25,0.00),(0.25,0.05)]

def observe(g):
    g.send_joints(G.OBS,1800); g.spin(0.8)
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

def grab(g,det):
    f=det["fine"]; b=f["base"]
    ja,jg=plan(g,b[0],b[1],f["h"])
    if ja is None: return False,"unreachable"
    g.send_joints(ja+[G.GRIP_OPEN],1500)
    g.send_joints(jg+[G.GRIP_OPEN],1300)
    g.send_joints(jg+[G.GRIP_CLOSE],1000)
    time.sleep(0.7)
    g.send_joints(ja+[G.GRIP_CLOSE],1300)
    still=observe(g)
    if still is None: return True,"held"
    d=math.hypot(still["fine"]["base"][0]-b[0],still["fine"]["base"][1]-b[1])*1000
    return (d>60),"moved %.0f mm"%d

def place(g,x,y,h=0.030):
    ja,jg=plan(g,x,y,h)
    if ja is None: return False
    g.send_joints(ja+[G.GRIP_CLOSE],1500)
    g.send_joints(jg+[G.GRIP_CLOSE],1300)
    g.send_joints(jg+[G.GRIP_OPEN],1000)
    g.send_joints(ja+[G.GRIP_OPEN],1200)
    return True

def main():
    rclpy.init(); g=G.Grab()
    t0=time.time()
    while time.time()-t0<15 and (g.rgb is None or g.depth is None): rclpy.spin_once(g,timeout_sec=0.3)
    g.kin.wait_for_service(timeout_sec=10.0)

    det=observe(g)
    if det is None: print("no block visible to start"); return
    ok,msg=grab(g,det)
    if not ok: print("initial pick failed (%s) - cannot run sweep"%msg); return
    print("initial pick ok\n")
    print("%6s %6s   %8s %8s %7s   %s"%("tx","ty","det_x","det_y","place_err","grasp"))
    rows=[]
    for (tx,ty) in TARGETS:
        if not place(g,tx,ty):
            print("%6.2f %6.2f   -- cannot place there --"%(tx,ty)); continue
        det=observe(g)
        if det is None:
            print("%6.2f %6.2f   -- lost sight after placing --"%(tx,ty)); break
        b=det["fine"]["base"]
        perr=math.hypot(b[0]-tx,b[1]-ty)*1000
        ok,msg=grab(g,det)
        print("%6.2f %6.2f   %8.4f %8.4f %6.0fmm   %s"%(tx,ty,b[0],b[1],perr,"GRIP" if ok else msg))
        rows.append([tx,ty,round(b[0],4),round(b[1],4),round(perr),int(ok),msg])
        if not ok:
            det=observe(g)
            if det is None: break
            ok2,_=grab(g,det)
            if not ok2:
                print("       recovery pick also failed - stopping sweep"); break
    with open("/home/jetson/calib/pos_sweep.csv","w",newline="") as fh:
        w=csv.writer(fh); w.writerow(["target_x","target_y","det_x","det_y","place_err_mm","grip_ok","note"])
        w.writerows(rows)
    good=[r for r in rows if r[5]]
    print("\n%d of %d positions gripped"%(len(good),len(rows)))
    if rows:
        errs=[r[4] for r in rows]
        print("placement error: mean %.0f mm, max %.0f mm"%(np.mean(errs),max(errs)))
    g.send_joints(G.OBS,1800)

if __name__=="__main__": main()
