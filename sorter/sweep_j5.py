#!/usr/bin/env python3
"""Sweep joint5 to find the rule mapping object orientation -> jaw angle.

Every trial re-detects first (a failed grab nudges the block), and successes are
placed back down, so each row is logged against the orientation it actually had.
"""
import sys, math, time, csv, numpy as np, rclpy
sys.path.insert(0,"/home/jetson/calib")
import grab_in_place as G

J5_LIST=[int(v) for v in (sys.argv[1].split(",") if len(sys.argv)>1
                          else "55,75,95,115,135,155,175".split(","))]
PITCHES=G.PITCHES

def main():
    rclpy.init(); g=G.Grab()
    t0=time.time()
    while time.time()-t0<15 and (g.rgb is None or g.depth is None):
        rclpy.spin_once(g,timeout_sec=0.3)
    g.kin.wait_for_service(timeout_sec=10.0)
    rows=[]
    print("%4s %8s %8s %8s %7s %6s  %s"%("j5","short_ang","grip_j","servo_j1","short","long","result"))
    for j5 in J5_LIST:
        g.send_joints(G.OBS,2000); g.spin(0.8)
        s=g.fk(G.OBS[:5]); g.CurEndPos=[s.x,s.y,s.z,s.roll,s.pitch,s.yaw]
        det=g.detect(frames=5)
        if det is None:
            print("%4d  -- no detection --"%j5); continue
        f=det["fine"]; b=f["base"]
        gj=math.degrees(math.atan2(f["ivx"],f["ivy"]))
        gz=max(0.010,f["h"]*0.5)
        above=f["h"]+G.CLEAR
        sol=None
        for p in PITCHES:
            ja,e1=g.solve(b[0],b[1],above,p)
            jg,e2=g.solve(b[0],b[1],gz,p)
            if ja and jg: sol=(p,ja,jg); break
        if sol is None:
            print("%4d  -- unreachable --"%j5); continue
        p,ja,jg=sol
        ja=ja[:4]+[j5]; jg=jg[:4]+[j5]
        if jg[3]>90: ja[3]=min(ja[3],90); jg[3]=min(jg[3],90)
        g.send_joints(ja+[G.GRIP_OPEN],1600)
        g.send_joints(jg+[G.GRIP_OPEN],1400)
        g.send_joints(jg+[G.GRIP_CLOSE],1100)
        time.sleep(0.7)
        g.send_joints(ja+[G.GRIP_CLOSE],1400)
        # verify
        g.send_joints([G.OBS[0],G.OBS[1],G.OBS[2],G.OBS[3],G.OBS[4],G.GRIP_CLOSE],1800); g.spin(1.0)
        still=g.detect(frames=4)
        ok = still is None
        moved=None
        if not ok:
            moved=math.hypot(still["fine"]["base"][0]-b[0],still["fine"]["base"][1]-b[1])*1000
            ok = moved>60     # gone far => probably picked and the detector saw another object
        print("%4d %8.1f %8.1f %8d %7.0f %6.0f  %s"
              %(j5,f["short_ang"],gj,jg[0],f["short_mm"],f["long_mm"],
                "GRIP" if ok else "miss (moved %.0f mm)"%moved))
        rows.append([j5,round(f["short_ang"],1),round(gj,1),jg[0],
                     round(f["short_mm"]),round(f["long_mm"]),
                     round(f["ivx"],1),round(f["ivy"],1),int(ok),
                     "" if moved is None else round(moved)])
        if ok:
            # put it back down so the next trial has a block to work with
            g.send_joints(jg+[G.GRIP_CLOSE],1400)
            g.send_joints(jg+[G.GRIP_OPEN],1000)
            g.send_joints(ja+[G.GRIP_OPEN],1200)
    with open("/home/jetson/calib/j5_sweep.csv","w",newline="") as fh:
        w=csv.writer(fh)
        w.writerow(["j5","short_ang_base","gripper_joint","servo_j1","short_mm","long_mm",
                    "ivx","ivy","grip_ok","moved_mm"]); w.writerows(rows)
    ok=[r for r in rows if r[8]]
    print("\n%d of %d trials gripped"%(len(ok),len(rows)))
    if ok:
        print("successful j5 values: %s"%[r[0] for r in ok])
    g.send_joints(G.OBS,2000)

if __name__=="__main__": main()
