"""Static dry run: waypoints + one detection, WITHOUT moving the robot."""
import sys, math, time
sys.argv=["dryrun"]
sys.path.insert(0,"/home/jetson/calib")
import rclpy
from cube_patrol import CubePatrol

rclpy.init(); e=CubePatrol()
t0=time.time()
while time.time()-t0<40 and (e.map is None or e.rgb is None or e.depth is None or e.pose() is None):
    rclpy.spin_once(e,timeout_sec=0.2)
for what,ok in (("map",e.map is not None),("colour",e.rgb is not None),
                ("depth",e.depth is not None),("pose",e.pose() is not None)):
    print("   %-8s %s"%(what,"ok" if ok else "MISSING"))
p=e.pose()
print("   robot at (%+.2f,%+.2f) heading %+.0f deg"%(p[0],p[1],math.degrees(p[2])))

print("\n--- waypoints ---")
wps=e.waypoints()
for i,(x,y) in enumerate(wps[:16]):
    print("   wp %2d (%+.2f,%+.2f)  %.2f m away"%(i+1,x,y,math.hypot(x-p[0],y-p[1])))
if len(wps)>16: print("   ... and %d more"%(len(wps)-16))

print("\n--- route check on the 3 nearest waypoints (no driving) ---")
order=sorted(range(len(wps)),key=lambda i:math.hypot(wps[i][0]-p[0],wps[i][1]-p[1]))
for i in order[:3]:
    x,y=wps[i]
    ok,why=e.route_ok(x,y)
    print("   (%+.2f,%+.2f): %s -> %s"%(x,y,"OK" if ok else "ABANDON",why))

print("\n--- one detection from where the robot stands ---")
e.spin(2.0)
for attempt in range(3):
    d=e.detect()
    print("   attempt %d: %d detection(s)"%(attempt+1,len(d)))
    for x,y,z,c in d:
        print("      cube at map (%+.3f,%+.3f) height %.3f m conf %.2f"%(x,y,z,c))
    e.add(d); e.spin(1.0)
e.publish()
print("\n   %d cube(s) currently marked; markers published on /cube_markers"%len(e.cubes))
rclpy.shutdown()
