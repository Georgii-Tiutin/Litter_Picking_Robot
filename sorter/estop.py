#!/usr/bin/env python3
"""Verified emergency stop.

Written after 2026-09-12, when killing the explorer did NOT stop the robot: Nav2's
controller_server keeps executing an accepted goal regardless of whether the action
client still exists. Killing the client removes the requester, not the request.

Never report the robot as stopped without running this and seeing it confirm.
"""
import subprocess, sys, time, math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal
import tf2_ros

HARD = "--hard" in sys.argv          # also kill the nav2 nodes

class EStop(Node):
    def __init__(self):
        super().__init__("estop")
        self.vel=self.create_publisher(Twist,"/cmd_vel",1)
        self.tfbuf=tf2_ros.Buffer(); self.tfl=tf2_ros.TransformListener(self.tfbuf,self)
        self.cancel=self.create_client(CancelGoal,"/navigate_to_pose/_action/cancel_goal")
    def spin(self,s):
        t0=time.time()
        while time.time()-t0<s: rclpy.spin_once(self,timeout_sec=0.05)
    def zero(self,n=15):
        t0=time.time()
        while self.vel.get_subscription_count()==0 and time.time()-t0<3:
            rclpy.spin_once(self,timeout_sec=0.05)
        for _ in range(n):
            self.vel.publish(Twist()); rclpy.spin_once(self,timeout_sec=0.02); time.sleep(0.06)
    def cancel_all(self):
        """Empty goal_id + zero stamp = cancel every goal on this action."""
        if not self.cancel.wait_for_service(timeout_sec=3.0):
            return "no cancel service (nav2 not running?)"
        req=CancelGoal.Request()            # zeroed goal_info == cancel all
        f=self.cancel.call_async(req)
        rclpy.spin_until_future_complete(self,f,timeout_sec=5.0)
        r=f.result()
        if r is None: return "cancel call failed"
        return "cancelled %d goal(s)"%len(r.goals_canceling)
    def pose(self):
        for frame in ("base_footprint","base_link"):
            for ref in ("map","odom"):
                try:
                    t=self.tfbuf.lookup_transform(ref,frame,rclpy.time.Time())
                    return ref,frame,t.transform.translation.x,t.transform.translation.y
                except Exception: pass
        return None

def kill(pattern,label):
    out=subprocess.run(["bash","-lc",
        "ps -eo pid,cmd | grep -E '%s' | grep -v grep | awk '{print $1}'"%pattern],
        capture_output=True,text=True).stdout.split()
    for p in out:
        subprocess.run(["kill","-9",p],capture_output=True)
    return "%s: killed %d"%(label,len(out))

def main():
    rclpy.init(); e=EStop(); e.spin(0.8)
    print("1. cancelling Nav2 goals      : %s"%e.cancel_all())
    print("2. %s"%kill("explore\\.py|sweep_.*\\.py|chase\\.py|cuboid_grasp\\.py","killing cmd_vel clients"))
    if HARD:
        print("   %s"%kill("controller_server|bt_navigator|component_container","killing nav2 nodes"))
        time.sleep(2)
    print("3. publishing zero velocity   : sent")
    e.zero()
    print("4. verifying the robot is actually still:")
    samples=[]
    for i in range(3):
        e.spin(0.6); p=e.pose()
        if p is None:
            print("     sample %d: no TF available"%i); time.sleep(1.6); continue
        ref,frame,x,y=p
        samples.append((x,y))
        print("     sample %d: %s->%s (%+.3f, %+.3f)"%(i,ref,frame,x,y))
        time.sleep(1.8)
    moved=None
    if len(samples)>=2:
        moved=max(math.hypot(a[0]-b[0],a[1]-b[1]) for a in samples for b in samples)
        print("     max movement across samples: %.1f mm"%(moved*1000))
    print("5. /cmd_vel check:")
    r=subprocess.run(["bash","-lc",
        "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=30; "
        "timeout 5 ros2 topic echo /cmd_vel 2>/dev/null | grep -cE '^  x:'"],
        capture_output=True,text=True)
    n=r.stdout.strip() or "0"
    print("     %s velocity messages seen in 5 s"%n)
    ok = (moved is not None and moved<0.02) and n in ("0","")
    print("\n%s"%("STOPPED - verified stationary and nothing commanding velocity" if ok
                  else "NOT CONFIRMED - still moving or still being commanded; rerun with --hard"))
    return 0 if ok else 1

if __name__=="__main__": sys.exit(main())
