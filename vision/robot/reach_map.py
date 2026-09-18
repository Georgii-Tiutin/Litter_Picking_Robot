#!/usr/bin/env python3
"""Map the reachable envelope: for each (z,pitch), which (x,y) survive IK+clamp+revalidate."""
import rclpy
from rclpy.node import Node
from arm_interface.srv import ArmKinemarics
TOL=5.0
class P(Node):
    def __init__(self):
        super().__init__("reach")
        self.c=self.create_client(ArmKinemarics,"get_kinemarics")
        while not self.c.wait_for_service(timeout_sec=1.0): pass
    def call(self,r):
        f=self.c.call_async(r); rclpy.spin_until_future_complete(self,f,timeout_sec=5.0); return f.result()
    def ok(self,x,y,z,p):
        r=ArmKinemarics.Request(); r.tar_x,r.tar_y,r.tar_z=x,y,z
        r.roll,r.pitch,r.yaw=0.0,p,0.0; r.kin_name="ik"
        s=self.call(r)
        if s is None: return False
        j=[s.joint1,s.joint2,s.joint3,s.joint4,s.joint5]
        c=[max(0,min(180,int(round(v)))) for v in j]
        q=ArmKinemarics.Request()
        q.cur_joint1,q.cur_joint2,q.cur_joint3=float(c[0]),float(c[1]),float(c[2])
        q.cur_joint4,q.cur_joint5,q.cur_joint6=float(c[3]),float(c[4]),0.0
        q.kin_name="fk"
        f=self.call(q)
        if f is None: return False
        e=(((f.x-x)**2+(f.y-y)**2+(f.z-z)**2)**0.5)*1000
        return e<=TOL
rclpy.init(); p=P()
xs=[0.14,0.16,0.18,0.20,0.22,0.24,0.26,0.28,0.30]
ys=[-0.09,-0.06,-0.03,0.0,0.03,0.06,0.09]
for pitch in (1.5708,1.2,0.9,0.6):
    for z in (0.02,0.05,0.08):
        grid=[]; n=0
        for y in ys:
            row=""
            for x in xs:
                good=p.ok(x,y,z,pitch); row+="#" if good else "."
                n+=good
            grid.append(row)
        print(f"pitch {pitch:.2f} z {z:.2f}  reachable {n:2d}/{len(xs)*len(ys)}")
        for r in grid: print("     "+r)
