#!/usr/bin/env python3
"""Dry-run IK for a grid: report solved + clamped joints, no motion."""
import sys, rclpy
from rclpy.node import Node
from arm_interface.srv import ArmKinemarics

class P(Node):
    def __init__(self):
        super().__init__("probe")
        self.cli=self.create_client(ArmKinemarics,"get_kinemarics")
        while not self.cli.wait_for_service(timeout_sec=1.0): pass
    def call(self,r):
        f=self.cli.call_async(r); rclpy.spin_until_future_complete(self,f,timeout_sec=5.0)
        return f.result()
    def ik(self,x,y,z,p):
        r=ArmKinemarics.Request(); r.tar_x,r.tar_y,r.tar_z=x,y,z
        r.roll,r.pitch,r.yaw=0.0,p,0.0; r.kin_name="ik"; return self.call(r)
    def fk(self,j):
        r=ArmKinemarics.Request()
        r.cur_joint1,r.cur_joint2,r.cur_joint3=float(j[0]),float(j[1]),float(j[2])
        r.cur_joint4,r.cur_joint5,r.cur_joint6=float(j[3]),float(j[4]),0.0
        r.kin_name="fk"; return self.call(r)

rclpy.init(); p=P(); Z,PITCH=0.274,0.349
print("     x      y   raw joints                             clamped            err_mm")
for x in [0.16,0.19,0.22,0.25,0.28]:
    for y in [-0.08,-0.04,0.0,0.04,0.08]:
        s=p.ik(x,y,Z,PITCH)
        j=[s.joint1,s.joint2,s.joint3,s.joint4,s.joint5]
        c=[max(0,min(180,int(round(v)))) for v in j]
        f=p.fk(c)
        e=(((f.x-x)**2+(f.y-y)**2+(f.z-Z)**2)**0.5)*1000
        flag="" if e<=5 else "  <-- REJECT"
        print(f"{x:6.2f}{y:+7.2f}   {[round(v,1) for v in j]!s:34s} {c!s:22s} {e:6.2f}{flag}")
