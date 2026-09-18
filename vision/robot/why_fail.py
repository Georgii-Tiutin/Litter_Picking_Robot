import rclpy
from rclpy.node import Node
from arm_interface.srv import ArmKinemarics
class P(Node):
    def __init__(self):
        super().__init__("wf"); self.c=self.create_client(ArmKinemarics,"get_kinemarics")
        while not self.c.wait_for_service(timeout_sec=1.0): pass
    def call(self,r):
        f=self.c.call_async(r); rclpy.spin_until_future_complete(self,f,timeout_sec=5.0); return f.result()
rclpy.init(); p=P()
print("     x      y     z  raw joints                        clamped                err_mm")
for z in (0.06,0.08):
    for x in (0.22,0.25,0.28):
        y=0.0
        r=ArmKinemarics.Request(); r.tar_x,r.tar_y,r.tar_z=x,y,z
        r.roll,r.pitch,r.yaw=0.0,1.5708,0.0; r.kin_name="ik"
        s=p.call(r)
        j=[s.joint1,s.joint2,s.joint3,s.joint4,s.joint5]
        c=[max(0,min(180,int(round(v)))) for v in j]
        q=ArmKinemarics.Request()
        q.cur_joint1,q.cur_joint2,q.cur_joint3=float(c[0]),float(c[1]),float(c[2])
        q.cur_joint4,q.cur_joint5,q.cur_joint6=float(c[3]),float(c[4]),0.0
        q.kin_name="fk"; f=p.call(q)
        e=(((f.x-x)**2+(f.y-y)**2+(f.z-z)**2)**0.5)*1000
        print(f"{x:6.2f}{y:+7.2f}{z:6.2f}  {[round(v,1) for v in j]!s:32s} {c!s:22s} {e:7.2f}")
