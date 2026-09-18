"""Ask the global planner directly whether the failed goal was reachable, and
measure how much free space the robot actually has around it."""
import math, numpy as np, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
import tf2_ros

GOAL=(-1.40,-1.34)

rclpy.init(); n=Node("why_stuck")
buf=tf2_ros.Buffer(); tf2_ros.TransformListener(buf,n)
grids={}
for topic,key in (("/map","map"),("/local_costmap/costmap","local"),("/global_costmap/costmap","glob")):
    n.create_subscription(OccupancyGrid,topic,lambda m,k=key:grids.__setitem__(k,m),
        rclpy.qos.QoSProfile(depth=1,reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                             durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<8e9 and len(grids)<3:
    rclpy.spin_once(n,timeout_sec=0.2)

try:
    tr=buf.lookup_transform("map","base_footprint",rclpy.time.Time()).transform
    rx,ry=tr.translation.x,tr.translation.y
    q=tr.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y**2+q.z**2))
    print("robot at (%+.2f,%+.2f) heading %+.0f deg"%(rx,ry,math.degrees(yaw)))
except Exception as e:
    print("no pose:",e); rx=ry=yaw=0.0

# how boxed in is the robot, according to the costmap Nav2 plans on?
g=grids.get("glob")
if g is not None:
    a=np.array(g.data,dtype=np.int16).reshape(g.info.height,g.info.width)
    res=g.info.resolution
    print("\nclearance around the robot on the GLOBAL costmap (cost>=99 blocks the footprint):")
    for deg in range(0,360,30):
        th=math.radians(deg); blocked=None
        for d in np.arange(0.10,1.60,res):
            x=rx+d*math.cos(th); y=ry+d*math.sin(th)
            cx=int((x-g.info.origin.position.x)/res); cy=int((y-g.info.origin.position.y)/res)
            if not(0<=cx<g.info.width and 0<=cy<g.info.height): blocked=("edge",d); break
            if a[cy,cx]>=99: blocked=("cost %d"%a[cy,cx],d); break
        print("   %+4d deg: %s" % (deg, "clear to 1.6 m" if blocked is None
              else "blocked at %.2f m (%s)"%(blocked[1],blocked[0])))

ac=ActionClient(n,ComputePathToPose,"compute_path_to_pose")
if not ac.wait_for_server(timeout_sec=8.0):
    print("\nno planner server"); raise SystemExit
def plan(gx,gy,label):
    gm=ComputePathToPose.Goal(); gm.use_start=False
    p=PoseStamped(); p.header.frame_id="map"; p.pose.position.x=float(gx); p.pose.position.y=float(gy)
    p.pose.orientation.w=1.0; gm.goal=p
    f=ac.send_goal_async(gm); rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    h=f.result()
    if h is None or not h.accepted: print("   %-22s REJECTED"%label); return
    rf=h.get_result_async(); rclpy.spin_until_future_complete(n,rf,timeout_sec=20)
    r=rf.result()
    poses=r.result.path.poses if r is not None else []
    if not poses: print("   %-22s NO PATH FOUND"%label); return
    L=sum(math.hypot(poses[i+1].pose.position.x-poses[i].pose.position.x,
                     poses[i+1].pose.position.y-poses[i].pose.position.y) for i in range(len(poses)-1))
    d=math.hypot(gx-rx,gy-ry)
    print("   %-22s path OK: %d poses, %.2f m long (straight line %.2f m)"%(label,len(poses),L,d))

print("\nCan the planner reach the goal it kept failing on?")
plan(GOAL[0],GOAL[1],"the failed goal")
print("\nCan it reach anywhere at all? (1 m in each direction)")
for deg,name in ((0,"1 m forward(+x)"),(90,"1 m left(+y)"),(180,"1 m back(-x)"),(270,"1 m right(-y)")):
    th=math.radians(deg); plan(rx+math.cos(th),ry+math.sin(th),name)
rclpy.shutdown()
