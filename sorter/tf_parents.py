"""Find frames that have more than one parent - illegal in TF and a classic cause of
two consumers disagreeing about where a sensor is pointing."""
import rclpy, collections
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

rclpy.init(); n=Node("tf_parents")
parents=collections.defaultdict(set); src=collections.defaultdict(set)
def cb(msg,kind):
    for t in msg.transforms:
        parents[t.child_frame_id].add(t.header.frame_id)
        src[(t.header.frame_id,t.child_frame_id)].add(kind)
n.create_subscription(TFMessage,"/tf",lambda m:cb(m,"/tf"),50)
n.create_subscription(TFMessage,"/tf_static",lambda m:cb(m,"/tf_static"),
    QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST))
t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<12e9: rclpy.spin_once(n,timeout_sec=0.2)

bad=[(c,p) for c,p in parents.items() if len(p)>1]
print("frames seen: %d"%len(parents))
if bad:
    print("\n*** FRAMES WITH MULTIPLE PARENTS (TF is ambiguous here) ***")
    for c,p in bad:
        print("   %-34s parents: %s"%(c,", ".join(sorted(p))))
        for pp in sorted(p):
            print("        via %-24s on %s"%(pp,",".join(sorted(src[(pp,c)]))))
else:
    print("\nno frame has more than one parent")

print("\ncamera-related transforms being published:")
for (p,c),k in sorted(src.items()):
    if "camera" in c.lower() or "camera" in p.lower() or "DCW" in c or "arm" in p.lower():
        print("   %-28s -> %-34s  [%s]"%(p,c,",".join(sorted(k))))
rclpy.shutdown()
