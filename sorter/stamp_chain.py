"""Measure timestamp age at every stage of the scan pipeline, to find where the future
offset is introduced:  /scan0 /scan1  ->  merger  ->  /scan_multi  ->  filter  ->  /scan
Negative age = the message claims to be from the FUTURE.
"""
import rclpy, statistics
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

TOPICS=["/scan0","/scan1","/scan_multi","/scan"]
rclpy.init(); n=Node("stampchain")
ages={t:[] for t in TOPICS}
frames={t:set() for t in TOPICS}

def mk(t):
    def cb(m):
        now=n.get_clock().now().nanoseconds/1e9
        st=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        ages[t].append(now-st); frames[t].add(m.header.frame_id)
    return cb

for t in TOPICS:
    n.create_subscription(LaserScan,t,mk(t),qos_profile_sensor_data)

t0=n.get_clock().now()
while (n.get_clock().now()-t0).nanoseconds<25e9: rclpy.spin_once(n,timeout_sec=0.05)

print("%-14s %7s %9s %9s %9s   %s"%("topic","msgs","median","min","max","frame_id"))
for t in TOPICS:
    a=ages[t]
    if not a:
        print("%-14s %7d %9s"%(t,0,"(silent)")); continue
    print("%-14s %7d %+9.4f %+9.4f %+9.4f   %s"
          %(t,len(a),statistics.median(a),min(a),max(a),",".join(frames[t]) or "-"))
print("\nnegative = stamped in the FUTURE (this is what makes AMCL discard scans)")
rclpy.shutdown()
