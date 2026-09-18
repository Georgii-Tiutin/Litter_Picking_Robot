#!/usr/bin/env python3
"""Publish the cubes found by cube_patrol.py as durable RViz markers.

cube_patrol.py published markers from its own process, so they vanished for any new
subscriber the moment the mission ended. This reads the saved result and keeps publishing,
colour-coded by how much evidence there is for each cube:

    green  - >=5 sightings : corroborated from several viewpoints
    amber  - 2-4 sightings : seen, but weakly - may be a duplicate of a neighbour
    red    - not on the floor (height > 0.10 m) : almost certainly a false positive
"""
import re, sys, math, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

SRC        = sys.argv[1] if len(sys.argv)>1 else "/home/jetson/maps/cubes_2026-09-13.txt"
import os
STRONG     = int(os.environ.get("STRONG","5"))   # sightings needed to call a cube corroborated
# When fed a list the TRACKER has already confirmed (3+ sightings from 2+ viewpoints), set
# STRONG=1: the confirmation decision was made with better evidence than a raw sighting count.
FLOOR_MAX  = 0.10   # above this it is not a cuboid lying on the floor

PAT=re.compile(r"cube\s+(\d+):\s+map\s+\(([-+0-9.]+),\s*([-+0-9.]+)\)\s+height\s+([0-9.]+)\s*m,\s*(\d+)\s+sighting")

def load(path):
    out=[]
    for line in open(path):
        m=PAT.search(line)
        if m:
            out.append((int(m.group(1)),float(m.group(2)),float(m.group(3)),
                        float(m.group(4)),int(m.group(5))))
    return out

def main():
    cubes=load(SRC)
    strong=[c for c in cubes if c[4]>=STRONG and c[3]<=FLOOR_MAX]
    print("%d cube record(s); %d corroborated (>=%d sightings, on the floor)"
          %(len(cubes),len(strong),STRONG))
    rclpy.init(); n=Node("cube_markers")
    lat=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=n.create_publisher(MarkerArray,"/cube_markers",lat)

    def build():
        ma=MarkerArray()
        for cid,x,y,h,s in cubes:
            off_floor = h>FLOOR_MAX
            weak = s<STRONG
            m=Marker()
            m.header.frame_id="map"; m.header.stamp=n.get_clock().now().to_msg()
            m.ns="cubes"; m.id=cid; m.type=Marker.CUBE; m.action=Marker.ADD
            m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=0.03
            m.pose.orientation.w=1.0
            m.scale.x=m.scale.y=m.scale.z=0.06
            if off_floor:  m.color.r,m.color.g,m.color.b=1.0,0.15,0.15
            elif weak:     m.color.r,m.color.g,m.color.b=1.0,0.72,0.0
            else:          m.color.r,m.color.g,m.color.b=0.1,1.0,0.2
            m.color.a=0.95
            ma.markers.append(m)
            t=Marker()
            t.header=m.header; t.ns="cube_labels"; t.id=1000+cid
            t.type=Marker.TEXT_VIEW_FACING; t.action=Marker.ADD
            t.pose.position.x=x; t.pose.position.y=y; t.pose.position.z=0.20
            t.pose.orientation.w=1.0; t.scale.z=0.08
            t.color.r=t.color.g=t.color.b=1.0; t.color.a=1.0
            t.text="#%d x%d%s"%(cid,s," OFF-FLOOR" if off_floor else ("?" if weak else ""))
            ma.markers.append(t)
        return ma

    ma=build()
    pub.publish(ma)
    n.create_timer(2.0, lambda: pub.publish(build()))
    print("publishing %d markers on /cube_markers (green=corroborated, amber=weak, red=off-floor)"
          %len(cubes))
    rclpy.spin(n)

if __name__=="__main__": sys.exit(main())
