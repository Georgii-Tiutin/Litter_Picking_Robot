#!/bin/bash
# Safe-stop the cube tracker AND zero the base velocity (it drives /cmd_vel).
source /opt/ros/humble/setup.bash
source ~/M3Pro_ws/install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=30
pkill -INT -f "[c]ube_arm_tracker.py"
sleep 3
pkill -9 -f "[c]ube_arm_tracker.py" 2>/dev/null
for i in $(seq 1 10); do
  ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1
  sleep 0.15
done
echo BASE_STOPPED
