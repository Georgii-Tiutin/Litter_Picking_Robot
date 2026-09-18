#!/bin/bash
# Cube arm tracker: track -> face -> approach. Home/start pose = [90,135,0,25,90,0].
source /opt/ros/humble/setup.bash
source ~/yahboomcar_ws/install/setup.bash
source ~/M3Pro_ws/install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=30
export DISPLAY=:0
export XAUTHORITY=/home/jetson/.Xauthority
export CUBE_MOVE=1
export CUBE_SIGN=1
export CUBE_SIGN_BASE=-1
export CUBE_HOME_J2=135
export CUBE_CONF=0.5
export CUBE_APPROACH_MAXT=20
python3 /home/jetson/cube_tracker/cube_arm_tracker.py
