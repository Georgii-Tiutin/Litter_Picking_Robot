#!/bin/bash
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
source /opt/ros/humble/setup.bash
source "$HOME/yahboomcar_ws/install/setup.bash" 2>/dev/null
exec python3 "$HOME/models/cuboid_v1/show_detect.py"
