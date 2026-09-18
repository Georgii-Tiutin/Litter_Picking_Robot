#!/bin/bash
source /opt/ros/humble/setup.bash
source "$HOME/yahboomcar_ws/install/setup.bash" 2>/dev/null
exec ros2 launch orbbec_camera dabai_dcw2.launch.py
