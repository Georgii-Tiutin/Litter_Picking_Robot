#!/bin/bash
source /opt/ros/humble/setup.bash
source "$HOME/yahboomcar_ws/install/setup.bash" 2>/dev/null
export CUBOID_MODEL="$HOME/models/v3_clean/cuboid_v3_clean.pt"
export CUBOID_TOPIC="detect_v3clean"
export CUBOID_LABEL="v3-clean"
export CUBOID_NODE="detect_v3_node"
export CUBOID_OUT="$HOME/models/compare/out_v3"
exec python3 "$HOME/models/cuboid_v1/cuboid_detect.py"
