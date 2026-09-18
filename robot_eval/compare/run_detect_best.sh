#!/bin/bash
source /opt/ros/humble/setup.bash
source "$HOME/yahboomcar_ws/install/setup.bash" 2>/dev/null
export CUBOID_MODEL="$HOME/models/cuboid_v1/best.pt"
export CUBOID_TOPIC="detect_best"
export CUBOID_LABEL="best.pt"
export CUBOID_NODE="detect_best_node"
export CUBOID_OUT="$HOME/models/compare/out_best"
exec python3 "$HOME/models/cuboid_v1/cuboid_detect.py"
