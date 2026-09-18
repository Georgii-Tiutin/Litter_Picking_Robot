#!/bin/bash
source /opt/ros/humble/setup.bash
source "$HOME/yahboomcar_ws/install/setup.bash" 2>/dev/null
export CUBOID_MODEL="$HOME/models/two_stage/cuboid_twostage_rooms.pt"
export CUBOID_TOPIC="detect_twostage"
export CUBOID_LABEL="two-stage"
export CUBOID_NODE="detect_ts_node"
export CUBOID_OUT="$HOME/models/compare/out_ts"
exec python3 "$HOME/models/cuboid_v1/cuboid_detect.py"
