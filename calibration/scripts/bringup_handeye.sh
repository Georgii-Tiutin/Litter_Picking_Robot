#!/usr/bin/env bash
# Bring up the minimum stack for hand-eye capture.
# Usage:  bash ~/project0/calibration/scripts/bringup_handeye.sh
# Then in another terminal:
#   python3 ~/project0/calibration/scripts/capture_handeye.py
set -e

source /opt/ros/humble/setup.bash
source ~/yahboomcar_ws/install/setup.bash
export ROS_DOMAIN_ID=30

LOG_DIR=~/project0/logs
mkdir -p "$LOG_DIR"

stop() {
  echo "shutting down..."
  pkill -f "ros2 launch orbbec" 2>/dev/null || true
  pkill -f "arm_kin kin_srv"    2>/dev/null || true
  exit 0
}
trap stop INT TERM

echo "[1/2] launching Orbbec DCW2 driver..."
ros2 launch orbbec_camera dabai_dcw2.launch.py \
  > "$LOG_DIR/orbbec.log" 2>&1 &
ORBBEC_PID=$!

echo "[2/2] launching arm_kin/kin_srv..."
ros2 run arm_kin kin_srv \
  > "$LOG_DIR/kin_srv.log" 2>&1 &
KIN_PID=$!

sleep 4
echo
echo "=== status ==="
ros2 topic list | grep -E "/camera/color|/arm6_joints" || true
ros2 service list | grep get_kinemarics || true
echo
echo "Camera log: $LOG_DIR/orbbec.log"
echo "FK log:     $LOG_DIR/kin_srv.log"
echo
echo "Now run:    python3 ~/project0/calibration/scripts/capture_handeye.py"
echo "Ctrl-C here to shut down camera + FK service."
wait "$ORBBEC_PID" "$KIN_PID"
