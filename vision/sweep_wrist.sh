#!/bin/bash
# Find the wrist angle that presents the gripper tag squarely to the overhead camera.
# Corner tags sit near the frame edges; the gripper tag is whichever detection is far from them.
for J4 in 0 20 40 60 80 100; do
  ssh robot "export ROS_DOMAIN_ID=30; source /opt/ros/humble/setup.bash; \
    source ~/yahboomcar_ws/install/setup.bash; \
    python3 ~/calib/set_joints.py 90 120 0 $J4 90 142 1500" >/dev/null 2>&1
  sleep 1
  ./capture.sh "sweep_j4_${J4}.jpg" >/dev/null
  echo -n "joint4=$J4  "
  ./.venv/bin/python - "sweep_j4_${J4}.jpg" <<'PY'
import sys, numpy as np, cv2
from pupil_apriltags import Detector
CORNERS = [(1182,45),(74,52),(76,651),(1185,656)]
g = cv2.cvtColor(cv2.imread(sys.argv[1]), cv2.COLOR_BGR2GRAY)
tags = Detector(families="tag36h11", nthreads=4, quad_decimate=1.0, refine_edges=1).detect(g)
best = None
for t in tags:
    if min(np.hypot(t.center[0]-cx, t.center[1]-cy) for cx, cy in CORNERS) > 120:
        best = t
if best is None:
    print("gripper tag NOT visible")
else:
    c = best.corners
    s = [float(np.linalg.norm(c[i]-c[(i+1)%4])) for i in range(4)]
    print(f"centre=({best.center[0]:6.1f},{best.center[1]:6.1f})  "
          f"sides {min(s):4.1f}-{max(s):4.1f}  squareness {min(s)/max(s):.2f}  "
          f"mean {np.mean(s):4.1f}px  margin {best.decision_margin:.0f}")
PY
done
