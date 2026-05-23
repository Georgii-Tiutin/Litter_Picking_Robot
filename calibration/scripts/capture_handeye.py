#!/usr/bin/env python3
"""Capture (arm pose, AprilTag pose) pairs for hand-eye calibration.

Runs on the robot. Subscribes to the Orbbec camera and the arm's
commanded-joint topic, runs dt_apriltags on each color frame, and on
Enter keypress saves a capture record.

Bringup needed in separate terminals (or via bringup_handeye.sh):
  source /opt/ros/humble/setup.bash
  source ~/yahboomcar_ws/install/setup.bash
  export ROS_DOMAIN_ID=30
  ros2 launch orbbec_camera dabai_dcw2.launch.py
  ros2 run arm_kin kin_srv

Then move the arm with the joystick to varied poses where the tag is
visible. Press Enter to capture. Aim for >= 15 captures with diverse
rotation and translation. Type 'd' Enter to delete the last capture,
'q' Enter to quit.
"""

import argparse
import json
import os
import queue
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from arm_interface.srv import ArmKinemarics
from arm_msgs.msg import ArmJoint, ArmJoints
from cv_bridge import CvBridge
from dt_apriltags import Detector
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

TAG_SIZE_M = 0.080
TAG_FAMILY = "tag36h11"
TARGET_TAG_ID = 0

# joy_M3Pro init pose (yahboom_joy_M3Pro.py:37): [j1..j5, gripper] in degrees.
# Used as a seed because the M3PRO arm has no servo position readback —
# joystick updates arrive as single-joint deltas on /arm_joint, so we
# accumulate state from this baseline.
INIT_JOINTS_DEG = [90, 40, 60, 20, 90, 90]

# Preset poses for hand-eye calibration. The 'n' command in the capture
# script advances through these and publishes each one to /arm6_joints.
# Anchor pose [90, 40, 60, 20, 90, 90] confirmed by user 2026-05-04 to
# point the eye-in-hand camera down at the tag from a sensible distance.
# All variations cluster around the anchor with deliberate rotation +
# translation diversity (required for AX=XB to be non-degenerate).
# Gripper held at 90 (mid-range, avoids servo stalls).
#
# Each row: [joint1, joint2, joint3, joint4, joint5, gripper]
# joint1 = base yaw         joint2 = shoulder pitch
# joint3 = elbow            joint4 = wrist pitch
# joint5 = wrist roll       joint6 = gripper
PRESET_POSES_DEG = [
    [ 90,  40,  60,  20,  90, 90],   # 0: anchor (verified)
    [ 90,  50,  50,  20,  90, 90],   # 1: closer  (j2 fold + j3 close)
    [ 90,  35,  65,  20,  90, 90],   # 2: farther
    [ 90,  40,  60,  35,  90, 90],   # 3: wrist tilt down +15
    [ 90,  40,  60,   8,  90, 90],   # 4: wrist tilt up -12
    [ 75,  40,  60,  20,  90, 90],   # 5: base yaw left -15
    [105,  40,  60,  20,  90, 90],   # 6: base yaw right +15
    [ 90,  40,  60,  20,  65, 90],   # 7: wrist roll left -25
    [ 90,  40,  60,  20, 115, 90],   # 8: wrist roll right +25
    [ 80,  50,  50,  20,  90, 90],   # 9: yaw-L + closer
    [100,  50,  50,  20,  90, 90],   # 10: yaw-R + closer
    [ 80,  35,  65,  20,  90, 90],   # 11: yaw-L + farther
    [100,  35,  65,  20,  90, 90],   # 12: yaw-R + farther
    [ 85,  45,  55,  28,  75, 90],   # 13: yaw-L + tilt + roll-L
    [ 95,  45,  55,  28, 105, 90],   # 14: yaw-R + tilt + roll-R
    [ 90,  45,  55,  30,  70, 90],   # 15: tilt-down + roll-L
    [ 90,  45,  55,  30, 110, 90],   # 16: tilt-down + roll-R
    # ---- high-diversity presets (added 2026-05-04 after first solve) ----
    # Goals: stretch tag-z range to >=150mm and pairwise rotation deltas to >=25 deg
    [ 90,  25,  80,  10,  90, 90],   # 17: arm extended FAR (camera high above tag)
    [ 90,  60,  35,  40,  90, 90],   # 18: arm tucked CLOSE (camera low over tag)
    [ 90,  40,  60,  20,  45, 90],   # 19: extreme roll left (-45)
    [ 90,  40,  60,  20, 135, 90],   # 20: extreme roll right (+45)
    [ 70,  35,  65,  20,  60, 90],   # 21: yaw far-left + farther + roll-L
    [110,  35,  65,  20, 120, 90],   # 22: yaw far-right + farther + roll-R
    [ 80,  55,  45,  35,  60, 90],   # 23: yaw-L + close + tilt + roll-L
    [100,  55,  45,  35, 120, 90],   # 24: yaw-R + close + tilt + roll-R
]

CAL_DIR = Path("/home/jetson/project0/calibration")
CAPTURE_DIR = CAL_DIR / "handeye_captures"
JSON_OUT = CAL_DIR / "handeye_captures.json"
PREVIEW_PATH = CAL_DIR / "preview.jpg"  # latest annotated frame, written every ~200 ms


class Capture(Node):
    def __init__(self, show_display=True):
        super().__init__("handeye_capture")
        self.bridge = CvBridge()
        self.detector = Detector(
            families=TAG_FAMILY,
            nthreads=2,
            quad_decimate=1.0,
            refine_edges=True,
        )
        self.K = None
        self.lock = threading.Lock()
        self.latest_tag = None
        self.latest_corners = None  # 4x2 pixel coords of detected tag, for overlay
        self.latest_image = None
        self.latest_image_stamp = None
        self.latest_joints = list(INIT_JOINTS_DEG)
        self.joints_dirty = False  # True once we've received any /arm_joint or /arm6_joints
        self.tag_seen_count = 0
        self.show_display = show_display
        self.last_preview_write = 0.0
        self.preview_window_open = False

        self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.cb_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.cb_image,
            qos_profile_sensor_data,
        )
        # Full-frame updates (rare — joy_M3Pro emits this on init / mode switch).
        self.create_subscription(
            ArmJoints, "/arm6_joints", self.cb_joints_full, 10
        )
        # Per-joint updates (common — every joystick servo button press).
        self.create_subscription(
            ArmJoint, "/arm_joint", self.cb_joint_single, 10
        )

        # Publisher to drive the arm via preset poses (bypasses joystick).
        self.arm_pub = self.create_publisher(ArmJoints, "/arm6_joints", 10)
        self.preset_index = -1  # -1 = haven't moved to any preset yet

        self.kin_client = self.create_client(ArmKinemarics, "get_kinemarics")
        if not self.kin_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "/get_kinemarics service not up. Start: ros2 run arm_kin kin_srv"
            )

        CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
        if JSON_OUT.exists():
            self.captures = json.loads(JSON_OUT.read_text())
        else:
            self.captures = []
        self.get_logger().info(
            f"Loaded {len(self.captures)} prior captures from {JSON_OUT}"
        )

    def cb_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f"camera_info: fx={self.K[0,0]:.2f} fy={self.K[1,1]:.2f} "
                f"cx={self.K[0,2]:.2f} cy={self.K[1,2]:.2f}"
            )

    def cb_image(self, msg):
        if self.K is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge: {e}")
            return
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cam_params = (self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2])
        try:
            dets = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=cam_params,
                tag_size=TAG_SIZE_M,
            )
        except Exception as e:
            self.get_logger().error(f"detector: {e}")
            return
        target = next((d for d in dets if d.tag_id == TARGET_TAG_ID), None)
        with self.lock:
            self.latest_image = img
            self.latest_image_stamp = msg.header.stamp
            if target is not None:
                T = np.eye(4)
                T[:3, :3] = np.array(target.pose_R)
                T[:3, 3] = np.array(target.pose_t).flatten()
                self.latest_tag = T
                self.latest_corners = np.array(target.corners)
                self.tag_seen_count += 1
            else:
                self.latest_tag = None
                self.latest_corners = None
            joints_snapshot = list(self.latest_joints)
            joints_dirty = self.joints_dirty
            n_caps = len(self.captures)
        self.update_preview(img, target, joints_snapshot, joints_dirty, n_caps)

    def update_preview(self, img, det, joints, joints_dirty, n_caps):
        out = img.copy()
        h, w = out.shape[:2]
        if det is not None:
            corners = np.array(det.corners).astype(int)
            cv2.polylines(out, [corners.reshape(-1, 1, 2)], True, (0, 230, 0), 2)
            # mark first corner with a red dot to show tag orientation
            cv2.circle(out, tuple(corners[0]), 6, (0, 0, 255), -1)
            # 3D axes from tag pose
            try:
                axis_len = TAG_SIZE_M * 0.5
                pts3d = np.float32([
                    [0, 0, 0], [axis_len, 0, 0],
                    [0, axis_len, 0], [0, 0, axis_len],
                ])
                rvec, _ = cv2.Rodrigues(np.array(det.pose_R))
                tvec = np.array(det.pose_t).reshape(3, 1)
                proj, _ = cv2.projectPoints(pts3d, rvec, tvec, self.K, np.zeros(5))
                proj = proj.reshape(-1, 2).astype(int)
                cv2.line(out, tuple(proj[0]), tuple(proj[1]), (0, 0, 255), 2)   # X red
                cv2.line(out, tuple(proj[0]), tuple(proj[2]), (0, 230, 0), 2)   # Y green
                cv2.line(out, tuple(proj[0]), tuple(proj[3]), (255, 100, 0), 2) # Z blue
            except cv2.error:
                pass
            z_mm = int(det.pose_t[2] * 1000)
            cv2.putText(out, f"DETECTED  id={det.tag_id}  z={z_mm} mm",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 230, 0), 2)
        else:
            cv2.putText(out, "NO TAG (reposition or move closer)",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 230), 2)
        cv2.putText(out, f"captures: {n_caps}",
                    (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        seed_tag = "" if joints_dirty else "  (SEED — wiggle joystick or pub /arm6_joints)"
        cv2.putText(out, f"joints={joints}{seed_tag}",
                    (10, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200) if joints_dirty else (0, 200, 230), 1)
        cv2.putText(out, "press ENTER in capture terminal to record",
                    (10, h - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (180, 180, 180), 1)

        if self.show_display:
            try:
                cv2.imshow("handeye_capture", out)
                cv2.waitKey(1)
                self.preview_window_open = True
            except cv2.error as e:
                if not self.preview_window_open:
                    self.get_logger().warn(
                        f"cv2.imshow failed ({e}); falling back to file-only "
                        f"preview at {PREVIEW_PATH}. Re-run with VNC or "
                        f"`ssh -X` for a live window."
                    )
                self.show_display = False
        now = time.monotonic()
        if now - self.last_preview_write > 0.2:
            try:
                cv2.imwrite(str(PREVIEW_PATH), out)
            except Exception:
                pass
            self.last_preview_write = now

    def cb_joints_full(self, msg):
        with self.lock:
            self.latest_joints = [
                int(msg.joint1),
                int(msg.joint2),
                int(msg.joint3),
                int(msg.joint4),
                int(msg.joint5),
                int(msg.joint6),
            ]
            self.joints_dirty = True
        self.get_logger().info(
            f"/arm6_joints -> joints={self.latest_joints}"
        )

    def cb_joint_single(self, msg):
        idx = int(msg.id) - 1  # Yahboom convention: servo IDs 1..6
        if idx < 0 or idx >= 6:
            return
        with self.lock:
            self.latest_joints[idx] = int(msg.joint)
            self.joints_dirty = True
            snapshot = list(self.latest_joints)
        self.get_logger().info(
            f"/arm_joint id={idx+1} -> {int(msg.joint)} | joints={snapshot}"
        )

    def call_fk(self, joints):
        req = ArmKinemarics.Request()
        req.kin_name = "fk"
        req.cur_joint1 = float(joints[0])
        req.cur_joint2 = float(joints[1])
        req.cur_joint3 = float(joints[2])
        req.cur_joint4 = float(joints[3])
        req.cur_joint5 = float(joints[4])
        req.cur_joint6 = float(joints[5])
        req.tar_x = req.tar_y = req.tar_z = 0.0
        req.roll = req.pitch = req.yaw = 0.0
        future = self.kin_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done() and future.result() is not None:
            r = future.result()
            return [r.x, r.y, r.z, r.roll, r.pitch, r.yaw]
        return None

    def attempt_capture(self):
        with self.lock:
            tag = self.latest_tag.copy() if self.latest_tag is not None else None
            joints = list(self.latest_joints)
            joints_dirty = self.joints_dirty
            img = self.latest_image.copy() if self.latest_image is not None else None
        if tag is None:
            self.get_logger().warn("No AprilTag visible right now — reposition.")
            return
        if not joints_dirty:
            self.get_logger().warn(
                f"No /arm_joint or /arm6_joints received yet. Using seed pose "
                f"{joints} which assumes the joy_M3Pro init pose. If the arm "
                f"is actually elsewhere, nudge any joint with the joystick "
                f"once before capturing."
            )
            return
        if img is None:
            self.get_logger().warn("No image received yet.")
            return
        fk = self.call_fk(joints)
        if fk is None:
            self.get_logger().warn(
                "FK service call failed. Is `ros2 run arm_kin kin_srv` running?"
            )
            return
        idx = len(self.captures)
        img_name = f"cap_{idx:03d}.png"
        cv2.imwrite(str(CAPTURE_DIR / img_name), img)
        rec = {
            "index": idx,
            "joints_deg": joints,
            "fk_base_to_endeffector": {
                "x": fk[0],
                "y": fk[1],
                "z": fk[2],
                "roll": fk[3],
                "pitch": fk[4],
                "yaw": fk[5],
            },
            "tag_in_camera_optical_4x4": tag.tolist(),
            "image": img_name,
            "tag_id": TARGET_TAG_ID,
            "tag_size_m": TAG_SIZE_M,
        }
        self.captures.append(rec)
        JSON_OUT.write_text(json.dumps(self.captures, indent=2))
        self.get_logger().info(
            f"Captured #{idx}  joints={joints}  "
            f"tag_z={tag[2,3]:.3f}m  total={len(self.captures)}"
        )

    def goto_pose(self, joints, settle_s=2.5, label=""):
        msg = ArmJoints()
        msg.joint1, msg.joint2, msg.joint3 = int(joints[0]), int(joints[1]), int(joints[2])
        msg.joint4, msg.joint5, msg.joint6 = int(joints[3]), int(joints[4]), int(joints[5])
        msg.time = 2000
        self.arm_pub.publish(msg)
        with self.lock:
            self.latest_joints = list(joints)
            self.joints_dirty = True
        self.get_logger().info(
            f"{label}published {list(joints)}; settling {settle_s:.1f}s"
        )
        time.sleep(settle_s)
        self.get_logger().info(f"{label}settled.")

    def goto_preset(self, idx, settle_s=2.5):
        if idx < 0 or idx >= len(PRESET_POSES_DEG):
            self.get_logger().warn(f"preset {idx} out of range (0..{len(PRESET_POSES_DEG)-1})")
            return
        self.goto_pose(PRESET_POSES_DEG[idx], settle_s,
                       label=f"[preset {idx}/{len(PRESET_POSES_DEG)-1}] ")
        self.preset_index = idx

    def next_preset(self):
        nxt = self.preset_index + 1
        if nxt >= len(PRESET_POSES_DEG):
            self.get_logger().info("Already at last preset.")
            return
        self.goto_preset(nxt)

    def prev_preset(self):
        prv = self.preset_index - 1
        if prv < 0:
            self.get_logger().info("Already at first preset.")
            return
        self.goto_preset(prv)

    def delete_last(self):
        if not self.captures:
            self.get_logger().warn("No captures to delete.")
            return
        last = self.captures.pop()
        JSON_OUT.write_text(json.dumps(self.captures, indent=2))
        img_path = CAPTURE_DIR / last.get("image", "")
        if img_path.exists():
            img_path.unlink()
        self.get_logger().info(
            f"Deleted #{last.get('index')}. Remaining: {len(self.captures)}."
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-display", action="store_true",
                    help="Skip cv2.imshow window; only write preview to "
                         f"{PREVIEW_PATH}")
    args = ap.parse_args()

    show_display = not args.no_display and bool(os.environ.get("DISPLAY"))
    if not show_display and not args.no_display:
        print(f"[capture] $DISPLAY not set — preview window disabled.\n"
              f"[capture] Live annotated preview written to {PREVIEW_PATH}.\n"
              f"[capture] To get a window: `ssh -X` from your Mac, or open "
              f"the robot via VNC and run this script there.\n")

    rclpy.init()
    node = Capture(show_display=show_display)
    cmd_q = queue.Queue()
    stop = threading.Event()

    def stdin_loop():
        print(
            "\n=== handeye capture ===\n"
            f"  n Enter            next preset (publishes /arm6_joints, 0..{len(PRESET_POSES_DEG)-1})\n"
            "  p Enter            previous preset\n"
            "  g N Enter          go to preset N\n"
            "  set j1 j2 j3 j4 j5 j6 Enter   move to ANY pose (degrees, 6 ints)\n"
            "  bump jN +/-D Enter  bump joint N (1..6) by D degrees\n"
            "  Enter              capture current pose + tag\n"
            "  s Enter            skip current preset (just advance)\n"
            "  d Enter            delete last capture\n"
            "  q Enter            quit\n",
            flush=True,
        )
        while not stop.is_set():
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                cmd_q.put(("quit", None))
                break
            cmd_raw = line.strip()
            cmd = cmd_raw.lower()
            if cmd in ("q", "d", "n", "p", "s"):
                cmd_q.put((cmd, None))
            elif cmd.startswith("g "):
                try:
                    cmd_q.put(("g", int(cmd.split()[1])))
                except (ValueError, IndexError):
                    print("usage: g N (where N is preset index)", flush=True)
            elif cmd.startswith("set "):
                try:
                    parts = cmd.split()[1:]
                    if len(parts) != 6:
                        raise ValueError("need 6 values")
                    cmd_q.put(("set", [int(x) for x in parts]))
                except ValueError as e:
                    print(f"usage: set j1 j2 j3 j4 j5 j6  ({e})", flush=True)
            elif cmd.startswith("bump "):
                try:
                    parts = cmd.split()[1:]
                    if len(parts) != 2 or not parts[0].lower().startswith("j"):
                        raise ValueError
                    j_idx = int(parts[0][1:]) - 1
                    delta = int(parts[1])
                    if j_idx < 0 or j_idx > 5:
                        raise ValueError
                    cmd_q.put(("bump", (j_idx, delta)))
                except ValueError:
                    print("usage: bump jN +/-D   e.g. bump j2 -10", flush=True)
            else:
                cmd_q.put(("capture", None))

    t = threading.Thread(target=stdin_loop, daemon=True)
    t.start()

    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                cmd, arg = cmd_q.get_nowait()
            except queue.Empty:
                continue
            if cmd in ("q", "quit"):
                stop.set()
                break
            if cmd == "d":
                node.delete_last()
            elif cmd in ("n", "s"):
                node.next_preset()
            elif cmd == "p":
                node.prev_preset()
            elif cmd == "g":
                node.goto_preset(arg)
            elif cmd == "set":
                node.goto_pose(arg, label="[set] ")
            elif cmd == "bump":
                j_idx, delta = arg
                with node.lock:
                    new_pose = list(node.latest_joints)
                new_pose[j_idx] += delta
                node.goto_pose(new_pose, label=f"[bump j{j_idx+1} {delta:+d}] ")
            else:
                node.attempt_capture()
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
