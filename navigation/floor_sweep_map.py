#!/usr/bin/env python3
"""
floor_sweep_map.py — Sweep the M3PRO arm left-right and stitch the eye-in-hand depth
camera into a floor point-cloud map, published as /floor_map (base_link) for RViz.

Runs ON THE ROBOT. The arm is open-loop (no joint feedback), so each cloud is placed
using the FK of the COMMANDED joints, exactly the convention capture_handeye.py used
(publish joints directly to /arm6_joints, FK the same values — NO joint1 inversion),
which is the convention the hand-eye calibration was solved under.

Transform per point:  p_base = T_base_Gripping(FK) @ T_Gripping_camOptical(hand-eye) @ p_cam
Cloud frame is camera_color_optical_frame == the hand-eye child frame (direct apply).

Prereqs on robot (all headless):
  - camera:  ros2 launch orbbec_camera dabai_dcw2.launch.py   (/camera/depth/points)
  - FK srv:  ros2 run arm_kin kin_srv                          (/get_kinemarics)
  - chassis/arm micro-ROS agent running (/arm6_joints -> servos)

Example:
  python3 floor_sweep_map.py --handeye hand_eye_endeffector_to_camera.yaml \
      --joint1 70 80 90 100 110 --settle 3.0 --voxel 0.01
"""
import argparse
import math
import time
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from arm_msgs.msg import ArmJoints
from arm_interface.srv import ArmKinemarics
from sensor_msgs.msg import JointState

ANCHOR = [90, 40, 60, 20, 90, 90]  # [j1, j2, j3, j4, j5, gripper] — camera-down observe pose

# URDF revolute joint names for the 5 arm servos (arm4 has the vendor 'Joiint' typo).
ARM_JOINT_NAMES = ["arm1_Joint", "arm2_Joint", "arm3_Joint", "arm4_Joiint", "arm5_Joint"]
# kin_srv maps servo degrees -> URDF radians as (deg - 90) * pi/180 (90 deg = neutral).
DEG2URDF = math.pi / 180.0


def rpy_to_mat(x, y, z, roll, pitch, yaw):
    """ROS RPY (R = Rz(yaw) @ Ry(pitch) @ Rx(roll)) + translation -> 4x4."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry @ Rx
    T[:3, 3] = [x, y, z]
    return T


class FloorSweep(Node):
    def __init__(self, args):
        super().__init__("floor_sweep_map")
        self.args = args

        with open(args.handeye) as f:
            he = yaml.safe_load(f)
        R = np.array(he["rotation_matrix"], dtype=np.float64)
        t = he["translation"]
        self.T_grip_cam = np.eye(4)
        self.T_grip_cam[:3, :3] = R
        self.T_grip_cam[:3, 3] = [t["x"], t["y"], t["z"]]
        self.get_logger().info(f"hand-eye {he['parent_frame']}->{he['child_frame']} loaded")

        self.latest_cloud = None
        self.create_subscription(
            PointCloud2, "/camera/depth/points", self._cloud_cb, qos_profile_sensor_data
        )
        self.arm_pub = self.create_publisher(ArmJoints, "/arm6_joints", 10)
        self.fk = self.create_client(ArmKinemarics, "get_kinemarics")

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # latched for RViz
        map_qos.history = HistoryPolicy.KEEP_LAST
        self.map_pub = self.create_publisher(PointCloud2, "/floor_map", map_qos)

        # Publish /joint_states so RViz's RobotModel arm matches the real (commanded) pose.
        # Kill the dummy joint_state_publisher before running, else it fights this at 0.
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.cmd_joints = list(ANCHOR)  # last commanded servo angles
        self.create_timer(0.1, self._pub_joint_states)  # 10 Hz

        self.accum = np.empty((0, 3), dtype=np.float32)

    def _pub_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ARM_JOINT_NAMES
        js.position = [(self.cmd_joints[i] - 90) * DEG2URDF for i in range(5)]
        self.js_pub.publish(js)

    def _cloud_cb(self, msg):
        self.latest_cloud = msg

    def _spin(self, secs):
        end = self.get_clock().now().nanoseconds + int(secs * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_arm(self, joints, settle):
        msg = ArmJoints()
        msg.joint1, msg.joint2, msg.joint3 = int(joints[0]), int(joints[1]), int(joints[2])
        msg.joint4, msg.joint5, msg.joint6 = int(joints[3]), int(joints[4]), int(joints[5])
        msg.time = int(self.args.movetime)
        self.arm_pub.publish(msg)
        self.cmd_joints = list(joints)  # RViz arm (via /joint_states timer) follows the real arm
        self.get_logger().info(f"moved arm -> {joints}, settling {settle:.1f}s")
        self._spin(settle)

    def call_fk(self, joints):
        req = ArmKinemarics.Request()
        req.cur_joint1, req.cur_joint2, req.cur_joint3 = float(joints[0]), float(joints[1]), float(joints[2])
        req.cur_joint4, req.cur_joint5, req.cur_joint6 = float(joints[3]), float(joints[4]), float(joints[5])
        req.kin_name = "fk"
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if r is None:
            raise RuntimeError("FK call failed")
        return rpy_to_mat(r.x, r.y, r.z, r.roll, r.pitch, r.yaw)

    def grab_cloud(self, timeout=4.0):
        self.latest_cloud = None
        end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.latest_cloud is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.latest_cloud

    def stitch(self, joints):
        msg = self.grab_cloud()
        if msg is None:
            self.get_logger().warn("no cloud captured, skipping pose")
            return
        data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.column_stack([data["x"], data["y"], data["z"]]).astype(np.float64)
        # filter by optical depth (z forward) — drop too-close (unreliable) / too-far
        zc = pts[:, 2]
        keep = (zc > self.args.zmin) & (zc < self.args.zmax)
        pts = pts[keep]
        if pts.shape[0] == 0:
            self.get_logger().warn("no points in depth window")
            return
        T_base_grip = self.call_fk(joints)
        T = T_base_grip @ self.T_grip_cam
        pb = (T[:3, :3] @ pts.T).T + T[:3, 3]  # base_link points
        # optional base_link height gate (floor band) to reject far walls/ceiling
        if self.args.zband is not None:
            m = (pb[:, 2] > -self.args.zband) & (pb[:, 2] < self.args.zband)
            pb = pb[m]
        self.accum = np.vstack([self.accum, pb.astype(np.float32)])
        self.accum = self._voxel(self.accum, self.args.voxel)
        self.get_logger().info(
            f"pose j1={joints[0]}: +{pb.shape[0]} pts -> {self.accum.shape[0]} total"
        )
        self.publish_map()

    @staticmethod
    def _voxel(pts, v):
        if v <= 0 or pts.shape[0] == 0:
            return pts
        keys = np.floor(pts / v).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[np.sort(idx)]

    def publish_map(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        cloud = pc2.create_cloud_xyz32(header, self.accum.tolist())
        self.map_pub.publish(cloud)

    def run(self):
        self.get_logger().info("waiting for FK service + first cloud...")
        self.fk.wait_for_service(timeout_sec=10.0)
        if self.grab_cloud(timeout=6.0) is None:
            self.get_logger().error("no depth cloud — is the camera up? aborting.")
            return
        # center first (gentle), then sweep
        seq = self.args.joint1
        self.move_arm([90] + ANCHOR[1:], self.args.settle)
        for j1 in seq:
            joints = [j1] + ANCHOR[1:]
            self.move_arm(joints, self.args.settle)
            self.stitch(joints)
        # return to center, keep map latched
        self.move_arm(ANCHOR, self.args.settle)
        # save the accumulated cloud + a completion marker (robust to flaky SSH)
        if self.args.save:
            np.save(self.args.save, self.accum)
            self.get_logger().info(f"saved cloud -> {self.args.save}.npy")
        with open(self.args.marker, "w") as f:
            f.write(f"done points={self.accum.shape[0]}\n")
        self.get_logger().info(f"sweep done: {self.accum.shape[0]} points on /floor_map. Ctrl-C to exit.")
        while rclpy.ok():
            self.publish_map()
            self._spin(2.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--handeye", required=True)
    ap.add_argument("--joint1", type=int, nargs="+", default=[70, 80, 90, 100, 110],
                    help="joint1 sweep angles (deg); anchor j2-5 held at 40,60,20,90")
    ap.add_argument("--settle", type=float, default=3.0, help="seconds to wait after each move")
    ap.add_argument("--movetime", type=int, default=1500, help="arm move duration ms")
    ap.add_argument("--zmin", type=float, default=0.12, help="min optical depth (m)")
    ap.add_argument("--zmax", type=float, default=1.5, help="max optical depth (m)")
    ap.add_argument("--zband", type=float, default=None,
                    help="if set, keep base_link |z|<band (m) to isolate the floor plane")
    ap.add_argument("--voxel", type=float, default=0.01, help="voxel downsample size (m)")
    ap.add_argument("--save", default=None, help="np.save the accumulated Nx3 cloud to this path (.npy)")
    ap.add_argument("--marker", default="/tmp/sweep_done.txt", help="completion marker file")
    args = ap.parse_args()

    rclpy.init()
    node = FloorSweep(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
