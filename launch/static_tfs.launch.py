"""
project0 static transforms.

Publishes the calibrated eye-in-hand transform from hand-eye calibration
(Phase 0.6.2). This transform expresses the camera optical frame in the
arm's end-effector frame as solved by cv2.calibrateHandEye on 27 captures
with the PARK method.

Key facts:
- Source yaml: ~/project0/calibration/hand_eye_endeffector_to_camera.yaml
- Method: PARK   n_captures: 27
- AX=XB residual median: rot=13.7 deg, trans=82.5 mm
  (limited by open-loop joints + 5DOF + URDF tolerances; see
   calibration/README.md "Phase 0.6.2 / Residuals" section.)

Frame topology:
  Gripping   (URDF, fixed child of arm5; the FK terminus arm_kin uses)
       |
       +--> camera_optical_calib   (this static TF)

We deliberately use a DISTINCT frame name (camera_optical_calib) instead
of the driver's camera_color_optical_frame, because the orbbec_camera
driver already publishes a TF to camera_color_optical_frame from
camera_link, and TF allows only one parent per frame.

Perception code (Phase 2 onward) should:
1. Get cube pose in camera_color_optical_frame (from the driver's frame).
2. Look up TF from camera_optical_calib -> camera_color_optical_frame.
   Both are physically the same point but tagged with different frame
   ids; for the calibration to apply, treat them as identical (they
   only differ because of TF-tree-topology constraints).
3. Equivalently and more directly: read
   hand_eye_endeffector_to_camera.yaml, apply the 4x4 transform manually,
   compose with live base_link->Gripping TF lookup.

Caveat (Phase 0.6.2 limitation): the camera is physically mounted on
arm4, but FK measured the chain end at arm5/Gripping. A non-zero
joint5 (arm5_Joint) angle introduces a small joint5-dependent error
into the calibration (estimated <=35 mm at joint5 = 60 or 120 vs the
calibration mean joint5=90). For best accuracy in perception, hold
joint5 near 90 deg when localising the cube.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Calibrated values from hand_eye_endeffector_to_camera.yaml
    # (PARK, n=27, 2026-05-04)
    gripping_to_camera_calib = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gripping_to_camera_optical_calib",
        arguments=[
            "--x", "-0.106538",
            "--y", "-0.015415",
            "--z", "-0.097122",
            "--roll",  "1.326384",
            "--pitch", "-0.208860",
            "--yaw",   "-1.343833",
            "--frame-id", "Gripping",
            "--child-frame-id", "camera_optical_calib",
        ],
    )

    return LaunchDescription([gripping_to_camera_calib])
