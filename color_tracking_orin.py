#!/usr/bin/env python3
"""
Color Tracking for ROSMASTER M3PRO on Jetson Orin.
Adapted from Raspberry Pi / Yahboom Raspbot version.

Uses ROS2 topics for camera input, servo control, and RGB LED feedback.
Requires: ros2 launch orbbec_camera dabai_dcw2.launch.py
          sh start_agent.sh
"""

import time
import math
import threading

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from arm_msgs.msg import ArmJoint

# ---------------------------------------------------------------------------
# Inline Positional PID (replaces Raspberry Pi PID module)
# ---------------------------------------------------------------------------
class PositionalPID:
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.SystemOutput = 0.0
        self._step_signal = 0.0
        self._err = 0.0
        self._err_last = 0.0
        self._err_sum = 0.0
        self._inertia_time = 0.0
        self._sample_time = 0.0

    def SetStepSignal(self, value):
        self._step_signal = value

    def SetInertiaTime(self, sample_time, inertia_time):
        self._sample_time = sample_time
        self._inertia_time = inertia_time
        self._err = self._step_signal - self.SystemOutput
        self._err_sum += self._err
        output = (self.Kp * self._err
                  + self.Ki * self._err_sum
                  + self.Kd * (self._err - self._err_last))
        self._err_last = self._err
        self.SystemOutput = output


# ---------------------------------------------------------------------------
# ROS2 Color Tracking Node
# ---------------------------------------------------------------------------
class ColorTrackingNode(Node):
    # Default HSV thresholds (red)
    COLOR_PRESETS = {
        'red':    (np.array([0,  43,  89]), np.array([7,   255, 255])),
        'green':  (np.array([54, 104, 64]), np.array([78,  255, 255])),
        'blue':   (np.array([92,  80, 60]), np.array([124, 255, 255])),
        'yellow': (np.array([26, 100, 91]), np.array([32,  255, 255])),
        'orange': (np.array([11,  43, 46]), np.array([25,  255, 255])),
    }

    RGB_VALUES = {
        'red':    (1.0, 0.0, 0.0),
        'green':  (0.0, 1.0, 0.0),
        'blue':   (0.0, 0.0, 1.0),
        'yellow': (1.0, 1.0, 0.0),
        'orange': (1.0, 0.19, 0.0),
    }

    # Servo IDs on M3PRO arm
    PAN_SERVO_ID = 1   # Joint 1: base rotation (horizontal)
    TILT_SERVO_ID = 4  # Joint 4: wrist pitch (vertical)
    SERVO_TIME_MS = 100  # Movement duration per command (fast tracking)

    # Servo limits (degrees)
    PAN_MIN = 0
    PAN_MAX = 180
    TILT_MIN = 0
    TILT_MAX = 110

    # Image dimensions (expected from Orbbec camera)
    IMG_W = 640
    IMG_H = 480

    # GUI button panel
    BUTTON_H = 60
    BUTTON_COLORS = [
        ('red',    (0, 0, 255)),
        ('green',  (0, 200, 0)),
        ('blue',   (255, 0, 0)),
        ('yellow', (0, 255, 255)),
        ('orange', (0, 120, 255)),
        ('stop',   (128, 128, 128)),
    ]

    def __init__(self):
        super().__init__('color_tracking')

        # --- Publishers ---
        self.pub_servo = self.create_publisher(ArmJoint, '/arm_joint', 1)
        self.pub_rgb = self.create_publisher(ColorRGBA, '/rgb', 1)

        # --- Subscriber (camera) ---
        self.bridge = CvBridge()
        self.frame = None
        self.sub_image = self.create_subscription(
            Image, '/camera/color/image_raw', self._image_callback, 1)

        # --- State ---
        self.color_lower = self.COLOR_PRESETS['red'][0]
        self.color_upper = self.COLOR_PRESETS['red'][1]
        self.tracking = False
        self.active_color = None

        self.color_x = 0.0
        self.color_y = 0.0
        self.color_radius = 0.0

        # Current servo positions (start centered)
        self.pan_angle = 90.0
        self.tilt_angle = 55.0  # Midpoint of 0–110

        # --- PID controllers ---
        self.xservo_pid = PositionalPID(0.8, 0.2, 0.01)
        self.yservo_pid = PositionalPID(0.8, 0.2, 0.01)

        # --- FPS tracking ---
        self.t_start = time.time()
        self.frame_count = 0

        # Send servos to initial position
        self._send_servo(self.PAN_SERVO_ID, int(self.pan_angle))
        self._send_servo(self.TILT_SERVO_ID, int(self.tilt_angle))

        # --- GUI setup ---
        self._win_name = 'Color Tracking'
        cv2.namedWindow(self._win_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self._win_name, self._on_mouse)

        # Precompute button regions
        n = len(self.BUTTON_COLORS)
        bw = self.IMG_W // n
        self._buttons = []
        for i, (name, bgr) in enumerate(self.BUTTON_COLORS):
            x1 = i * bw
            x2 = x1 + bw if i < n - 1 else self.IMG_W
            self._buttons.append((name, bgr, x1, x2))

        # Processing timer (30 Hz)
        self.create_timer(1.0 / 30.0, self._process)

        self.get_logger().info('Color tracking node started. Waiting for camera...')

    # ------------------------------------------------------------------
    # Mouse callback for GUI buttons
    # ------------------------------------------------------------------
    def _on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        # Check if click is in the button panel area (below camera frame)
        if y < self.IMG_H:
            return
        for name, _bgr, x1, x2 in self._buttons:
            if x1 <= x < x2:
                if name == 'stop':
                    self.select_color('none')
                else:
                    self.select_color(name)
                break

    # ------------------------------------------------------------------
    # Draw button panel onto canvas
    # ------------------------------------------------------------------
    def _draw_buttons(self, canvas):
        y_top = self.IMG_H
        for name, bgr, x1, x2 in self._buttons:
            # Fill button
            cv2.rectangle(canvas, (x1, y_top), (x2, y_top + self.BUTTON_H), bgr, -1)
            # Highlight active button
            if name == self.active_color:
                cv2.rectangle(canvas, (x1 + 2, y_top + 2),
                              (x2 - 2, y_top + self.BUTTON_H - 2), (255, 255, 255), 3)
            # Label
            label = name.upper()
            sz = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            tx = x1 + (x2 - x1 - sz[0]) // 2
            ty = y_top + (self.BUTTON_H + sz[1]) // 2
            cv2.putText(canvas, label, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # ------------------------------------------------------------------
    # Camera callback
    # ------------------------------------------------------------------
    def _image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # ------------------------------------------------------------------
    # Servo helper
    # ------------------------------------------------------------------
    def _send_servo(self, servo_id, angle):
        msg = ArmJoint()
        msg.id = servo_id
        msg.joint = int(angle)
        msg.time = self.SERVO_TIME_MS
        self.pub_servo.publish(msg)

    # ------------------------------------------------------------------
    # RGB helper
    # ------------------------------------------------------------------
    def _send_rgb(self, r, g, b):
        msg = ColorRGBA()
        msg.r = float(r)
        msg.g = float(g)
        msg.b = float(b)
        msg.a = 1.0
        self.pub_rgb.publish(msg)

    # ------------------------------------------------------------------
    # Select tracking color
    # ------------------------------------------------------------------
    def select_color(self, color_name):
        """Switch tracked color. Valid: red, green, blue, yellow, orange, none."""
        if color_name == 'none':
            self.tracking = False
            self.active_color = None
            self._send_rgb(0.0, 0.0, 0.0)
            # Reset servos to center
            self.pan_angle = 90.0
            self.tilt_angle = 55.0
            self._send_servo(self.PAN_SERVO_ID, int(self.pan_angle))
            self._send_servo(self.TILT_SERVO_ID, int(self.tilt_angle))
            self.get_logger().info('Tracking stopped.')
            return

        if color_name not in self.COLOR_PRESETS:
            self.get_logger().warn(f'Unknown color: {color_name}')
            return

        self.color_lower, self.color_upper = self.COLOR_PRESETS[color_name]
        r, g, b = self.RGB_VALUES[color_name]
        self._send_rgb(r, g, b)
        self.tracking = True
        self.active_color = color_name
        self.get_logger().info(f'Tracking color: {color_name}')

    # ------------------------------------------------------------------
    # Main processing loop (called by timer)
    # ------------------------------------------------------------------
    def _process(self):
        if self.frame is None:
            return

        frame = self.frame.copy()

        if self.tracking:
            self._track_color(frame)

        # FPS overlay
        self.frame_count += 1
        elapsed = time.time() - self.t_start
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        cv2.putText(frame, f'FPS {int(fps)}', (40, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # Build canvas: camera frame + button panel
        canvas = np.zeros((self.IMG_H + self.BUTTON_H, self.IMG_W, 3), dtype=np.uint8)
        canvas[:self.IMG_H, :, :] = frame
        self._draw_buttons(canvas)

        cv2.imshow(self._win_name, canvas)
        key = cv2.waitKey(1) & 0xFF

        # Keyboard color selection (still works)
        if key == ord('r'):
            self.select_color('red')
        elif key == ord('g'):
            self.select_color('green')
        elif key == ord('b'):
            self.select_color('blue')
        elif key == ord('y'):
            self.select_color('yellow')
        elif key == ord('o'):
            self.select_color('orange')
        elif key == ord('c') or key == 27:  # 'c' or ESC
            self.select_color('none')
        elif key == ord('q'):
            self.get_logger().info('Quit requested.')
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # Color tracking logic (adapted from Color_Recognize2 with deadzone)
    # ------------------------------------------------------------------
    def _track_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_lower, self.color_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        if len(cnts) == 0:
            return

        cnt = max(cnts, key=cv2.contourArea)
        (self.color_x, self.color_y), self.color_radius = cv2.minEnclosingCircle(cnt)

        if self.color_radius < 10:
            return

        # Draw detection circle
        cv2.circle(frame, (int(self.color_x), int(self.color_y)),
                   int(self.color_radius), (255, 0, 255), 2)

        # Image center
        cx = self.IMG_W / 2.0  # 320
        cy = self.IMG_H / 2.0  # 240

        # --- Pan (X axis) with deadzone ---
        error_x = self.color_x - cx
        if math.fabs(error_x) > 20:
            self.xservo_pid.SystemOutput = error_x
            self.xservo_pid.SetStepSignal(0)
            self.xservo_pid.SetInertiaTime(0.01, 0.05)

            # Negative because: object is right of center → servo should pan right
            # (decreasing angle on M3PRO joint 1 rotates right)
            self.pan_angle -= self.xservo_pid.SystemOutput * 0.05
            self.pan_angle = max(self.PAN_MIN, min(self.PAN_MAX, self.pan_angle))
            self._send_servo(self.PAN_SERVO_ID, int(self.pan_angle))

        # --- Tilt (Y axis) with deadzone ---
        error_y = self.color_y - cy
        if math.fabs(error_y) > 20:
            self.yservo_pid.SystemOutput = error_y
            self.yservo_pid.SetStepSignal(0)
            self.yservo_pid.SetInertiaTime(0.01, 0.1)

            # Positive error_y means object is below center → tilt down
            self.tilt_angle += self.yservo_pid.SystemOutput * 0.03
            self.tilt_angle = max(self.TILT_MIN, min(self.TILT_MAX, self.tilt_angle))
            self._send_servo(self.TILT_SERVO_ID, int(self.tilt_angle))

        # Debug overlay
        cv2.putText(frame, f'x:{int(self.color_x)} pan:{int(self.pan_angle)}',
                    (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f'y:{int(self.color_y)} tilt:{int(self.tilt_angle)}',
                    (40, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = ColorTrackingNode()

    print('\n--- Color Tracking (Jetson Orin / M3PRO) ---')
    print('Keys: r=red  g=green  b=blue  y=yellow  o=orange  c=close  q=quit')
    print('Prerequisites:')
    print('  sh start_agent.sh')
    print('  ros2 launch orbbec_camera dabai_dcw2.launch.py\n')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Reset servos and turn off LEDs
        node.select_color('none')
        time.sleep(0.2)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
