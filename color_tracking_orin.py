#!/usr/bin/env python3
"""
Color Tracking for ROSMASTER M3PRO on Jetson Orin NX.
Adapted from Yahboom Raspbot Jupyter notebook version.

Uses ROS2 topics for camera input and servo control.
OpenCV GUI with clickable color buttons replaces Jupyter widgets.

Prerequisites:
  - Micro-ROS agent running (sh ~/start_agent.sh)
  - Orbbec camera running (ros2 launch orbbec_camera dabai_dcw2.launch.py)
"""

import time
import math

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from arm_msgs.msg import ArmJoint


# ---------------------------------------------------------------------------
# Positional PID (replaces Raspberry Pi PID module)
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

    def SetStepSignal(self, value):
        self._step_signal = value

    def SetInertiaTime(self, sample_time, inertia_time):
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

    # HSV thresholds (same as Pi version)
    COLOR_PRESETS = {
        'red':    (np.array([0,   43,  89]), np.array([7,   255, 255])),
        'green':  (np.array([54, 104,  64]), np.array([78,  255, 255])),
        'blue':   (np.array([92,  80,  60]), np.array([124, 255, 255])),
        'yellow': (np.array([26, 100,  91]), np.array([32,  255, 255])),
        'orange': (np.array([11,  43,  46]), np.array([25,  255, 255])),
    }

    # RGB LED values (for /rgb topic — requires custom firmware subscriber)
    RGB_VALUES = {
        'red':    (1.0, 0.0, 0.0),
        'green':  (0.0, 1.0, 0.0),
        'blue':   (0.0, 0.0, 1.0),
        'yellow': (1.0, 1.0, 0.0),
        'orange': (1.0, 0.19, 0.0),
    }

    # --- M3PRO servo mapping ---
    # Pi: bot.Ctrl_Servo(1, angle) → Joint 1 (pan)
    # Pi: bot.Ctrl_Servo(2, angle) → Joint 4 (tilt) on M3PRO
    PAN_SERVO_ID = 1
    TILT_SERVO_ID = 4
    SERVO_TIME_MS = 100

    PAN_MIN = 0
    PAN_MAX = 180
    PAN_CENTER = 90

    TILT_MIN = 0
    TILT_MAX = 100   # Pi used 100 as max in Color_Recognize2
    TILT_CENTER = 55

    # Image dimensions
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
        ('close',  (128, 128, 128)),
    ]

    def __init__(self):
        super().__init__('color_tracking')

        # --- Publishers ---
        self.pub_servo = self.create_publisher(ArmJoint, '/arm_joint', 1)
        self.pub_rgb = self.create_publisher(ColorRGBA, '/rgb', 1)

        # --- Camera subscriber ---
        self.bridge = CvBridge()
        self.frame = None
        self.sub_image = self.create_subscription(
            Image, '/camera/color/image_raw', self._image_callback, 1)

        # --- Tracking state ---
        # Pi globals: g_mode, color_lower, color_upper, color_x, color_y, etc.
        self.g_mode = 0  # 0 = off, 1 = tracking
        self.active_color = None
        self.color_lower = self.COLOR_PRESETS['red'][0]
        self.color_upper = self.COLOR_PRESETS['red'][1]

        self.color_x = 0.0
        self.color_y = 0.0
        self.color_radius = 0.0

        # Pi globals: target_valuex, target_valuey (PWM-style, 500-2500 range)
        self.target_valuex = 1500
        self.target_valuey = 1500

        # --- PID controllers (same gains as Pi) ---
        self.xservo_pid = PositionalPID(0.8, 0.2, 0.01)
        self.yservo_pid = PositionalPID(0.8, 0.2, 0.01)

        # --- FPS tracking ---
        self.t_start = time.time()
        self.frame_count = 0

        # --- Send servos to center (Pi: bot.Ctrl_Servo(1,90), bot.Ctrl_Servo(2,25)) ---
        self._send_servo(self.PAN_SERVO_ID, self.PAN_CENTER)
        self._send_servo(self.TILT_SERVO_ID, self.TILT_CENTER)

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
        if y < self.IMG_H:
            return
        for name, _bgr, x1, x2 in self._buttons:
            if x1 <= x < x2:
                if name == 'close':
                    self._on_close()
                else:
                    self._on_color_selected(name)
                break

    # ------------------------------------------------------------------
    # Button handlers (adapted from Pi on_*button_clicked callbacks)
    # ------------------------------------------------------------------
    def _on_color_selected(self, color_name):
        """Equivalent to Pi's on_Redbutton_clicked, on_Greenbutton_clicked, etc."""
        self.color_lower, self.color_upper = self.COLOR_PRESETS[color_name]
        self.g_mode = 1
        self.active_color = color_name

        # RGB LED feedback (Pi: bot.Ctrl_WQ2812_ALL(...))
        r, g, b = self.RGB_VALUES[color_name]
        self._send_rgb(r, g, b)

        self.get_logger().info(f'Tracking color: {color_name}')

    def _on_close(self):
        """Equivalent to Pi's on_Closebutton_clicked."""
        self.g_mode = 0
        self.active_color = None

        # Turn off LEDs (Pi: bot.Ctrl_WQ2812_ALL(0, 0))
        self._send_rgb(0.0, 0.0, 0.0)

        # Reset servos to center (Pi: bot.Ctrl_Servo(1, 90), bot.Ctrl_Servo(2, 25))
        self._send_servo(self.PAN_SERVO_ID, self.PAN_CENTER)
        self._send_servo(self.TILT_SERVO_ID, self.TILT_CENTER)

        self.get_logger().info('Tracking stopped.')

    # ------------------------------------------------------------------
    # Draw button panel
    # ------------------------------------------------------------------
    def _draw_buttons(self, canvas):
        y_top = self.IMG_H
        for name, bgr, x1, x2 in self._buttons:
            cv2.rectangle(canvas, (x1, y_top), (x2, y_top + self.BUTTON_H), bgr, -1)
            # Highlight active
            if name == self.active_color:
                cv2.rectangle(canvas, (x1 + 2, y_top + 2),
                              (x2 - 2, y_top + self.BUTTON_H - 2),
                              (255, 255, 255), 3)
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
    # Servo helper (replaces Pi's bot.Ctrl_Servo)
    # ------------------------------------------------------------------
    def _send_servo(self, servo_id, angle):
        msg = ArmJoint()
        msg.id = servo_id
        msg.joint = int(angle)
        msg.time = self.SERVO_TIME_MS
        self.pub_servo.publish(msg)

    # ------------------------------------------------------------------
    # RGB helper (replaces Pi's bot.Ctrl_WQ2812_ALL)
    # ------------------------------------------------------------------
    def _send_rgb(self, r, g, b):
        msg = ColorRGBA()
        msg.r = float(r)
        msg.g = float(g)
        msg.b = float(b)
        msg.a = 1.0
        self.pub_rgb.publish(msg)

    # ------------------------------------------------------------------
    # Main processing loop (30 Hz timer, replaces Pi's while True loop)
    # ------------------------------------------------------------------
    def _process(self):
        if self.frame is None:
            return

        frame = self.frame.copy()

        if self.g_mode == 1:
            self._track_color(frame)

        # FPS overlay
        self.frame_count += 1
        elapsed = time.time() - self.t_start
        fps = self.frame_count / elapsed if elapsed > 0 else 0
        cv2.putText(frame, f'FPS {int(fps)}', (40, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 3)

        # Build canvas with button panel
        canvas = np.zeros((self.IMG_H + self.BUTTON_H, self.IMG_W, 3), dtype=np.uint8)
        canvas[:self.IMG_H, :, :] = frame
        self._draw_buttons(canvas)

        cv2.imshow(self._win_name, canvas)
        key = cv2.waitKey(1) & 0xFF

        # Keyboard shortcuts (same as Pi key concept)
        if key == ord('r'):
            self._on_color_selected('red')
        elif key == ord('g'):
            self._on_color_selected('green')
        elif key == ord('b'):
            self._on_color_selected('blue')
        elif key == ord('y'):
            self._on_color_selected('yellow')
        elif key == ord('o'):
            self._on_color_selected('orange')
        elif key == ord('c') or key == 27:
            self._on_close()
        elif key == ord('q'):
            self.get_logger().info('Quit requested.')
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # Color tracking (ported from Pi's Color_Recognize2 with deadzone)
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

        # --- X axis PID with deadzone ---
        # Pi: SetStepSignal(250) with 640px image → target pixel ~250
        # Pi: target_valuex = 1600 + PID_output → servo = (target_valuex - 500) / 10
        # Adapted: use image center (320) as step signal
        if math.fabs(self.IMG_W / 2.0 - self.color_x) > 20:
            self.xservo_pid.SystemOutput = self.color_x
            self.xservo_pid.SetStepSignal(self.IMG_W / 2.0)
            self.xservo_pid.SetInertiaTime(0.01, 0.05)

            # Pi formula: servo = (1600 + pid_output - 500) / 10 = 110 + pid_output/10
            # For M3PRO: map similarly, centering around PAN_CENTER (90)
            target_valuex = int(1400 + self.xservo_pid.SystemOutput)
            target_servox = int((target_valuex - 500) / 10)
            target_servox = max(self.PAN_MIN, min(self.PAN_MAX, target_servox))

            self._send_servo(self.PAN_SERVO_ID, target_servox)

        # --- Y axis PID with deadzone ---
        # Pi: SetStepSignal(200), target_valuey = 1150 + PID_output
        # Pi deadzone: math.fabs(180 - color_y) > 75
        if math.fabs(self.IMG_H / 2.0 - self.color_y) > 75:
            self.yservo_pid.SystemOutput = self.color_y
            self.yservo_pid.SetStepSignal(self.IMG_H / 2.0)
            self.yservo_pid.SetInertiaTime(0.01, 0.1)

            # Pi formula: servo = (1150 + pid_output - 500) / 10 = 65 + pid_output/10
            target_valuey = int(1050 + self.yservo_pid.SystemOutput)
            target_servoy = int((target_valuey - 500) / 10)
            target_servoy = max(self.TILT_MIN, min(self.TILT_MAX, target_servoy))

            self._send_servo(self.TILT_SERVO_ID, target_servoy)

        # Debug overlay
        cv2.putText(frame, f'x:{int(self.color_x)} pan:{int((1400 + self.xservo_pid.SystemOutput - 500) / 10)}',
                    (40, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f'y:{int(self.color_y)} tilt:{int((1050 + self.yservo_pid.SystemOutput - 500) / 10)}',
                    (40, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = ColorTrackingNode()

    print('\n--- Color Tracking (Jetson Orin NX / M3PRO) ---')
    print('Click color buttons or use keys: r g b y o c q')
    print('Prerequisites:')
    print('  sh ~/start_agent.sh')
    print('  ros2 launch orbbec_camera dabai_dcw2.launch.py\n')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node._on_close()
        time.sleep(0.2)
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
