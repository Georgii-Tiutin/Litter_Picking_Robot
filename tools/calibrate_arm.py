#!/usr/bin/env python3
"""Arm calibration script for ROSMASTER M3 Pro (Jetson Orin NX).

Two-phase calibration:
  Phase 1: Move arm to vertical position via ROS2 (agent must be running)
  Phase 2: Calibrate servo offsets via direct serial (agent must be stopped)
"""

import sys
import os
import time
import subprocess

ROS_ENV = os.environ.copy()
ROS_ENV["ROS_DOMAIN_ID"] = "30"


def is_agent_running():
    result = subprocess.run(
        ["pgrep", "-f", "micro_ros_agent"],
        capture_output=True, text=True
    )
    return result.returncode == 0


def ros2_pub(topic, msg_type, data):
    subprocess.run(
        ["ros2", "topic", "pub", topic, msg_type, data, "--once"],
        capture_output=True, text=True, timeout=10, env=ROS_ENV
    )


def beep(duration_ms=100, count=1):
    for i in range(count):
        ros2_pub("/beep", "std_msgs/msg/UInt16", f"data: {duration_ms}")
        if i < count - 1:
            time.sleep(0.2)


def phase1_position_arm():
    print("\n=== Phase 1: Pre-position arm ===\n")

    if not is_agent_running():
        print("  micro_ros_agent is NOT running.")
        print("  Cannot pre-position arm via ROS2.")
        print("  Skipping to Phase 2 (manual positioning required).\n")
        return False

    print("  micro_ros_agent is running.")
    print("  Moving all joints to 90 deg, gripper to 180 deg (closed)...")

    ros2_pub(
        "/arm6_joints",
        "arm_msgs/msg/ArmJoints",
        "{joint1: 90, joint2: 90, joint3: 90, joint4: 90, joint5: 90, joint6: 180, time: 1500}"
    )

    time.sleep(2)
    beep(100, 3)

    print("  Arm should now be in vertical position.\n")
    return True


def phase2_calibrate():
    print("=== Phase 2: Servo calibration ===\n")

    if is_agent_running():
        print("  WARNING: micro_ros_agent is still running!")
        print("  Stop it first:")
        print("    sudo pkill -f micro_ros_agent\n")
        input("  Press Enter when the agent is stopped... ")

        if is_agent_running():
            print("  ERROR: Agent is still running. Cannot proceed.")
            return False

    print("  Opening serial connection...")

    sys.path.insert(0, os.path.expanduser("~"))
    try:
        from config_robot import MicroROS_Robot
    except ImportError:
        print("  ERROR: config_robot.py not found in home directory.")
        return False

    try:
        bot = MicroROS_Robot(debug=False)
    except Exception as e:
        print(f"  ERROR: Cannot open serial port: {e}")
        print("  Make sure the agent is stopped and the cable is connected.")
        return False

    version = bot.read_version()
    print(f"  Firmware version: {version}")

    if not version:
        print("  WARNING: Could not read firmware version. Retrying...")
        time.sleep(1)
        version = bot.read_version()
        if not version:
            print("  ERROR: No response from control board.")
            del bot
            return False

    print("  Enabling torque briefly...")
    bot.set_arm_torque(1)
    time.sleep(0.5)

    print("  Disabling torque (servos are now limp).\n")
    bot.set_arm_torque(0)
    time.sleep(0.5)

    print("  >> Manually adjust the arm now:")
    print("     - Straighten all arm segments vertically")
    print("     - Close the gripper fully")
    print("     - Target shape: straight vertical line + closed gripper (L-shape)\n")

    try:
        confirm = input("  Type 'y' and press Enter when ready: ")
    except KeyboardInterrupt:
        print("\n  Cancelled. Re-enabling torque for safety...")
        bot.set_arm_torque(1)
        del bot
        return False

    if confirm.strip().lower() != 'y':
        print("  Aborted. Re-enabling torque...")
        bot.set_arm_torque(1)
        del bot
        return False

    print()
    all_ok = True
    for i in range(1, 7):
        state = bot.set_arm_calib_offset(i)
        if state < 0:
            state = bot.set_arm_calib_offset(i)
        status = "OK" if state == 1 else "FAILED"
        if state != 1:
            all_ok = False
        print(f"  Servo {i}: state={state} {status}")

    print()
    bot.set_arm_torque(1)
    print("  Torque re-enabled.\n")

    del bot

    if all_ok:
        print("=== Calibration complete! All 6 servos OK. ===\n")
    else:
        print("=== Calibration finished with errors. Check servo states above. ===\n")

    print("  >> Restart the agent: sh ~/start_agent.sh\n")
    return all_ok


def main():
    print("\n" + "=" * 45)
    print("  ROSMASTER M3 Pro — Arm Calibration")
    print("=" * 45)

    phase1_position_arm()

    input("  Press Enter to continue to calibration... ")

    phase2_calibrate()


if __name__ == "__main__":
    main()
