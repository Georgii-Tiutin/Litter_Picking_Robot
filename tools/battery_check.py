#!/usr/bin/env python3
"""Check battery voltage and display charge percentage."""

import subprocess
import sys

# 3S Li-ion discharge curve: (voltage, percentage)
# Based on Yahboom battery label: 12.6V max, 8.1V cut-off
DISCHARGE_CURVE = [
    (12.60, 100),
    (12.30,  90),
    (12.00,  80),
    (11.70,  60),
    (11.40,  40),
    (11.10,  20),
    (10.80,  10),
    (10.50,   5),
    ( 9.90,   1),
    ( 8.10,   0),
]

def voltage_to_percent(voltage):
    if voltage >= DISCHARGE_CURVE[0][0]:
        return 100
    if voltage <= DISCHARGE_CURVE[-1][0]:
        return 0
    for i in range(len(DISCHARGE_CURVE) - 1):
        v_high, p_high = DISCHARGE_CURVE[i]
        v_low, p_low = DISCHARGE_CURVE[i + 1]
        if v_low <= voltage <= v_high:
            ratio = (voltage - v_low) / (v_high - v_low)
            return p_low + ratio * (p_high - p_low)
    return 0

def get_status(percent):
    if percent > 90:
        return "FULL"
    if percent > 40:
        return "OK"
    if percent > 20:
        return "LOW"
    return "CRITICAL"

def read_battery_voltage():
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source ~/yahboomcar_ws/install/setup.bash 2>/dev/null; "
        "export ROS_DOMAIN_ID=30 && "
        "timeout 5 ros2 topic echo /battery --once"
    )
    result = subprocess.run(
        ["bash", "-c", cmd],
        capture_output=True, text=True, timeout=10
    )
    if result.returncode != 0:
        print(f"Error reading battery topic: {result.stderr.strip()}")
        sys.exit(1)
    for line in result.stdout.splitlines():
        if line.startswith("data:"):
            return float(line.split(":")[1].strip())
    print("No battery data received.")
    sys.exit(1)

if __name__ == "__main__":
    voltage = read_battery_voltage()
    percent = voltage_to_percent(voltage)
    status = get_status(percent)
    print(f"Battery: {voltage:.2f}V | {percent:.0f}% | {status}")
