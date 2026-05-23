---
name: reconfig-ros2-server
description: Fix ROS2 node discovery across WiFi networks by configuring FastDDS to bind to all interfaces (0.0.0.0)
user_invocable: true
---

# Reconfigure ROS2 FastDDS to Bind All Interfaces

Fix the issue where ROS2 nodes become undiscoverable after the robot switches WiFi networks. FastDDS binds DDS sockets to whichever interface IP is active at startup; this skill forces binding to `0.0.0.0` so discovery works on any network.

## Prerequisites

- SSH access to the robot (use the `robot-ssh` skill's connection logic)

## Connection

Use the same SSH connection logic as the `robot-ssh` skill:

1. Check current WiFi: `networksetup -getairportnetwork en0`
2. Use the appropriate IP:
   - **ROSMASTER** network → `192.168.8.88`
   - **MHL** network → `192.168.50.103`
3. SSH command: `ssh -i ~/.ssh/id_ed25519 jetson@<IP>`

## Instructions

Execute all steps **on the robot via SSH**. Verify each step before proceeding.

### Step 1 — Create the FastDDS XML profile

Create `/home/jetson/fastdds_all_interfaces.xml`:

```bash
cat > /home/jetson/fastdds_all_interfaces.xml <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_all</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>0.0.0.0</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <profiles>
        <participant profile_name="default_participant" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>udp_all</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
XML
```

Verify the file was created: `cat /home/jetson/fastdds_all_interfaces.xml`

### Step 2 — Add the export to startup files

The env var `FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_all_interfaces.xml` must be set before any ROS2 process starts. Add it to each of the following files, **only if not already present**.

#### 2a — `/home/jetson/.bashrc`

Add the export **before** any `source /opt/ros/...` lines:

```bash
grep -q 'FASTRTPS_DEFAULT_PROFILES_FILE' /home/jetson/.bashrc || \
  sed -i '/source \/opt\/ros/i export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_all_interfaces.xml' /home/jetson/.bashrc
```

If there is no `source /opt/ros` line, append instead:

```bash
grep -q 'FASTRTPS_DEFAULT_PROFILES_FILE' /home/jetson/.bashrc || \
  echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_all_interfaces.xml' >> /home/jetson/.bashrc
```

#### 2b — `/home/jetson/start_agent.sh`

Add the export **before** the `ros2 run` command:

```bash
grep -q 'FASTRTPS_DEFAULT_PROFILES_FILE' /home/jetson/start_agent.sh || \
  sed -i '/ros2 run/i export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_all_interfaces.xml' /home/jetson/start_agent.sh
```

#### 2c — `/home/jetson/joy_control/joy.sh`

Add the export **before** sourcing ROS2:

```bash
grep -q 'FASTRTPS_DEFAULT_PROFILES_FILE' /home/jetson/joy_control/joy.sh || \
  sed -i '/source.*ros/i export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_all_interfaces.xml' /home/jetson/joy_control/joy.sh
```

#### 2d — `/home/jetson/.config/autostart/check_sensor.desktop`

Add the export to the `Exec=` line. First inspect the current line:

```bash
grep '^Exec=' /home/jetson/.config/autostart/check_sensor.desktop
```

Then prepend the export inside the Exec command (using `bash -c`), e.g.:

```bash
sed -i 's|^Exec=\(.*\)|Exec=bash -c "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_all_interfaces.xml \&\& \1"|' /home/jetson/.config/autostart/check_sensor.desktop
```

**Important**: The exact sed command depends on the current `Exec=` value. Read the file first and adjust accordingly. If the export is already present, skip this file.

### Step 3 — Verify all edits

```bash
echo "=== .bashrc ===" && grep FASTRTPS /home/jetson/.bashrc
echo "=== start_agent.sh ===" && grep FASTRTPS /home/jetson/start_agent.sh
echo "=== joy.sh ===" && grep FASTRTPS /home/jetson/joy_control/joy.sh
echo "=== check_sensor.desktop ===" && grep FASTRTPS /home/jetson/.config/autostart/check_sensor.desktop
```

All four files should show the export line.

### Step 4 — Immediate test (optional)

Kill stale processes and restart with the new profile to test without rebooting:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_all_interfaces.xml
killall micro_ros_agent 2>/dev/null
# Ask the user how micro_ros_agent is normally started, then restart it
# e.g.: source /opt/ros/humble/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
```

Then check discovery:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_all_interfaces.xml
ros2 topic list
```

Expected topics include `/scan0`, `/scan1`, `/imu/data_raw`.

### Step 5 — Reboot and verify persistence

```bash
sudo reboot
```

**Warn the user**: this will drop SSH. Wait ~60 seconds, then reconnect and run:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/jetson/fastdds_all_interfaces.xml
ros2 topic list
```

Also verify sockets are bound to `0.0.0.0` (not a specific IP):

```bash
ss -ulnp | grep micro_ros
```

## Rollback

To undo, remove the export lines from all four files and delete the XML profile:

```bash
sed -i '/FASTRTPS_DEFAULT_PROFILES_FILE/d' /home/jetson/.bashrc /home/jetson/start_agent.sh /home/jetson/joy_control/joy.sh
# For check_sensor.desktop, manually restore the original Exec= line
rm /home/jetson/fastdds_all_interfaces.xml
sudo reboot
```

## Notes

- Multicast already works on MHL — this fix ensures **unicast** DDS discovery also works across network switches.
- The XML profile disables built-in transports and uses only the custom `udp_all` transport bound to `0.0.0.0`.
- After this change, ROS2 nodes will be discoverable regardless of which WiFi network the robot is connected to.
