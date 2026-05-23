---
name: disable-hotspot
description: Modify the robot's startup routine to disable the ROSMASTER hotspot and auto-connect to MHL WiFi instead
user_invocable: true
---

# Disable ROSMASTER Hotspot & Connect to MHL WiFi

Modify the Jetson Orin robot's startup networking so it no longer creates the ROSMASTER hotspot and instead connects to the MHL WiFi network automatically on boot.

## Prerequisites

- SSH access to the robot (use the robot-ssh skill's connection logic)
- The MHL WiFi network must be in range during testing

## Connection

Use the same SSH connection logic as the `robot-ssh` skill:

1. Check current WiFi: `networksetup -getairportnetwork en0`
2. Use the appropriate IP:
   - **ROSMASTER** network → `192.168.8.88`
   - **MHL** network → `192.168.50.103`
3. SSH command: `ssh -i ~/.ssh/id_ed25519 jetson@<IP>`

## Instructions

Execute the following steps **on the robot via SSH**. After each step, verify the command succeeded before proceeding.

### Step 1 — Investigate the current hotspot setup

Run these commands to understand how the ROSMASTER hotspot is currently configured:

```bash
# Check for hostapd
systemctl list-unit-files | grep -i hostapd
systemctl is-active hostapd 2>/dev/null

# Check for create_ap or autohotspot services
systemctl list-unit-files | grep -iE 'hotspot|create_ap|autohotspot'

# Check NetworkManager connections (hotspot is often a NM connection)
nmcli connection show | grep -iE 'hotspot|rosmaster|Hotspot'

# Check for custom startup scripts that create the hotspot
grep -rl 'hotspot\|create_ap\|hostapd\|ROSMASTER' /etc/rc.local /home/jetson/.config/autostart/ /etc/systemd/system/ /etc/NetworkManager/dispatcher.d/ 2>/dev/null

# Check NetworkManager connection files
ls /etc/NetworkManager/system-connections/
```

**Important**: Record what you find before making changes. Tell the user what mechanism is in use.

### Step 2 — Disable the hotspot

Based on what Step 1 reveals, apply the appropriate method:

#### If hostapd service:
```bash
sudo systemctl stop hostapd
sudo systemctl disable hostapd
sudo systemctl mask hostapd
```

#### If NetworkManager hotspot connection:
```bash
# Get the exact connection name from Step 1, then:
nmcli connection modify "<HOTSPOT_CONNECTION_NAME>" connection.autoconnect no
# Or delete it entirely:
# nmcli connection delete "<HOTSPOT_CONNECTION_NAME>"
```

#### If create_ap / autohotspot service:
```bash
sudo systemctl stop <service_name>
sudo systemctl disable <service_name>
sudo systemctl mask <service_name>
```

#### If custom script in autostart:
```bash
# Disable the desktop autostart entry
mv /home/jetson/.config/autostart/<hotspot_file>.desktop /home/jetson/.config/autostart/<hotspot_file>.desktop.disabled
```

#### If configured in /etc/rc.local:
```bash
sudo cp /etc/rc.local /etc/rc.local.bak
# Comment out the hotspot-related lines
sudo sed -i 's/^\(.*hotspot.*\)$/#\1  # disabled by disable-hotspot skill/' /etc/rc.local
sudo sed -i 's/^\(.*hostapd.*\)$/#\1  # disabled by disable-hotspot skill/' /etc/rc.local
sudo sed -i 's/^\(.*create_ap.*\)$/#\1  # disabled by disable-hotspot skill/' /etc/rc.local
```

### Step 3 — Configure MHL WiFi to auto-connect on boot

```bash
# Check if MHL connection already exists
nmcli connection show | grep MHL

# If it exists, ensure autoconnect is on and priority is high:
nmcli connection modify MHL connection.autoconnect yes
nmcli connection modify MHL connection.autoconnect-priority 100

# If it does NOT exist, ask the user for the MHL WiFi password, then:
# nmcli connection add type wifi ifname wlan0 con-name MHL ssid MHL
# nmcli connection modify MHL wifi-sec.key-mgmt wpa-psk wifi-sec.psk "<PASSWORD>"
# nmcli connection modify MHL connection.autoconnect yes
# nmcli connection modify MHL connection.autoconnect-priority 100
```

**Important**: If the MHL connection doesn't exist and you need to create it, **ask the user for the WiFi password** — do not guess or hardcode it.

### Step 4 — Verify the configuration

```bash
# Show all connections and their autoconnect status
nmcli -f NAME,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show

# Confirm the hotspot is disabled
systemctl is-enabled hostapd 2>/dev/null; echo "hostapd: $?"
nmcli connection show | grep -iE 'hotspot|rosmaster'

# Confirm MHL is set to autoconnect
nmcli connection show MHL | grep -E 'autoconnect|priority'
```

### Step 5 — Test (optional, will disconnect if on ROSMASTER)

If the user wants to test immediately and is currently connected via the ROSMASTER hotspot, **warn them** that this will drop the SSH connection. They will need to reconnect via the MHL network (IP `192.168.50.103`) after the robot switches.

```bash
# Switch to MHL now (will drop connection if on ROSMASTER hotspot)
nmcli connection up MHL
```

After reconnecting via MHL:
```bash
# Confirm the robot is on MHL
iwgetid -r
ip addr show wlan0
```

## Rollback

If something goes wrong and the user needs to restore the hotspot:

```bash
# Unmask/re-enable hostapd if it was the mechanism:
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl start hostapd

# Or re-enable the NM hotspot connection:
nmcli connection modify "<HOTSPOT_CONNECTION_NAME>" connection.autoconnect yes

# Or restore rc.local backup:
sudo cp /etc/rc.local.bak /etc/rc.local
```

## Notes

- After disabling the hotspot, the robot will only be reachable on MHL WiFi (`192.168.50.103`) or via a direct ethernet connection.
- If MHL is unavailable at boot, the robot will have no WiFi connectivity until MHL comes in range.
- A full reboot test is recommended to confirm the changes persist.
