---
name: robot-ssh
description: Connect to the remote Jetson Orin robot via SSH and execute commands on it
user_invocable: true
---

# Robot SSH

Execute commands on the remote Jetson Orin robot over SSH.

## Connection

- **Default SSH command**: `ssh -i ~/.ssh/id_ed25519 jetson@192.168.8.88`
- **User**: `jetson`
- **Key**: `~/.ssh/id_ed25519`

### Known networks and robot IPs

| WiFi Network | Robot IP |
|---|---|
| **ROSMASTER** | `192.168.8.88` |
| **MHL** | `192.168.50.103` |

If connected to an unknown network, ask the user for the robot's current IP.

## Instructions

1. **Before anything else**, check the current WiFi network:
   ```
   networksetup -getairportnetwork en0
   ```
   - If connected to **ROSMASTER**: use IP `192.168.8.88`.
   - If connected to **MHL**: use IP `192.168.50.103`.
   - If connected to another network: ask the user for the robot's current IP.
   - If not connected to any WiFi: tell the user they need to connect to a WiFi network first.
2. Verify connectivity by running:
   ```
   ssh -i ~/.ssh/id_ed25519 -o ConnectTimeout=5 jetson@<IP> echo ok
   ```
3. If the connection fails, ask the user to check the network and provide the correct IP.
3. To execute commands on the robot, use SSH with the command appended:
   ```
   ssh -i ~/.ssh/id_ed25519 jetson@<IP> "<command>"
   ```
4. For commands that produce long output, consider piping through `tail` or `head` on the remote side.
5. For multi-command sequences, chain them with `&&` inside the SSH command string or use a heredoc:
   ```
   ssh -i ~/.ssh/id_ed25519 jetson@<IP> bash <<'REMOTE'
   command1
   command2
   REMOTE
   ```
6. The robot runs Ubuntu on a Jetson Orin. Expect standard Linux tools and likely ROS2 to be available.
7. For file transfers to/from the robot, use `scp -i ~/.ssh/id_ed25519` with the same user and IP.
