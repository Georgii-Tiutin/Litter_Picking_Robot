# Robot config backups

Off-robot copies of files moved or modified on the ROSMASTER M3 Pro, so they
survive a reflash or an accidental `rm` on the Jetson.

| File | Original location on robot | Why it's here | Date |
|---|---|---|---|
| `joy_control.desktop` | `~/.config/autostart/joy_control.desktop` | Gamepad autostart, disabled per course 0 §1.3. The robot's own copy now sits at `~/joy_control.desktop`. | 2026-08-26 |

## Restoring gamepad autostart

On the robot:

```
mv ~/joy_control.desktop ~/.config/autostart/
```

Or, if the robot's copy is gone, push this one back:

```
scp robot_config_backups/joy_control.desktop robot:~/.config/autostart/
```

Either way it takes effect on the next boot. To run the gamepad control
immediately without rebooting: `sh ~/joy_control/joy.sh`
