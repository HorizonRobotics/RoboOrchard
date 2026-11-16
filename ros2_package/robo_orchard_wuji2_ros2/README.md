# Wuji Hand2 ROS 2 driver

Standalone Wuji Hand2 driver for Ubuntu 22.04 / ROS 2 Humble, using the
official Wuji SDK C. Intended for Hand2 2.2 Beta; firmware 2.13.1 has not
been hardware-validated by this integration.

## Requirements

Install the following dependencies before building:

| Dependency | Installation |
| --- | --- |
| Official Wuji SDK C 2026.10.26 | Download from the [official release](https://github.com/wuji-technology/wuji-sdk/releases/tag/v2026.10.26) |
| ROS 2 and system dependencies | Install those declared in [package.xml](package.xml), including `nlohmann-json3-dev` |
| Official Hand2 ROS interfaces | Use the included [wuji_msgs](../wuji_msgs); do not install another copy in the workspace |

Choose the Linux GNU archive for your architecture (`x86_64` or `aarch64`),
not the Python wheel. For x86-64:

```bash
curl -fLO https://github.com/wuji-technology/wuji-sdk/releases/download/v2026.10.26/wuji-sdk-c-2026.10.26-x86_64-linux-gnu.tar.gz
echo 'ecd2fd28fb8c09c177ded2bb2961e9293709bdc9de71957a5a647e83cff35722  wuji-sdk-c-2026.10.26-x86_64-linux-gnu.tar.gz' | sha256sum -c -
tar -xzf wuji-sdk-c-2026.10.26-x86_64-linux-gnu.tar.gz
```

After sourcing ROS 2 Humble, build from the repository root:

```bash
make ros2-build build_args="--merge-install --packages-select wuji_msgs robo_orchard_wuji2_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DWUJI_SDK_ROOT=/path/to/wuji-sdk-c-2026.10.26-x86_64-linux-gnu"
source ros2_package/install/setup.bash
```

Keep the SDK directory available at runtime; rebuild if you move it.

## Launch and configuration

Launch one hand, disabled by default:

```bash
ros2 launch robo_orchard_wuji2_ros2 wuji_hand2.launch.py \
  hand_name:=hand_right hand_side:=right serial_number:=<serial-number>
```

For a left hand, use `hand_name:=hand_left hand_side:=left`. Give each hand
a unique namespace and use explicit serial numbers when multiple devices
are connected.

Launch one left and one right hand:

```bash
ros2 launch robo_orchard_wuji2_ros2 wuji_hand2_dual.launch.py \
  left_serial_number:=000123 right_serial_number:=000456
```

The dual launch uses `/hand_left` and `/hand_right`. Each side requires
an explicit serial number or endpoint (`left_serial_number` / `left_endpoint`
and `right_serial_number` / `right_endpoint`). Single-hand launches support
automatic discovery without a selector. Other launch arguments apply to
both hands.

Use `--show-args` to inspect launch arguments and defaults:

```bash
ros2 launch robo_orchard_wuji2_ros2 wuji_hand2.launch.py --show-args
```

| Parameter | Default | Usage |
| --- | --- | --- |
| `endpoint` | `""` | Direct connection address; takes precedence over discovery |
| `serial_number` | `""` | Device serial selector; leading zeros are preserved |
| `hand_side` | `right` in launch/config | `left` / `right`; empty in the bare node means no side filter |
| `connect_timeout` | `10.0` | Connection timeout, seconds; `0.0` waits indefinitely |
| `auto_enable` | `false` | Enable all joints at startup |
| `publish_joint_diagnostics` | `true` | Publish joint diagnostics |
| `publish_imu` | `true` | Publish IMU data |
| `publish_fingertip_tactile` | `false` | Publish tactile data for online fingertip modules |

Supplied serial and side selectors are validated, including direct endpoint
connections. Missing or ambiguous matches fail rather than select arbitrarily.
Parameters are startup-only. For file-based configuration, copy
[config/wuji_hand2.yaml](config/wuji_hand2.yaml) and run:

```bash
ros2 run robo_orchard_wuji2_ros2 wuji_hand2_node --ros-args \
  --params-file /path/to/hand.yaml -r __ns:=/hand_right
```

For glove teleoperation, use the
[teleop package](../robo_orchard_teleop_ros2/README.md#quick-start).

## State and command contract

Topics are relative to the hand namespace:

| Topic | Type | Units / contents |
| --- | --- | --- |
| `joint_states` | `sensor_msgs/msg/JointState` | Position rad, velocity rad/s, effort current A (not torque) |
| `joint_commands` | `sensor_msgs/msg/JointState` | Same units as state |
| `joint_diagnostics` | `wuji_msgs/msg/JointDiagnosticsArray` | Joint status/errors, current, voltage, temperature and communication statistics |
| `imu` | `sensor_msgs/msg/Imu` | rad/s and m/s²; orientation unavailable |
| `fingertip_tactile/{thumb,index,middle,ring,pinky}` | `wuji_msgs/msg/FingertipTactile` | N and °C |

All topics use best-effort, volatile QoS: commands depth 1, state/sensors depth 5.
State may contain only online joints; match joints by name, not array position.

Joint names use `l_` or `r_` prefixes with the bodies below.
Named commands accept any order or a subset. Anonymous commands require
exactly 20 positions in the order below.
Velocity and effort may be empty (zero-filled), or must match the position
count. Unknown, duplicate or opposite-side names, invalid lengths and
nonfinite/unrepresentable values reject the whole command.
Omitted joints in subset commands receive zeros, not their previous command.

| Indices | Joint name bodies, in order |
| --- | --- |
| 0–3 | `thumb_cmc_flex`, `thumb_cmc_abd`, `thumb_mcp`, `thumb_ip` |
| 4–7 | `index_finger_mcp_flex`, `index_finger_mcp_abd`, `index_finger_pip`, `index_finger_dip` |
| 8–11 | `middle_finger_mcp_flex`, `middle_finger_mcp_abd`, `middle_finger_pip`, `middle_finger_dip` |
| 12–15 | `ring_finger_mcp_flex`, `ring_finger_mcp_abd`, `ring_finger_pip`, `ring_finger_dip` |
| 16–19 | `pinky_mcp_flex`, `pinky_mcp_abd`, `pinky_pip`, `pinky_dip` |

## Services

Services are relative to the hand namespace and use reliable QoS:

| Service | Type | Usage |
| --- | --- | --- |
| `set_enabled` | `wuji_msgs/srv/SetEnabled` | Enable/disable joints using a 20-entry mask |
| `enable` | `std_srvs/srv/Trigger` | Enable all joints |
| `reset_ctrl` | `std_srvs/srv/Trigger` | Move to the configured posture; see below |
| `clear_faults` | `wuji_msgs/srv/ClearFaults` | Clear faults using a 20-entry mask |
| `set_effort_limit` | `wuji_msgs/srv/SetEffortLimit` | Set one nonnegative current limit, A |
| `set_effort_limits` | `wuji_msgs/srv/SetEffortLimits` | Set 20 nonnegative current limits, A |
| `set_mit_params` | `wuji_msgs/srv/SetMitParams` | Set 20 nonnegative kp/kd values |
| `emergency_stop` | `std_srvs/srv/Trigger` | Stop the whole hand |
| `calibrate_tactile` | `std_srvs/srv/Trigger` | Calibrate all five fingers; secure the hand and unload sensor surfaces first |

All-false masks are no-ops. Check service responses for operation failures.
Tactile calibration does not disable joints and may take 30 seconds or longer
if an SDK call blocks.

## Explicit posture reset

Enable the hand before resetting:

```bash
ros2 service call /hand_right/enable std_srvs/srv/Trigger '{}'
ros2 service call /hand_right/reset_ctrl std_srvs/srv/Trigger '{}'
```

The default target is 20 zero positions (fully extended), not disabled.
Choose a safe target; the driver does not enforce model-specific joint limits.
Reset does not enable joints, clear faults, change gains or redefine origins.

| Parameter | Default | Usage |
| --- | --- | --- |
| `reset_joint_position` | 20 zeros | 20 finite positions in radians, in the order above |
| `reset_duration_s` | `2.0` | Positive motion duration, seconds |
| `reset_timeout_s` | `5.0` | Total timeout, seconds; must exceed motion duration |
| `reset_feedback_timeout_s` | `0.2` | Maximum age of complete feedback, seconds |
| `reset_position_tolerance` | `0.2` | Positive arrival tolerance per joint, radians |

Success requires fresh complete feedback within tolerance; a hand already
at the target needs no motion. Missing/stale feedback, send failure, timeout
or interruption returns `success=false`.
Ordinary commands sent during reset are discarded. Concurrent resets are
rejected; disabling joints, emergency stop or shutdown interrupts the reset.

## Safety

There is no command watchdog or automatic stop on command loss. Control Manager
STOP only stops routing; it does not disable or stop the hardware.
Emergency stop is not latched: later commands or resets may issue motion.
Use explicit hardware stops and independent physical safety measures.

Required joint-state stream failure exits the node; an optional stream failure
stops only that stream. Shutdown attempts to disable all joints and disconnect.
Connection and SDK initialization do not establish a hardware safety interlock.
