# RoboOrchard Wuji Glove ROS 2

This package publishes Wuji Glove streams and retargets the 21-point hand
skeleton to a 20-joint `sensor_msgs/msg/JointState` command candidate. The
teleop runtime sends that candidate through the global Control Manager before
the selected command reaches the configured Wuji Hand1 or Wuji Hand2 driver.

## Repository Layout

- `ros2_package/robo_orchard_wuji_glove_msg_ros2`: Wuji Glove ROS messages.
- `ros2_package/robo_orchard_wuji_glove_ros2`: driver, conversions, retarget
  node, configuration, and standalone glove launch file.
- `ros2_package/robo_orchard_teleop_ros2/launch/`
  `wuji_glove_teleop_compat.launch.py`: managed glove-to-hand control.
- `ros2_package/robo_orchard_teleop_ros2/launch/`
  `wuji_glove_dagger_compat.launch.py`: global algorithm/glove selection
  through the Control Manager.

## Nodes

- `wuji_glove_driver_node`: connects one glove and publishes tactile, EMF,
  fingertip, hand-angle, skeleton, tactile point-cloud, palm IMU, TF, device
  information, and diagnostics topics. Each high-cost stream is controlled by
  a ROS parameter.
- `wuji_glove_retarget_node`: subscribes to `hand_skeleton`, runs the Wuji Hand
  retargeter, and publishes a complete 20-position
  `retargeted_joint_commands` message.

## Published State

The fully resolved `glove_namespace` is the canonical source identity. Every
running driver must use a unique namespace. Its transient-local `device_info`
message records that namespace and binds it to the connected serial number,
handedness, and SDK device name. Remapping or forwarding a source must preserve
the namespace and its `device_info` topic together; an individual standard ROS
sensor message does not carry vendor-specific identity metadata.

All topics are relative to `glove_namespace` except the standard global TF
topics. Every coordinate frame uses a `frame_prefix`; an empty standalone value
derives from the SDK `device_name`. A normal left/right teleop pair shares one
prefix so that both wrists retain the SDK's common `waist` root. Repeated
same-side devices must use different prefixes. The same prefix is applied to TF
parent/child frames and every sensor message `header.frame_id`.
Frames without a synchronized UTC device timestamp are discarded rather than
restamped with host receive time. The `discarded_frames` diagnostics value makes
these conversion failures observable.

| Topic | Type | Default |
| --- | --- | --- |
| `device_info` | `robo_orchard_wuji_glove_msg_ros2/msg/GloveInfo` | on |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | on |
| `tactile` | `robo_orchard_wuji_glove_msg_ros2/msg/TactileFrame` | on |
| `tactile_zones` | `robo_orchard_wuji_glove_msg_ros2/msg/TactileZones` | on |
| `tactile_binary` | `robo_orchard_wuji_glove_msg_ros2/msg/TactileFrame` | off |
| `tactile_residual` | `robo_orchard_wuji_glove_msg_ros2/msg/TactileFrame` | off |
| `emf_poses` | `robo_orchard_wuji_glove_msg_ros2/msg/PoseArrayWithConfidence` | on |
| `tip_poses` | `robo_orchard_wuji_glove_msg_ros2/msg/PoseArrayWithConfidence` | on |
| `hand_joint_angles` | `robo_orchard_wuji_glove_msg_ros2/msg/HandJointAngles` | on |
| `hand_skeleton` | `robo_orchard_wuji_glove_msg_ros2/msg/HandSkeleton` | on |
| `tactile_point_cloud` | `sensor_msgs/msg/PointCloud2` | on |
| `imu/palm` | `sensor_msgs/msg/Imu` | on |
| `retargeted_joint_commands` | `sensor_msgs/msg/JointState` | on |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | on |

The table describes the standalone `configured` stream profile. Compatibility
teleop launches default to `teleop_minimal`, which keeps only `hand_skeleton`,
`device_info`, and `diagnostics`; set `glove_stream_profile:=configured` when a
teleop process also needs the configured observation and TF streams.

These interfaces have three distinct owners: topics in the table above are
Glove observations, `retargeted_joint_commands` is the retargeter's candidate
command, and `/hand_*/joint_commands` plus `/hand_*/joint_states` belong to the
Wuji Hand driver. Compatibility launches remap the candidate to
`/hand_*/control/override`; the Control Manager alone publishes selected
commands to `/hand_*/joint_commands`. DAgger autonomous candidates use the
configured `/left_hand_algo_cmd` and `/right_hand_algo_cmd` topics.

## Vendor SDK

Install the official `wuji-sdk==2026.10.26` into the same Python environment
used by ROS 2 before building or launching:

```bash
python3 -m pip install "wuji-sdk==2026.10.26"
```

The `hand_model` parameter / standalone launch argument selects the target:

| `hand_model` | Target | Command joint names |
|---|---|---|
| `wuji_hand` (default) | Wuji Hand1 | `left_finger1_joint1` … `left_finger5_joint4`, or `right_` equivalents |
| `wuji_hand_2` | Wuji Hand2 | Official `l_` / `r_` names in firmware order |

See the [Wuji Hand2 command interface](../robo_orchard_wuji2_ros2/README.md#state-and-command-contract)
for joint names and ordering. Unknown models fail startup.

For standalone Wuji Hand2 retargeting:

```bash
ros2 launch robo_orchard_wuji_glove_ros2 wuji_glove.launch.py \
  hand_side:=right hand_model:=wuji_hand_2 \
  command_topic:=/hand_right/control/override
```

This launches the glove and retargeter, not a hand driver or Control Manager.
Use a matching separately configured Hand 2 driver and managed command path;
the compatibility teleop launches also accept `hand_model:=wuji_hand_2` and
select the matching driver. See the
[managed teleoperation launches](../robo_orchard_teleop_ros2/README.md#quick-start).

SDK 2026.10.26 removes the `emf_poses_rate_divider` resource. Replace old
`emf_poses_rate_divider` configuration with `emf_poses_rate_hz` (integer
0–65535). The default `0` leaves the SDK's default rate unchanged.
For a previous non-default divider, choose
an explicit target frequency in Hz based on your device's native rate; the
driver does not guess the native rate or translate dividers.

Rates are quantized by the device and shared by subscribers (last writer wins).
The setting also affects skeleton/retargeting input, even when EMF ROS output
is disabled, and is reapplied on reconnect. Unsupported rate control fails
connection initialization. Glove firmware 1.0.4 has not been hardware-validated.

Without an explicit serial number, exactly one glove must match the requested
side. Use explicit serial numbers for multiple same-side devices.
`time_sync_max_age_s` and `time_sync_max_rtt_ms` bound accepted clock
synchronization; failures cause reconnection. With skeleton output enabled,
`critical_stream_timeout_s` controls reconnection after missing valid frames.

The retargeter requires all 21 joints to meet `min_joint_confidence`. Low-quality
frames produce no robot command and expose a tracking-lost diagnostics state.
The node default is `0.5` for both models. The standalone launch
accepts `min_joint_confidence:=0.0` (or another value in `[0, 1]`) as an explicit
retarget-only override. Leaving the launch argument empty preserves the value
in `config_file`, or the node default when no value is configured. The shared
Wuji teleop, DAgger, and Marvin keyboard launches expose the same override as
`glove_min_joint_confidence`, shared across all Glove instances. Setting `0.0`
bypasses only the low-confidence gate; invalid shapes, non-finite data, and
confidence values outside `[0, 1]` remain rejected. This is a startup setting,
not a dynamic runtime parameter.
After tracking loss or a gap longer than `tracking_reset_gap_s`, retargeting
restarts before commands resume.

The default SDK user is recommended for teleoperation by the Wuji
documentation. Set `sdk_user` to a user ID or a unique display name only when
using a user-specific calibrated hand model.

`hand_model_path` selects the human calibration model, not the target robot.
SDK-managed glove calibration is stored below `~/.wuji/sdk/users/`. Mount the
host's `.wuji` directory at the container user's `~/.wuji`, set
`hand_model_path` to the mounted users directory, and pass `sdk_user` as either
a user ID or an exact display name. The user must exist in the SDK manager.
The model is `<hand_model_path>/<user-id>/models/right_hand.urdf` or
`left_hand.urdf`, according to `hand_side`. A users directory mounted elsewhere
is also supported. Leave `hand_model_path` empty to use the SDK's normal model
selection.

## Launch

Side-dependent launch arguments default to an empty string. When empty, each
launch derives them from `hand_side`:

| Argument | `left` | `right` |
| --- | --- | --- |
| `hand_name` | `hand_left` | `hand_right` |
| `glove_namespace` | `/wuji_glove/left` | `/wuji_glove/right` |
| `glove_device_name` | `wuji_glove_left` | `wuji_glove_right` |
| `glove_frame_prefix` | `wuji_glove_left` | `wuji_glove_right` |
| `algo_topic` | `/left_hand_algo_cmd` | `/right_hand_algo_cmd` |

The standalone glove launch uses `device_name` instead of
`glove_device_name`, with the same derived values. Explicit non-empty values
always override these defaults. Consequently, switching an otherwise identical
launch between hands only requires changing `hand_side`.

For the common two-instance `left,right` launch, both derived frame prefixes are
`wuji_glove`, producing `wuji_glove/waist -> wuji_glove/l_wrist` and
`wuji_glove/waist -> wuji_glove/r_wrist`. One explicit prefix is shared by both
sides; two explicit values request separate trees.

Both compatibility launch files accept multiple comma-separated sides. The
common left/right case needs no additional naming arguments:

```bash
ros2 launch robo_orchard_teleop_ros2 wuji_glove_teleop_compat.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json \
  hand_side:=left,right \
  glove_sdk_user:=<user-id-or-display-name> \
  glove_hand_model_path:=/root/.wuji/sdk/users
```

`hand_name`, `hand_serial_number`, `glove_namespace`,
`glove_serial_number`, `glove_device_name`, and `glove_frame_prefix` are
per-instance arguments. `glove_namespace` values are normalized to absolute ROS
names before uniqueness checks. `hand_model` and `glove_sdk_user` accept either
one value shared by all instances or one value per instance. The DAgger launch
also treats `algo_topic` as per-instance. Each other non-empty per-instance
argument must contain the
same number of comma-separated values as `hand_side`; an empty position keeps
that instance's derived default.

For multiple devices with the same handedness, names and physical device
selection cannot be inferred. Specify a unique hand name and both serial
numbers for every repeated side. The compatibility launches validate unique
Glove namespaces, SDK aliases, same-side frame prefixes, and non-empty serial
numbers:

```bash
ros2 launch robo_orchard_teleop_ros2 wuji_glove_dagger_compat.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json \
  hand_side:=left,left \
  hand_name:=hand_left_a,hand_left_b \
  hand_serial_number:=HAND_A,HAND_B \
  glove_serial_number:=GLOVE_A,GLOVE_B \
  glove_sdk_user:=<user-a>,<user-b> \
  glove_hand_model_path:=/root/.wuji/sdk/users
```

The remaining values derive from each `hand_name`: the example uses glove
namespaces `/wuji_glove/left_a` and `/wuji_glove/left_b`, SDK aliases
and frame prefixes `wuji_glove_left_a` and `wuji_glove_left_b`, and DAgger
algorithm topics `/left_a_hand_algo_cmd` and `/left_b_hand_algo_cmd`. Explicit
per-instance values override these derivations. The calibration model root and
hand rates remain shared scalar arguments for the whole launch.

Glove nodes only:

```bash
ros2 launch robo_orchard_wuji_glove_ros2 wuji_glove.launch.py \
  hand_side:=right glove_serial_number:=<serial-number> \
  sdk_user:=<user-id-or-display-name> \
  hand_model_path:=/root/.wuji/sdk/users
```

Managed glove-to-hand teleoperation with `wujihandros2` 1.1.0:

```bash
ros2 launch robo_orchard_teleop_ros2 wuji_glove_teleop_compat.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json \
  hand_side:=right \
  hand_serial_number:=<hand-serial> glove_serial_number:=<glove-serial> \
  glove_sdk_user:=<user-id-or-display-name> \
  glove_hand_model_path:=/root/.wuji/sdk/users
```

DAgger control selection:

```bash
ros2 launch robo_orchard_teleop_ros2 wuji_glove_dagger_compat.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json \
  hand_side:=right \
  hand_serial_number:=<hand-serial> glove_serial_number:=<glove-serial> \
  glove_sdk_user:=<user-id-or-display-name> \
  glove_hand_model_path:=/root/.wuji/sdk/users
```

Both launches route commands through one global Control Manager, which starts
in `STOP`. Call `/robot/control/takeover` to select glove commands and
`/robot/control/auto` to select algorithm commands. The glove publishes its
candidate to `/hand_right/control/override`; only Manager output uses
`/hand_right/joint_commands`. The default configuration assumes the standard
`hand_left` and `hand_right` names. Custom names or algorithm topics require a
matching `control_manager_config_file`.

## Wuji References

- [Wuji Glove data streams](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/)
- [IMU data](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/imu/)
- [Hand tracking](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/hand-tracking/)
- [Coordinate transforms](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/coordinate-transforms/)
- [Wuji SDK subscriptions](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/data-subscription/)
- [Wuji SDK retargeting](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/retargeting/)
- [Wuji Hand ROS 2 interface](https://docs.wuji.tech/docs/zh/wujihandros2/latest/ros2-interface/)
