# RoboOrchard Wuji Glove ROS 2

This package publishes Wuji Glove streams and retargets the 21-point hand
skeleton to the 20-joint `sensor_msgs/msg/JointState` command accepted by
`wujihandros2` 1.1.0.

## Repository Layout

- `ros2_package/robo_orchard_wuji_glove_msg_ros2`: Wuji Glove ROS messages.
- `ros2_package/robo_orchard_wuji_glove_ros2`: driver, conversions, retarget
  node, configuration, and standalone glove launch file.
- `ros2_package/robo_orchard_teleop_ros2/launch/`
  `wuji_glove_teleop_compat.launch.py`: glove-to-hand direct control.
- `ros2_package/robo_orchard_teleop_ros2/launch/`
  `wuji_glove_dagger_compat.launch.py`: algorithm/glove selection through the
  existing `take_over` muxer.

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
Wuji Hand driver. Direct teleoperation remaps the candidate command straight to
the Hand command topic. DAgger keeps the candidate topic visible and publishes
only the mux-selected command to `/hand_*/joint_commands`.

## Vendor SDK

The nodes use the official Wuji Python SDK directly. The tested and pinned
version is `wuji-sdk==2026.7.21`; install it into the same Python environment
used by ROS 2 before building or launching:

```bash
python3 -m pip install "wuji-sdk==2026.7.21"
```

`wuji_glove_driver_node` uses `SdkManager` to select a glove by serial number
or handedness, opens callback subscriptions, and closes every subscription
before disconnecting. `wuji_glove_retarget_node` uses
`RetargetSession.for_hand(HandModel.WujiHand, ...)` and publishes the returned
firmware-order 20-vector without additional joint reordering.

An empty `serial_number` scans the SDK discovery results, filters them to Wuji
Gloves of the requested handedness, and conservatively retains matching devices
whose type is unavailable during discovery. Selection is deterministic only
when exactly one candidate is discoverable. Zero or multiple candidates fail
the connection and are reported in the connection diagnostics; configure an
explicit serial number whenever multiple same-side candidates are present.
Once selected, the driver connects by serial number and validates the resulting
device type and handedness before opening subscriptions.

The SDK performs an initial clock synchronization inside `connect()` and then
resynchronizes in the background. Before opening subscriptions, the wrapper
also calls `sync_time()` and validates its timestamp and round-trip result. A
failed or invalid synchronization aborts initialization and follows the normal
disconnect/retry path; callbacks remain unable to publish until connection
initialization has completed. `time_sync_max_age_s` and
`time_sync_max_rtt_ms` bound the accepted result. When skeleton output is
enabled, the driver remains in WARMUP until its first valid frame and reconnects
if `critical_stream_timeout_s` elapses without another valid frame.

The retargeter requires all 21 joints to meet `min_joint_confidence`. Low-quality
frames produce no robot command and expose a tracking-lost diagnostics state.
Tracking loss or a gap longer than `tracking_reset_gap_s` resets the stateful SDK
retarget session before commands resume.

The default SDK user is recommended for teleoperation by the Wuji
documentation. Set `sdk_user` to a user ID or a unique display name only when
using a user-specific calibrated hand model. The node restores the process's
previous SDK user when it disconnects.

SDK-managed glove calibration is stored below `~/.wuji/sdk/users/`. Mount the
host's `.wuji` directory at the container user's `~/.wuji`, set
`hand_model_path` to the mounted users directory, and pass `sdk_user` as either
a user ID or an exact display name. The driver resolves that user through the
SDK manager, then selects `<hand_model_path>/<user-id>/models/right_hand.urdf`
or `left_hand.urdf` from `hand_side`.

When `hand_model_path` is the SDK's own managed users directory, the driver
switches to the resolved named user before connecting and lets the SDK load that
managed model itself. It does not pass an SDK-managed path through the external
override setter. A users directory mounted elsewhere is also supported: after
resolving the SDK user, the selected URDF is applied through the official
`glove.hand_model_path()` resource. In both cases the named user must exist in
the SDK manager loaded by the process. An empty `hand_model_path` preserves the
SDK's normal user-selection behavior.

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
  hand_side:=left,right \
  glove_sdk_user:=<user-id-or-display-name> \
  glove_hand_model_path:=/root/.wuji/sdk/users
```

`hand_name`, `hand_serial_number`, `glove_namespace`,
`glove_serial_number`, `glove_device_name`, and `glove_frame_prefix` are
per-instance arguments. `glove_namespace` values are normalized to absolute ROS
names before uniqueness checks. `glove_sdk_user` accepts either one value shared
by all instances or one value per instance. The DAgger launch also treats
`algo_topic` as per-instance. Each other non-empty argument must contain the
same number of comma-separated values as `hand_side`; an empty position keeps
that instance's derived default.

For multiple devices with the same handedness, names and physical device
selection cannot be inferred. Specify a unique hand name and both serial
numbers for every repeated side. The compatibility launches validate unique
Glove namespaces, SDK aliases, same-side frame prefixes, and non-empty serial
numbers:

```bash
ros2 launch robo_orchard_teleop_ros2 wuji_glove_dagger_compat.launch.py \
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

Direct glove-to-hand teleoperation with `wujihandros2` 1.1.0:

```bash
ros2 launch robo_orchard_teleop_ros2 wuji_glove_teleop_compat.launch.py \
  hand_side:=right \
  hand_serial_number:=<hand-serial> glove_serial_number:=<glove-serial> \
  glove_sdk_user:=<user-id-or-display-name> \
  glove_hand_model_path:=/root/.wuji/sdk/users
```

DAgger control selection:

```bash
ros2 launch robo_orchard_teleop_ros2 wuji_glove_dagger_compat.launch.py \
  hand_side:=right \
  hand_serial_number:=<hand-serial> glove_serial_number:=<glove-serial> \
  glove_sdk_user:=<user-id-or-display-name> \
  glove_hand_model_path:=/root/.wuji/sdk/users
```

The DAgger launch preserves the existing muxer behavior. It starts in
autonomous mode; `/hand_right/takeover_muxer/trigger_takeover` selects glove
commands and `/hand_right/takeover_muxer/release_control` selects algorithm
commands. The Glove candidate is
`/wuji_glove/right/retargeted_joint_commands`; only the mux output uses
`/hand_right/joint_commands`.

## Wuji References

- [Wuji Glove data streams](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/)
- [IMU data](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/imu/)
- [Hand tracking](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/hand-tracking/)
- [Coordinate transforms](https://docs.wuji.tech/docs/zh/wuji-glove/latest/sdk-data-reference/coordinate-transforms/)
- [Wuji SDK subscriptions](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/data-subscription/)
- [Wuji SDK retargeting](https://docs.wuji.tech/docs/zh/wuji-sdk/latest/retargeting/)
- [Wuji Hand ROS 2 interface](https://docs.wuji.tech/docs/zh/wujihandros2/latest/ros2-interface/)
