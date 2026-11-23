# HexFellow planar tracking adapter

Optional ROS 2 Humble package; legacy teleop and official driver launches do
not need it. It converts **one** observation-relative target in a `nav_msgs/Path`
containing exactly one `PoseStamped` at a time to feedback `Twist` commands
for the official HexFellow driver.
It does not replace the driver, use a motor SDK, schedule a trajectory, or
add control modes. Control Manager remains the authority for routing.

When enabled this node must be the **only publisher** to the driver's
`/chassis/cmd_vel`. Wire the manager's autonomous Path output to `target_topic`
and its separate override Twist output to `teleop_topic`; do not also route
teleop directly to the official driver.

## Launch the official driver wrapper

Install/build the official external `hex_bridge`, `hex_device_msgs`, and
`hex_device_ros_wrapper` packages plus their vendor SDK prerequisites. They
are runtime dependencies, not vendored/reimplemented here. For setup see
[teleoperation dependencies](../../projects/HoloBrain/teleop/README.md#hexfellow-chassislift).

The wrapper starts official `launch/ros2/multi_bringup.launch.py` **once**
and the tracker automatically:

```bash
ros2 launch robo_orchard_hexfellow_ros2 hexfellow_tracking.launch.py \
  chassis_url:=192.0.2.10:8439 lift_url:=192.0.2.11:8439
```

Supply actual device addresses. This entry point does not start a Manager,
Pico, or Deploy; use it with your existing managed runtime. If the official
drivers already run, append `start_drivers:=false` (address arguments are
still required, but ignored). Never leave another publisher on driver
`/chassis/cmd_vel`. `enable_chassis_bridge`, `enable_lift_bridge`,
`chassis_is_kcp`, `lift_is_kcp` default to `true`; both `*_read_only` options
default to `false`. Disabling a bridge alone does not omit its driver adapter.
The wrapper passes official `frame_id:=<body_frame>` (default `base_link`),
`simple_mode:=true`, and `enable_ros_clock:=true`. Official topics remain
`/chassis/cmd_vel` and `/chassis/odom`, with odom parent `odom`.
When reusing drivers, configure them identically; device-clock stamps are not
compatible with ROS observation references.

For dual Piper + Pico + chassis/lift, start the complete optional path instead
of the old compatibility launch:

```bash
ros2 launch robo_orchard_hexfellow_ros2 \
  hexfellow_piper_pico_tracking.launch.py \
  chassis_url:=192.0.2.10:8439 lift_url:=192.0.2.11:8439 \
  left_can_port:=can_left right_can_port:=can_right \
  urdf_path:=/path/to/robot.urdf
```

This composes existing arm/Pico/HexFellow adapters with one Pico bridge, one
Control Manager, one official chassis/lift driver launch, and one tracker.
The old driver's include is disabled inside this composition. Caller CAN,
URDF, teleop speed, VR, and reset arguments retain the existing compatibility
launch behavior. `start_drivers:=false` reuses externally owned chassis/lift
drivers, not the Piper drivers. Do not separately start any included runtime.

`control_manager_config_file` defaults to installed
[`config/control_manager.example.yaml`](config/control_manager.example.yaml).
Review/customize its service names and inference node discovery for the robot.
Supply joint names/reset targets through the existing Piper launch arguments.
It contains five channels:

| Channel | Autonomous input | Override input | Manager output |
| --- | --- | --- | --- |
| Left arm (`JointState`) | `/left_algo_cmd` | `/pico_teleop/joint_left` | `/robot/left/joint_cmd` |
| Right arm (`JointState`) | `/right_algo_cmd` | `/pico_teleop/joint_right` | `/robot/right/joint_cmd` |
| Lift (`JointState`) | `/lift_algo_cmd` | `/hexfellow_teleop/lift_joint_cmd` | `/lift/joint_cmd` |
| Chassis (`Path`) | `/chassis_pose_algo_cmd` | — | `/robot/chassis/pose_target` |
| Chassis teleop (`Twist`) | — | `/hexfellow_teleop/cmd_vel` | `/robot/chassis/teleop_cmd_vel` |

Both chassis routes end at the tracker, which is the only publisher to the
official velocity input. Lift uses existing `JointCommandChannel` absolute
joint positions (meters for its prismatic joint), not a new chassis message.
The example disables replay; when enabled, Path/Twist remain excluded and
joint routes retain normal Manager replay behavior. Reset services affect the
two arms, not unsupported vendor reset services. Startup is STOP; request
`/robot/control/takeover` for teleop, or AUTO plus enabled inference for poses.
The wrapper does not enable inference or launch a model service.

Set `tracker_config_file:=<path>` for a copy of
[`config/tracker.yaml`](config/tracker.yaml) to tune all tracker parameters.
`body_frame` is a shared launch argument and overrides its YAML value to keep
the driver's odometry child frame and target reference frame consistent.
Keep output/odometry topics aligned with the official driver; custom Manager
topics must match the corresponding tracker inputs.

## Deploy and migration

The complete autonomous chain is:
`mobile_traj [T,7] xyz,wxyz` → Deploy `PoseCommandChannel` → single-pose `Path`
→ Control Manager → tracker + odometry → official driver `Twist`.
Every Path retains the original observation stamp; its inner pose carries
the model sample time. Deploy still sends one row per tick. Neither HTTP
payloads nor the driver receive the complete chunk or a new start-time protocol.
For the Manager example, use this Deploy action-channel fragment:

```json
{
  "kind": "pose_command",
  "server_output_key": "mobile_traj",
  "topic": "/chassis_pose_algo_cmd",
  "frame_id": "base_link"
}
```

The model must supply compatible samples at Deploy's configured execution
rate. For a 200Hz model, explicitly set Deploy's
`control_config.control_frequency` to `200.0`; its generic default is 25Hz.
The tracker reads sample intervals from the inner pose
stamps; it has no separate `target_rate` parameter. The independent output
loop uses `control_rate` (default 100Hz).
The lift may be added using a normal joint command channel. This is not a
complete model/observation profile. HoloBrain's current project generator
continues arm-only inference and legacy direct-driver teleop; its reserved
`/chassis_algo_cmd` Twist input is **not** this Pose channel. This package
does not add a project-specific inference generator.

Legacy teleop does not require the Tracker. The default `stale_timeout_s`
for Pico chassis and lift increases from 0.3 s to 0.5 s, even without the
Tracker. For lift, the timeout also applies to position feedback.
To migrate, choose this optional launch, update the Manager to the two
separate chassis types above, and update Deploy to `pose_command`.
Existing pose deployments must change the Manager chassis `msg_type` from
`geometry_msgs/msg/PoseStamped` to `nav_msgs/msg/Path`. Update explicit Deploy
message types and Nexus/custom routing templates too. Upgrade the tracker
and Deploy together, and remove `target_rate` from tracker parameter files.
Do not feed model xyz/wxyz into velocity fields or run tracker and direct
teleop publishers on the same driver topic.

For diagnostic node-only startup (drivers and Manager already supplied):

```bash
ros2 run robo_orchard_hexfellow_ros2 hexfellow_tracker --ros-args \
  -p inference_status_topic:=/robot/inference_service/status
```

## Frames and time

Default odometry is `/chassis/odom` (`nav_msgs/Odometry`), with `odom` parent
and `base_link` child. Targets on `/robot/chassis/pose_target` are Paths with
exactly one pose. Both headers must name `base_link` (`body_frame`): this is
the body frame **at the Path's original observation timestamp**, not the
current body frame or the inner pose's sample time. Empty/multi-pose Paths
and mismatched frames are rejected. The inner stamp must follow the outer
stamp; it is `observation_time + (original_row_index + 1) / control_frequency`,
rounded to nanoseconds. It describes sample timing, not a time to wait for.
Target
x/y/z are meters and quaternion is ROS xyzw. Every component must be finite
and the quaternion nonzero. The adapter projects onto x/y/yaw; z/roll/pitch
are intentionally not controlled. Autonomous output uses only linear.x and
angular.z; all other Twist components are zero.

The 35-second `history_duration` exceeds the usual 30-second HTTP timeout.
Odometry x/y and shortest-angle yaw are interpolated between bracketing
samples. Missing/out-of-history timestamps are rejected; there is no
current-odometry fallback or extrapolation. Each observation stamp latches
its reference pose once; subsequent rows retain that origin even after
history pruning. New chunks use their own reference, never differences
between unrelated chunks. Observation and odometry must share a ROS clock.

Receipt freshness uses monotonic wall time, not the intentionally old target
header. Default `command_timeout` and `odom_timeout` are 0.5s, and
`status_timeout` is 3s (allowing the 1Hz status heartbeats). Inputs need
periodic fresh messages. If odometry receipts are more than `odom_timeout`
apart, the first recovered sample starts a new history. Targets from the gap
are rejected; interpolation resumes between post-gap samples. Backward odometry
stamps, wrong frames, invalid poses, or a sample-to-sample jump exceeding
`reset_distance` (1m) / `reset_angle` (1rad) cancel the target and clear history.
These configurable guards detect physical odometry resets, including driver
initialization; they require an odometry rate high enough that legitimate
motion does not exceed those per-sample limits. After a reset, new observation
references must be bracketed by the new history. No automatic frame recovery.

## Feedback and stopping

Uses reference-velocity feedforward plus proportional body-frame error:
`v = vr * cos(etheta) + longitudinal_gain * ex`,
`omega = wr + lateral_gain * vr * ey + heading_gain * sin(etheta)`.
Here `ex`/`ey` are target position errors in the current body frame (meters),
and `etheta` is target minus current yaw. Positive gains default to 2/s,
4/m², and 4/s respectively. Signed reference forward speed `vr` and yaw rate
`wr` come from successive accepted targets in the same observation chunk:
project position differences onto the preceding target's heading and wrap
yaw differences, then divide by the difference between the two inner pose
sample stamps. Derivatives are computed on every accepted target callback,
not at the 100Hz output rate. Skipped samples use their full elapsed model
interval; callback receipt jitter does not affect the derivative. The shared
Path `header.stamp` selects only the fixed observation reference. The first
target of each new observation has zero feedforward; cancellation clears
the target, previous sample time and reference velocity. No differences are
taken across observations. Duplicate or backward sample times within the same
observation are ignored without replacing the goal or refreshing its timeout.
Repeated poses with increasing sample times produce zero reference velocity.

Each output tick consumes the latest newly received reference velocity,
without aggregating samples. With 200Hz targets and 100Hz output, each tick
normally has a fresh sample. A tick with no new target uses zero feedforward
and retains pose feedback until `command_timeout`; the last sample's velocity
is not extrapolated across an inference gap. There is no end-of-chunk marker
or trajectory scheduler. Scheduling jitter that leaves a tick without a new
target can therefore produce a pose-feedback-only tick.

Linear/angular speeds saturate at 0.3m/s and 0.3rad/s.
Both velocities scale together when either limit is reached, preserving the
controller's curvature rather than independently clipping its actuators.
There are no position/yaw deadbands or target-bearing `atan2` turns for tiny
lateral errors. This replaces the polar controller: `distance_gain`,
`final_heading_gain`, `position_tolerance`, and `yaw_tolerance` are removed;
migrate custom YAML to `longitudinal_gain`, `lateral_gain`, and `heading_gain`.

This is local trajectory tracking, not global arbitrary-pose regulation or
exact timed execution. At `vr=0`, pure lateral error is not corrected; a
stationary sideways target cannot be reached by this law. Fast/curved paths
may exceed chassis limits. Tune gains and limits cautiously on secured
hardware. Unit tests include ideal planar simulations, not hardware validation.

Only fresh `auto` ControlMode plus fresh `enabled` InferenceStatus permit
autonomous feedback. Defaults are `/robot/control/status` and
`/robot/inference_service/status` (Deploy publishes relative `status`;
configure for its actual namespace).
Mode exit,
inference disable, stale state/odom/target, and invalid input cancel the target;
returning to AUTO cannot resume it. Zero pose is a real target, not a stop.
Previously accepted observation stamps at/before cancellation are rejected,
including late rows of that chunk; observation stamps must advance between
chunks. This is not a network generation protocol: use the manager's live-only
Path route and Deploy's generation cancellation, not a replay/latched source.

In fresh `takeover`, finite Twist messages from
`/robot/chassis/teleop_cmd_vel` pass through **all six components unchanged**,
without requiring inference or odometry. The timer leaves fresh teleop alone
and sends zero if teleop or mode expires. STOP/resetting/unknown mode sends
zero. Mode transitions send zero; orderly shutdown/errors request zero.
A process crash/network failure cannot guarantee delivery: configure an
independent driver/hardware watchdog and emergency stop.

All topic names, expected frames, gains, limits, timeouts, reset thresholds,
and `control_rate` (100Hz) are ROS parameters.
Run focused non-hardware tests:

```bash
PYTHONPATH=ros2_package/robo_orchard_hexfellow_ros2 \
  python -m pytest ros2_package/robo_orchard_hexfellow_ros2/tests
```
