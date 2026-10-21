# RoboOrchard Teleop ROS 2

Teleoperation for RoboOrchard robots. The package turns operator input into
robot commands, and provides the take-over (DAgger) plumbing used during data
collection.

Shared activation, source matching, and baseline-cutover contracts are defined
in [`robo_orchard_teleop_msg_ros2`](../robo_orchard_teleop_msg_ros2/README.md).
They keep candidate commands as `JointState` messages and leave Control
Manager responsible for routing commands to hardware.

## Teleoperation modes

- **Aloha leader-follower** — a leader arm drives the follower over CAN.
  Piper only.
- **Pico VR** — a Pico headset streams controller poses and the arm follows
  through IK for Piper, Marvin, and Franka; Hexfellow uses the controller
  sticks for chassis velocity and lift position. The bridge reads raw device
  messages, which the upstream `xrobotoolkit_sdk` does not expose, so build
  [XRoboToolkit-PC-Service-Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind)
  with [`patches/py_bindings.cpp`](patches/py_bindings.cpp) in place of
  `bindings/py_bindings.cpp`. It replaces that file wholesale.
- **Wuji glove** — a data glove drives the Wuji dexterous hand. See
  [`robo_orchard_wuji_glove_ros2`](../robo_orchard_wuji_glove_ros2/README.md)
  for glove setup. The hand driver (`wujihand_bringup`) ships separately:
  clone it into `ros2_package/third_party/` and build it with this workspace.

|                    | Aloha | Pico VR | Wuji glove |
| ------------------ | :---: | :-----: | :--------: |
| Piper              |   ✅   |    ✅    |     —      |
| Marvin             |   —   |    ✅    |     —      |
| Franka FR3         |   —   |    ✅    |     —      |
| Hexfellow chassis/lift | — |    ✅    |     —      |
| Wuji hand          |   —   |    —    |     ✅      |
| Marvin + Wuji hand |   —   |    ✅    |     ✅      |

## Quick start

### Hexfellow managed teleoperation

Install the official external packages `hex_bridge`, `hex_device_msgs`, and
`hex_device_ros_wrapper`, along with the vendor SDK/build prerequisites. Build
their sources in `ros2_package/third_party/` with the workspace; the driver
sources are not bundled in this repository. Pico also requires the patched
XRoboToolkit SDK described above. Replace the example device addresses below
with the actual chassis and lift addresses.

```bash
ros2 launch robo_orchard_teleop_ros2 hexfellow_pico_teleop.launch.py \
  chassis_url:=192.0.2.10:8439 \
  lift_url:=192.0.2.11:8439 \
  control_manager_config_file:=/path/to/project/control_manager.json
```

This includes the vendor's `launch/ros2/multi_bringup.launch.py`, starting the
chassis and lift drivers and their communication bridges. It also starts one
Pico bridge under `/pico_bridge`, both teleop adapters, and one Control Manager
under `/robot/control`. Stop any separately launched Hexfellow drivers before
using this combined launch to avoid duplicate connections and nodes. It does
not automatically enter TAKEOVER or supply a Manager config. The caller owns
the configuration and hardware output topic mapping:

| Channel | ROS message type | Default override topic |
| --- | --- | --- |
| Chassis | `geometry_msgs/msg/Twist` | `/hexfellow_teleop/cmd_vel` |
| Lift | `sensor_msgs/msg/JointState` | `/hexfellow_teleop/lift_joint_cmd` |

Use the existing `joint_command` channel kind and configure each channel's
`autonomous_topic`, `override_topic`, and `output_topic` in the supplied
Manager file. The included driver launch uses `/chassis/cmd_vel` and
`/lift/joint_cmd` as command inputs and `/lift/motor_states` for lift feedback.
Chassis `simple_mode` is fixed to `true` to match the adapter's Twist output.

`chassis_url` and `lift_url` are required and forwarded to the vendor launch.
`enable_chassis_bridge` and `enable_lift_bridge` default to `true`;
`chassis_is_kcp` and `lift_is_kcp` also default to `true`. Both
`chassis_read_only` and `lift_read_only` default to `false`. These options use
the vendor's names and are passed through without reinterpretation. Disabling
a bridge does not disable its driver adapter; it is intended for an already
running external communication bridge.

`start_pico_bridge` and `start_control_manager` default to `true`. Set them
to `false` when composing this launch with an existing bridge and Manager;
the Hexfellow drivers and teleop adapters still start.

Both `hexfellow_pico_teleop.launch.py` and
`hexfellow_piper_pico_teleop_compat.launch.py` accept these speed arguments:

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `max_linear_x` | `0.2` | Maximum chassis forward/backward speed, m/s |
| `max_angular_z` | `0.2` | Maximum chassis yaw rate, rad/s |
| `position_rate` | `0.08` | Lift target position change rate, m/s |

For example, append `max_linear_x:=0.1 max_angular_z:=0.15 position_rate:=0.04`
to either launch command. These are teleop node startup parameters, not
Control Manager settings; restart the launch to apply changes. Lift
`position_rate` sets the target integration rate, not a hardware velocity
command; feedback lead limits and the driver also constrain actual motion.

Set `replay_time_s: 0` in the caller's Manager config for this initial velocity
control setup. Omitting it enables the Manager's default replay; replay is
currently global, not per channel. No Manager behavior is changed here.

Request global takeover before operating the sticks:

```bash
ros2 service call /robot/control/takeover std_srvs/srv/Trigger '{}'
```

Launch arguments `vr_topic`, `chassis_override_topic`, `lift_override_topic`,
`lift_state_topic`, and `publish_hz` customize the wiring. `vr_topic` remaps
both the bridge output and adapter inputs together. If another robot stack
already provides the bridge and Manager, do not start this whole launch a
second time: run only the two adapter executables below and add their channels
to the existing Manager configuration.

### Hexfellow chassis adapter

`hexfellow_chassis_pico_vr_teleop` reads `/pico_bridge/vr_state` and publishes
`geometry_msgs/msg/Twist` candidates on `/hexfellow_teleop/cmd_vel`. Configure
that topic as a Control Manager channel's `override_topic`; only the Manager
output should reach the hardware driver. `vr_topic` and `cmd_vel_topic` are
ROS parameters. The adapter does not change the Manager mode or start drivers.

```bash
ros2 run robo_orchard_teleop_ros2 hexfellow_chassis_pico_vr_teleop
```

Hold the left stick button for one second to toggle local control, then use
stick Y for forward/backward motion and stick X for yaw. Defaults are 30 Hz,
0.2 m/s maximum linear speed, 0.2 rad/s maximum yaw rate, and a 0.08 deadzone.
`max_linear_x`, `max_angular_z`, `axis_y_sign` (1), `axis_x_sign` (-1),
`publish_hz`, `deadzone`, `toggle_hold_time_s`, and `stale_timeout_s` are
configurable parameters. Inactive control, an invalid left controller, or
0.3 seconds without VR messages produces zero-velocity candidates. Input loss
also clears local activation, requiring another long press.

Manager STOP suppresses forwarding, including zero candidates. The hardware
driver must implement command-timeout stopping; this adapter cannot stop a
robot after the downstream driver or communication process has failed.

### Hexfellow lift adapter

`hexfellow_lift_pico_vr_teleop` shares `/pico_bridge/vr_state` and publishes
`sensor_msgs/msg/JointState` candidates on `/hexfellow_teleop/lift_joint_cmd`.
It reads measured position from `/lift/motor_states`, selecting `joint1` by
name (or the sole position in an unnamed single-joint message). Configure
`joint_cmd_topic` and `state_topic` when using different topic layouts.

```bash
ros2 run robo_orchard_teleop_ros2 hexfellow_lift_pico_vr_teleop
```

Right-stick Y integrates the target at up to `position_rate` (0.08 m/s).
The default `axis_y_sign=-1` makes pushing up decrease position, lifting up.
Targets stay within `min_position=0` and `max_position=0.6` meters and at most
`target_lead_limit=0.015` meters from measured position. Control starts from
feedback, never from a fixed initial target. The lift has no local long-press
activation: input is effective when feedback is valid and Manager is in
TAKEOVER. The adapter itself does not request that mode.

Returning to the deadzone, losing VR input, or an invalid right controller
publishes one hold candidate at the latest measured position after motion.
Missing or stale feedback suppresses all output until fresh feedback arrives,
then reseeds the target. `stale_timeout_s` (0.3 s) applies to both VR and valid
feedback; `publish_hz` (30) and `deadzone` (0.08) are also configurable. The
downstream lift controller remains responsible for safe holding on command or
communication loss.

### Franka managed teleoperation and DAgger

Build and source the dependencies described in the
[Franka driver setup](../robo_orchard_franka_ros2/README.md#dependencies).

Use `franka_pico_teleop.launch.py` for managed teleoperation. It starts the
driver, Pico bridge, Franka IK node, and one Control Manager. Like Piper and
Marvin, the launch remaps the teleop node's driver-facing command topic to a
dedicated override topic using the `/<source>_teleop/joint_<arm>` convention:

| Channel | Topic | Message |
| --- | --- | --- |
| Pico override | `/franka_teleop/joint_franka` | `sensor_msgs/msg/JointState` |
| Model commands (example) | `/inference/joint_franka` | `sensor_msgs/msg/JointState` |
| Manager output to driver | `/robot/franka/joint_cmd` | `sensor_msgs/msg/JointState` |
| Measured joints | `/franka/joint_states` | `sensor_msgs/msg/JointState` |

The Pico launch also accepts `load_gripper`, defaulting to `false`. Setting
it to `true` only starts the official gripper driver and hand model; Pico
does not send gripper commands. Its IK chain remains the seven-joint arm to
`fr3_link8`, and neither Grip nor Trigger commands gripper motion. The
native gripper is controlled independently through the official actions,
not through the Control Manager arm channel.

The arm channel is named `franka` independently of which Pico controller
(`source_type:=left` or `right`) operates it. Command positions are seven
joint angles in radians paired with `fr3_joint1` through `fr3_joint7`.
Only Control Manager output should reach the driver's command topic; neither
Deploy nor Pico should publish directly to it in a managed setup.

Save this configuration in your project and pass its path to the launch:

```yaml
channels:
  - name: franka_arm
    kind: joint_command
    msg_type: sensor_msgs/msg/JointState
    autonomous_topic: /inference/joint_franka
    override_topic: /franka_teleop/joint_franka
    output_topic: /robot/franka/joint_cmd
enable_services: []
reset_services:
  - /robot/franka/reset_ctrl
inference_disable_services: []
replay_time_s: 0.0
```

The driver launch activates the ros2_control controller; no Piper-style
`enable_ctrl` service is assumed. The configuration above is for standalone
teleoperation without a running Deploy node. For DAgger, configure
`inference_disable_services` with the actual Deploy disable service, for
example `/robot/inference_service/disable`. This lets the Manager disable
inference before hardware reset. Configure Deploy's joint command channel
to publish to the matching `autonomous_topic`, with the seven FR3 joint names.
Model and observation mappings remain project-owned.

```bash
ros2 launch robo_orchard_teleop_ros2 franka_pico_teleop.launch.py \
  robot_ip:=${ROBOT_IP} \
  control_manager_config_file:=/path/to/project/control_manager.yaml
```

Use the global Manager services to select command routing:

```bash
ros2 service call /robot/control/takeover std_srvs/srv/Trigger "{}"
ros2 service call /robot/control/auto std_srvs/srv/Trigger "{}"
ros2 service call /robot/control/stop std_srvs/srv/Trigger "{}"
ros2 service call /robot/control/reset std_srvs/srv/Trigger "{}"
```

Pico grip activation only gates local IK output; it does not select Manager
mode. Pico reset gestures request the global Manager reset, not the driver's
reset directly. The teleop node also watches `/robot/control/status`, so
resets requested by an App or another client discard the old pose baseline.
Reset blocks command output and leaves the Manager in `stop`; release and
re-activate Pico to capture a new baseline, then explicitly select the desired
Manager mode. A reset failure does not resume the old teleop session.

Pico input, joint feedback, and end-effector pose must all be recent. The
`stale_timeout_s` launch argument and node parameter defaults to `0.3` seconds,
measured from local receipt using a monotonic clock. Missing or stale input
stops new command generation and discards the teleoperation session. Once all
inputs recover, release and re-activate Pico to capture a fresh baseline.
The one-time flange-to-tool transform is not subject to this timeout.

For composition with a separate engagement coordinator, the Franka node
supports `publish_activation` (default `false`) and `activation_topic`
(default `/teleop/activation/state`). Enable only one activation publisher
for a shared session. It publishes `TeleopActivationState` at the control
rate using the existing Grip hold intent: `INACTIVE` means a real release,
`PENDING` means held during the activation delay, and `ACTIVE` means the
local arm session is active. Missing/invalid/stale input or a blocked held
Grip reports `UNAVAILABLE`, not a release. A Grip held at startup never
produces an `INACTIVE` heartbeat. These heartbeats do not select Manager mode.

The `recapture_service` parameter defaults to `~/recapture_baseline`, using
`robo_orchard_teleop_msg_ros2/srv/RecaptureBaseline`. A request requires fresh
valid Pico input, measured joints and end-effector pose, and no reset in
progress. It recaptures VR/arm baselines, clears pose smoothing and seeds IK
from current measured joints. It neither moves nor activates the arm and
does not clear a release requirement. Service, input and command callbacks
share the default mutually-exclusive callback group, so IK publication
cannot straddle recapture. Success returns the publisher's resolved/remapped
candidate topic and an exclusive nonzero timestamp barrier; all subsequent
candidates have strictly greater stamps, including on repeated clock ticks.
Failure returns an empty topic and zero barrier. Candidate messages remain
ordinary named `JointState` messages. Use the shared non-decreasing ROS clock
domain required by the [interface contract](../robo_orchard_teleop_msg_ros2/README.md).
The arm-only launch defaults and entrypoint remain unchanged; activation
publication is opt-in. The combined launch option below enables it.

Manager `stop` blocks further command forwarding; it is not a hardware
emergency stop and does not cancel an already accepted Ruckig target. Keep
the robot's hardware safety procedures in place.

This provides the command-routing side of DAgger. Run and configure Deploy
and Recorder separately: record observations, commands, and Manager
status/events to distinguish autonomous operation from human intervention.
The launch does not start model inference, recording, or a training loop.

### Franka and Robotiq combined teleoperation

Set `enable_robotiq:=true` on the same launch to add the independent Robotiq
driver, Robotiq Pico source and Engagement coordinator. Franka contains no
Robotiq-specific logic. Grip activates the group; Trigger controls opening.
`load_gripper:=true` and `enable_robotiq:=true` are mutually exclusive and
are rejected before any nodes start. With the default `enable_robotiq:=false`,
no Robotiq driver package is looked up or started.

Build the [Robotiq driver and SDK](../robo_orchard_robotiq_ros2/README.md)
in addition to the Franka dependencies. Copy
[`config/franka_robotiq_manager.example.yaml`](config/franka_robotiq_manager.example.yaml)
into your project and pass that copy explicitly. It has two channels, no
automatic gripper enable/recovery services, and `replay_time_s: 0.0` so cached
autonomous commands are not replayed when switching from AUTO to TAKEOVER.
For DAgger, configure the actual Deploy disable service and both autonomous
command topics in the project copy; the launch does not start inference or
recording.

```bash
ros2 launch robo_orchard_teleop_ros2 franka_pico_teleop.launch.py \
  robot_ip:=${ROBOT_IP} enable_robotiq:=true \
  robotiq_config_file:=/path/to/project/robotiq.yaml \
  control_manager_config_file:=/path/to/project/control_manager.yaml
```

`robotiq_config_file` defaults to empty (the driver's standard 2F-85 YAML).
`robotiq_use_fake:=true` selects the SDK fake **gripper only**; Franka is still
real hardware. This is not a simulation launch. Keep the default joint name
`robotiq_gripper` for this wiring. Custom joint layouts, fingers or namespaces
need project-owned source/driver wiring, not just a serial settings change.
`source_type` selects the same Pico controller for both producers.

`auto_enable:=true` optionally activates Robotiq once at driver startup;
the default is `false`. It has no effect in arm-only mode and does not select
Manager TAKEOVER. Clear the fingers before launching: activation may fully
open/close. Failed activation, stop and reconnection never trigger an
automatic retry or recovery. Hardware readiness stays in the driver;
Engagement still only coordinates matching, baseline recapture and forwarding.

The default
[`config/franka_robotiq_engagement.yaml`](config/franka_robotiq_engagement.yaml)
contains only the two candidate/override routes, the match topic and the
recapture service. Override its path with `engagement_config_file` when using
project-specific routes; update producer topics and Manager wiring together.
The stock launch fixes the producer topics shown below.

| Producer | Candidate to Engagement | Engagement to Manager | Manager to driver |
| --- | --- | --- | --- |
| Franka | `/franka_teleop/candidate` | `/franka_teleop/joint_franka` | `/robot/franka/joint_cmd` |
| Robotiq | `/robotiq_teleop/candidate` | `/robotiq_teleop/joint_robotiq` | `/robotiq/joint_commands` |

Only Manager publishes hardware commands. Do not run another teleop launch,
activation publisher, Manager or direct driver-command publisher alongside
this stack. Engagement consumes `/teleop/activation/state`,
`/robotiq_teleop/match_state`, `/robot/control/status`, and the two candidates;
it calls `/franka_teleop/recapture_baseline`. Watch
`/teleop_engagement/status` for the waiting/blocking reason.

1. Start with Grip released. Check driver status and clear the finger sweep;
   explicitly call `/robotiq/activate` (`std_srvs/srv/Trigger`), unless startup
   activation with `auto_enable:=true` has already succeeded. Activation can
   open/close fully. TAKEOVER does not activate or recover the gripper.
2. Request `/robot/control/takeover`. Hold Grip and adjust Trigger until its
   opening target matches measured opening for the stable-match dwell.
   Neither route forwards while waiting for the match.
3. Engagement recaptures the arm/hand-controller baseline, clears filters and
   reseeds IK. It waits for a post-recapture arm candidate and a fresh gripper
   candidate with its same-stamp positive match before forwarding. Recapture
   failure keeps both routes closed.
4. Once engaged, moving Trigger away from measured opening is normal control,
   not a new matching failure. Release closes both routes. Invalid inputs,
   driver readiness loss, expired heartbeats/candidates or leaving TAKEOVER
   invalidate the session; release and press again after recovery.

Each gate cycle checks both fresh candidates before publishing. The producers
run independently: loss of candidate publication is detected within the
freshness timeout, not synchronously with an IK failure. ROS publications
are not an atomic hardware transaction. Closing the gate or selecting STOP
does not cancel targets already accepted by either driver. For a required
gripper stop, explicitly call `/robotiq/stop`; a later `activate` is required
before commands resume. Manager reset only resets the arm in this example;
never add gripper `recover` unless the workflow permits dropping an object
and sweeping the fingers. Use independent hardware safety procedures.

The stock IK chain ends at `fr3_link8`. Before real operation, configure the
actual Robotiq adapter/TCP transform and payload mass, center of mass and
inertia in the Franka setup. This launch does not supply a calibrated Robotiq
tool model or load values. Fake tests do not validate mounting, collisions,
holding force or physical stop timing.

For a hardware-free end-to-end regression, build/source the message packages,
teleop, Manager and Robotiq driver with its SDK fake support, plus `franka_msgs`
and the teleop Python dependencies. From the repository root, choose an
unused nonzero ROS domain (not one used by a robot) and run:

```bash
ROS_DOMAIN_ID=171 ROS_LOCALHOST_ONLY=1 python3 \
  ros2_package/robo_orchard_teleop_ros2/tests/integration/franka_robotiq_chain.py
```

This test uses the shipped route/Manager configurations with a synthetic
seven-joint arm, synthetic Pico input and the real SDK fake driver. It checks
matching, baseline cutover, rejection of delayed old candidates, release,
STOP/TAKEOVER and held-input recovery. It starts no Franka hardware driver.
Run separately from the stub-based unit tests; it requires real ROS messages.

### Standalone Robotiq Pico source

`robotiq_pico_vr_teleop` is an independent candidate producer for a Robotiq
2F-85 with standard fingers in parallel mode. Start the
[Robotiq driver](../robo_orchard_robotiq_ros2/README.md), inspect its status,
and explicitly activate it following its safety procedure. Then run:

```bash
ros2 run robo_orchard_teleop_ros2 robotiq_pico_vr_teleop
```

The selected Pico controller's Trigger maps linearly to full finger opening
in metres: `(1 - trigger) * 0.085`, zero Trigger fully open, one fully closed.
`source_type` defaults to `right` and accepts `left`. This mapping is not
valid for custom fingers or encompassing mode. The node does not activate
or recover the gripper, select Manager mode, or depend on a Franka arm.
It does not publish activation state. Grip does not gate candidate generation;
a separate engagement coordinator owns shared activation and output gating.

| Parameter | Default |
| --- | --- |
| `vr_topic` | `/pico_bridge/vr_state` |
| `joint_state_topic` | `/robotiq/joint_states` |
| `driver_status_topic` | `/robotiq/status` |
| `joint_command_topic` | `/robotiq_teleop/candidate` |
| `match_state_topic` | `/robotiq_teleop/match_state` |
| `joint_name` | `robotiq_gripper` |
| `publish_hz` | `30.0` |
| `stale_timeout_s` | `0.3` |
| `max_clock_skew_s` | `0.05` |
| `match_tolerance_m` | `0.005` |
| `match_dwell_s` | `0.3` |

Readiness requires valid Trigger in `[0, 1]`, named measured opening in
`[0, 0.085]`, and the driver's `DiagnosticArray` entry with
`hardware_id == joint_name`. Its exact fields must report `online=true`,
`motion_enabled=true`, `operation_busy=false`, `activated=true`,
`activation_state=3` (SDK activation complete), zero `gripper_fault` and
`controller_fault`, and diagnostic level OK. Missing fields are unavailable,
not defaults. Driver status and feedback must both remain fresh.

VR freshness uses local monotonic receipt time, like Franka: the headset
clock need not match ROS time. The Pico Bridge rejects duplicate/regressing
device frames; this source does not validate the VR header stamp. Receipt
freshness detects stream loss, not end-to-end transport delay.
Driver feedback and status still require increasing nonzero ROS header stamps
within the freshness/skew budget as well as fresh local monotonic receipt
times. Replayed, regressing, expired, or future-beyond-skew driver input cannot
renew readiness. Use synchronized non-decreasing ROS clocks for ROS nodes.
Fresh candidates and heartbeats are
producer-stamped with strictly increasing timestamps. Invalid/unavailable
input suppresses candidates and resets match dwell immediately.
Candidates and their match heartbeats share one publication timestamp; the
heartbeat identifies the fully resolved candidate topic. Its retained
`loss_id` changes on observed readiness loss, even if inputs recover before
the next publication, so recovery cannot hide a fault from the coordinator.

`TeleopMatchState` is published periodically even while unavailable.
`ready=true` permits candidate generation; `matched=true` requires the
Trigger target and measured opening to stay within `match_tolerance_m` for
`match_dwell_s`. Mismatch resets dwell but does not suppress otherwise-ready
candidates: matching gates initial engagement in the coordinator, not normal
motion. Commands are single-joint `JointState` messages with empty velocity
and effort. Route candidates through the coordinator and Control Manager;
do not connect this ungated candidate topic directly to hardware. This node
does not itself start the coordinator or a combined launch.

### Engagement coordinator

`teleop_engagement` gates one configured group of ordinary `JointState`
candidate routes together. Sources retain ownership of readiness, input
mapping, matching tolerances and stable-match dwell. Control Manager retains
ownership of mode selection and hardware routing. The coordinator never
calls hardware activation/recovery/reset or selects Manager mode.

Supply a YAML `config_file` with `command_routes` (`input_topic` and
`output_topic` for each route), `match_state_topic` and `recapture_service`.
For example, a generic two-route group is:

```yaml
command_routes:
  - input_topic: /source_a/candidate
    output_topic: /source_a/override
  - input_topic: /source_b/candidate
    output_topic: /source_b/override
match_state_topic: /source_b/match_state
recapture_service: /source_a/recapture_baseline
```

```bash
ros2 run robo_orchard_teleop_ros2 teleop_engagement --ros-args \
  -p config_file:=/path/to/engagement.yaml
```

Names use normal ROS namespace/remapping resolution; duplicate input/output
topics and any overlap between inputs and outputs are rejected after
resolution. Configure Manager override topics to the coordinator's outputs,
not source candidates. The recapture response must identify exactly one
resolved configured input topic.

Node parameters default to `activation_topic=/teleop/activation/state`,
`manager_status_topic=/robot/control/status`, `status_topic=~/status`,
`publish_hz=30.0`, `stale_timeout_s=0.3`, `manager_status_timeout_s=2.5`,
`service_timeout_s=2.0`, and `max_clock_skew_s=0.05`. Manager status uses
its separate 2.5-second timeout for both ROS stamp age and local receipt age,
supporting Control Manager's default 1 Hz heartbeat. Candidate, activation
and match freshness remain 0.3 seconds. `status_topic` is a periodic
`DiagnosticArray`
with the current state (`idle`, `matching`, `recapturing`, `verifying`,
`active`) and human-readable reason, not a separate control interface.

Startup and faults require fresh `INACTIVE` followed by `ACTIVE`.
`PENDING` suppresses output without consuming a previously observed release;
held-at-startup `PENDING` or `ACTIVE` never arms the coordinator.
`UNAVAILABLE`, stale/invalid heartbeats, source readiness loss, candidate
stream loss while engaged, and leaving Manager TAKEOVER close every route
and require another release. Ordinary release also closes all routes; the
next activation must match and recapture again. Initial absence of arm
candidates while input is inactive does not block activation or matching.

With fresh activation, Manager TAKEOVER and source readiness, wait for
`matched=true`, then request baseline recapture asynchronously and clear
cached candidates. A successful response alone does not open the gate:
wait for a fresh candidate strictly above its exclusive `stamp_barrier`
on the identified route and fresh candidates on every other route. Queued
candidates at/below the barrier are discarded throughout that engagement.
The candidate identified by the match heartbeat's `candidate_topic` must
have exactly that heartbeat's stamp before first forwarding. Either topic
may arrive first; until both agree, all routes stay closed. This associates
the initial match with the actual candidate without robot-specific logic or
an additional configuration field.
Candidates arriving before the service response can be retained for
verification but never forwarded early. The service/candidate cutover must
complete within `service_timeout_s`; cancelled or obsolete replies cannot
restore an attempt. Initial match loss during cutover invalidates it.
After engagement, `matched=false` does not close the gate: normal commanded
motion may differ from feedback, while readiness must remain true.

All input heartbeats and candidates require increasing nonzero producer
ROS stamps within the freshness/skew budget plus fresh local monotonic
receipt times. Replayed/regressing/expired/future-beyond-skew deliveries
cannot renew freshness. Candidates require finite named JointState data;
the coordinator does not interpret joint names or units. Forwarded messages
preserve source content and stamps. Use synchronized non-decreasing ROS
clocks; engagement cannot survive a clock reset or guarantee transparent
producer restart detection. This executable does not start drivers, sources,
Manager or a combined launch.

### Other robots

Each command starts a complete stack: robot driver, input bridge and teleop
node. Append `--show-args` to any launch file to list its parameters.

Aloha, Piper:

```bash
ros2 launch robo_orchard_teleop_ros2 piper_aloha_compat.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json
```

Piper runtimes start in the global `STOP` state. Request takeover before
moving an Aloha leader, or request auto before starting autonomous output:

```bash
ros2 service call /robot/control/takeover std_srvs/srv/Trigger '{}'
```

The Piper drivers intentionally start with `auto_enable_arm_ctrl` enabled.
This establishes their hardware-side ready state; it does not open a command
route. In particular, takeover and reset may be requested directly from
`STOP`, and neither transition runs the Manager's Auto enable sequence. While
the Manager remains in `STOP`, it publishes no driver commands even though
the drivers are ready. A later Auto request still calls every configured
`enable_ctrl` service, which returns success when a driver is already ready.

Mainline Piper launches also broadcast follower end-effector TF by default:
`left_base_link -> left_end_effector` and
`right_base_link -> right_end_effector`. Override the frames with
`left_base_frame_id`, `left_ee_frame_id`, `right_base_frame_id`, and
`right_ee_frame_id`, or set `publish_ee_tf:=false` to keep only pose topics.
In four-arm Aloha/Dagger launches, master TF is disabled by default; the
`left_master_*` / `right_master_*` frame arguments and `publish_master_ee_tf`
configure it separately. Match these frame names to the hand-eye calibration
configuration. See the [Piper package README](../robo_orchard_piper_ros2/README.md)
for the complete parameter contract. The separate Raw Aloha launch is unchanged.

Pico VR, Hexfellow with dual Piper:

```bash
ros2 launch robo_orchard_teleop_ros2 hexfellow_piper_pico_teleop_compat.launch.py \
  left_can_port:=can_left \
  right_can_port:=can_right \
  chassis_url:=192.0.2.10:8439 \
  lift_url:=192.0.2.11:8439 \
  control_manager_config_file:=/path/to/project/control_manager.json
```

This replaces the former `hexfellow_dual_piper_pico_teleop.launch.py`
entrypoint; update startup commands to the new name. Launch arguments and
behavior are unchanged, and no alias for the former name is provided.

This composes the existing dual Piper Pico launch and Hexfellow launch,
including the lift. The Piper launch owns the single Pico bridge and Control
Manager; the Hexfellow copy of these two nodes is disabled. Do not also run
either standalone launch or separately started drivers. Both included
launches inherit caller arguments, including CAN ports, device connection
options, `urdf_path`, joint names, reset positions, and frame settings.
`vr_topic` defaults to `/pico_bridge/vr_state` and rewires the shared bridge
and all three teleop adapters together.

Supply one Manager configuration covering all four channels. In addition to
the Hexfellow mappings above, map `/pico_teleop/joint_left` and
`/pico_teleop/joint_right` (`sensor_msgs/msg/JointState`) to
`/robot/left/joint_cmd` and `/robot/right/joint_cmd`, respectively. Keep
`replay_time_s: 0` for the chassis velocity setup. Startup remains in STOP;
request TAKEOVER explicitly. Piper reset gestures address the same global
Manager reset service; configure reset participants in the caller's file.
Piper controller poses/triggers operate the arms/grippers, while the sticks
operate the chassis/lift; the existing activation rules remain unchanged.

Pico VR, Piper:

```bash
ros2 launch robo_orchard_teleop_ros2 piper_pico_teleop_compat.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json
```

Pico VR, Marvin:

```bash
ros2 launch robo_orchard_teleop_ros2 marvin_pico_teleop.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json
```

Wuji glove and hand:

```bash
ros2 launch robo_orchard_teleop_ros2 wuji_glove_teleop_compat.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json \
  hand_side:=left,right \
  hand_name:=hand_left,hand_right \
  hand_serial_number:=<left-hand-serial>,<right-hand-serial> \
  glove_serial_number:=<left-glove-serial>,<right-glove-serial>
```

Every glove argument takes one comma-separated entry per instance. Serial
numbers are required once both sides are connected, because handedness alone
no longer identifies a device.

The launch starts one Control Manager. Glove commands enter its override
channels and only Manager output reaches each Wuji Hand command topic. The
project must supply a matching
`control_manager_config_file` with its hand names
and algorithm topics. No Wuji Hand reset service is assumed.

Marvin with the Wuji hand:

```bash
ros2 launch robo_orchard_teleop_ros2 marvin_wuji_keyboard_teleop.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json \
  keyboard_config_file:=$HOME/teleop_keyboard.yaml \
  hand_side:=left,right \
  hand_name:=hand_left,hand_right \
  hand_serial_number:=<left-hand-serial>,<right-hand-serial> \
  glove_serial_number:=<left-glove-serial>,<right-glove-serial>
```

One command for the whole rig: the arms follow Pico, the hands follow the
gloves, and the driver profile defaults to the one carrying the hand's tool
dynamics. Activation is pinned to the keyboard, so `keyboard_config_file` is
required — see below.

Prefix any of these with `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 taskset -c
<performance-cores>`:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 taskset -c 0-7 \
  ros2 launch robo_orchard_teleop_ros2 marvin_pico_teleop.launch.py \
    control_manager_config_file:=/path/to/project/control_manager.json
```

`taskset` keeps the stack on the performance cores. The control loop is
latency-sensitive and drifts when the scheduler moves it onto efficiency
cores. The core numbers depend on the machine.

The thread-count variables keep NumPy's BLAS pool from spinning about four
cores, which drags the 30 Hz control loop down to 20 Hz.

## Piper feedback TF for calibration validation

The `piper_control_compat`, `piper_dagger_compat`, `piper_pico_dagger_compat`,
and `piper_pico_teleop_compat` launches broadcast follower end-effector TF
by default: `left_base_link -> left_end_effector` and
`right_base_link -> right_end_effector`. This supplies the dynamic pose edge
needed for reprojection validation of hand-eye calibration results.

Override the frames with `left_base_frame_id`, `left_ee_frame_id`,
`right_base_frame_id`, and `right_ee_frame_id`, or set `publish_ee_tf:=false`
to keep only pose topics. In the four-arm Dagger launch, master TF is disabled
by default; the `left_master_*` / `right_master_*` frame arguments and
`publish_master_ee_tf` configure it separately. Match these frame names to
the hand-eye calibration configuration. See the
[Piper package README](../robo_orchard_piper_ros2/README.md) for the complete
parameter contract. Ordinary Aloha (`piper_aloha_compat.launch.py`) delegates
to the managed four-arm DAgger launch, requires `control_manager_config_file`,
and starts in `STOP`; explicitly select `TAKEOVER` to route master commands.
Only Raw Aloha retains its separate low-level drivers and existing control
and reset behavior without a Control Manager.

## Engaging teleop

Teleop drives the robot only while the operator engages it. Two input sources
are available, selected with `operator_input_source`:

- `pico` (default) — hold the grip button on the Pico controller.
- `keyboard` — hold `T` for one second to enable local arm command output and
  release it to stop that output; hold `R` for one second to request a global
  reset.

Keyboard input replaces engage and reset only. The pose source is unchanged,
so the Pico headset still has to be running.

Driving an arm and a dexterous hand at the same time turns this into a
requirement rather than a preference. The glove teleoperates the hand while
Pico teleoperates the arm, which leaves the operator with no free hand for the
Pico grip button, so `keyboard` becomes the only usable source. A foot pedal
that enumerates as a keyboard suits this setup and keeps both hands free.
`marvin_wuji_keyboard_teleop.launch.py` pins the source to `keyboard` for this
reason and starts the keyboard node itself.

The keyboard is described by a configuration file. The shipped example is a
template — the node refuses to start until `device.vendor_id`,
`device.product_id` and `device.interface_number` identify the intended
`/dev/input/event*` interface. Keep your copy outside the package so a real
device identity never lands in the repository:

```bash
cp config/teleop_keyboard.example.yaml $HOME/teleop_keyboard.yaml
```

Start the keyboard node next to the teleop stack:

```bash
ros2 launch robo_orchard_teleop_ros2 teleop_keyboard.launch.py \
  config_file:=$HOME/teleop_keyboard.yaml

ros2 launch robo_orchard_teleop_ros2 marvin_pico_teleop.launch.py \
  control_manager_config_file:=/path/to/project/control_manager.json \
  operator_input_source:=keyboard \
  keyboard_control_side:=both
```

The node keeps running and keeps publishing even when no keyboard matches the
configuration, so check that it found the device:

```bash
ros2 topic echo /teleop/activation/state   # 1 = idle, 2 = engaged
```

A `state` stuck at `0` means the device was never matched — re-check the
vendor, product and interface numbers.

## DAgger take-over

The Piper and Wuji `*_dagger_compat.launch.py` files start one Control Manager
that routes policy commands as autonomous input and teleoperation commands as
override input. Pico grips, triggers, and keyboard or pedal activation only
gate local teleoperation output; they do not change the Control Manager mode.
Callers such as the frontend switch the global mode explicitly through the
Control Manager services. Pico reset gestures still request the global reset.
Piper and Marvin Pico nodes also subscribe to `/robot/control/status` with a
reliable queue of depth 10. Every observed `resetting` transition invalidates
both local teleop sessions and their pose baselines, including resets requested
by the App or another service client. Repeated reset heartbeats do not restart
the local reset. Output stays blocked until the Manager leaves `resetting`
and any locally requested reset returns, including failure responses. Held
Pico or keyboard activation cannot resume the old session: release and
re-activate to capture a new baseline. Reset leaves the Manager in `stop`;
hardware enablement and selecting `takeover` remain separate operations.
Wuji replay time is configured in the project Manager configuration; the old
`replay_time_s` launch argument is no longer used.

Handing over a gripper needs care, because the policy leaves it at an
arbitrary opening. On Piper the VR trigger drives the gripper — released is
open, squeezed is closed — so teleop does not engage the moment the operator
asks for it. It waits until the trigger maps to within `match_tolerance`
(default `0.1`, a tenth of the full travel) of the gripper's current opening,
which keeps the gripper from jumping on hand-off.

## Combinations without a ready-made launch

The launch files above cover the combinations in use today. Anything else —
another robot paired with the gloves, or keyboard activation for a stack that
does not start the keyboard node itself — is not provided. Either start the
extra launch files alongside each other as shown above, or copy the closest
launch file and add the nodes you need.


## Project-owned control configuration

Franka FR3 uses `franka_pico_teleop.launch.py` with the same required
`control_manager_config_file` argument. Its managed override topic is
`/franka_teleop/joint_franka`, carrying seven named joint positions in
`sensor_msgs/msg/JointState`, following the Piper/Marvin
`/<source>_teleop/joint_<arm>` convention. See the
[Franka managed teleoperation and DAgger](#franka-managed-teleoperation-and-dagger)
for project wiring, reset behavior, and DAgger setup.

Managed robot launches require `control_manager_config_file:=<path>`.
The Control Manager package provides validation and routing, not robot
profiles. HoloBrain's `teleop/aloha_dagger.sh` and `teleop/pico_dagger.sh`
already generate and pass the Piper configuration using
`projects/HoloBrain/teleop/gen_control_manager_config.py`.
Other projects supply their own command topics and lifecycle services.
Pure collection configurations must omit inference disable services when
Deploy is not launched.
