# RoboOrchard Teleop ROS 2

Teleoperation for RoboOrchard robots. The package turns operator input into
robot commands, and provides the take-over (DAgger) plumbing used during data
collection.

## Teleoperation modes

- **Aloha leader-follower** — a leader arm drives the follower over CAN.
  Piper only.
- **Pico VR** — a Pico headset streams controller poses and the arm follows
  through IK for Piper and Marvin; Hexfellow uses the controller sticks for
  chassis velocity and lift position. The bridge reads raw device messages, which
  the upstream `xrobotoolkit_sdk` does not expose, so build
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

Managed robot launches require `control_manager_config_file:=<path>`.
The Control Manager package provides validation and routing, not robot
profiles. HoloBrain's `teleop/aloha_dagger.sh` and `teleop/pico_dagger.sh`
already generate and pass the Piper configuration using
`projects/HoloBrain/teleop/gen_control_manager_config.py`.
Other projects supply their own command topics and lifecycle services.
Pure collection configurations must omit inference disable services when
Deploy is not launched.
