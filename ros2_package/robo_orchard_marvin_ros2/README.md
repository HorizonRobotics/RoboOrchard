# RoboOrchard Marvin ROS 2

This package provides a ROS 2 Driver node for the Tianji/Fuxi Marvin dual-arm
robot. It wraps the official control and kinematics C++ SDKs and exposes joint
commands, state feedback, mode switching, error clearing, and software
emergency stop interfaces.

## Build

Initialize the pinned SDK submodule, then use the repository build entry:

The Driver builds the official `contrlSDK100343` and `kinematicsSDK` sources
for the control-system version 100343 family.

At startup, the Driver reads the controller's `VERSION` parameter and compares
its version family with the linked SDK using the vendor rule
`controller_version / 1000 == sdk_version / 1000`. Failure to read the version
or a family mismatch releases the SDK connection and terminates startup.

```bash
git submodule update --init --recursive
make ros2-build
source ros2_package/install/setup.bash
```

## Launch

### Default Startup

The default launch only connects to the controller and does not enable either
arm:

```bash
ros2 launch robo_orchard_marvin_ros2 marvin_driver.launch.py
```

### Control Modes

The Driver uses the following control modes for both startup auto-enable and
the per-arm `set_mode` service:

| Mode | Value | `auto_enable_mode` |
| --- | ---: | --- |
| Idle / disabled | `0` | Not applicable |
| Position control | `1` | `position` |
| Joint impedance | `2` | `joint_impedance` |
| Joint drag | `3` | `joint_drag` |

Use mode `0` to disable an arm. Switching directly between enabled modes is
supported. Joint drag mode still requires holding the physical drag button.

### Optional Startup

Optional launch arguments are:

- `config_file`: parameter file; defaults to `config/marvin_m6s.yaml`.
- `auto_enable_side`: `none` (default), `left`, `right`, or `both`.
- `auto_enable_mode`: `position`, `joint_impedance`, or `joint_drag`; required
  when `auto_enable_side` is not `none`.

For example, start with the right arm enabled in joint impedance mode:

```bash
ros2 launch robo_orchard_marvin_ros2 marvin_driver.launch.py \
  auto_enable_side:=right auto_enable_mode:=joint_impedance
```

### Runtime Mode Change

For example, switch the right arm to joint impedance mode:

```bash
ros2 service call /robot/right/set_mode \
  robo_orchard_marvin_msg_ros2/srv/SetControlMode "{mode: 2}"
```

## Interfaces

Each arm uses `/robot/<side>/joint_cmd` and `/robot/<side>/joint_state` with
`sensor_msgs/msg/JointState`. The measured joint feedback is also converted by
the official kinematics SDK and published on `/robot/<side>/ee_pose` as
`geometry_msgs/msg/PoseStamped` in the fixed `robot_stand` frame. Driver state
is published on `control_mode`, `controller_state`, `impedance_type`, and
`error_code`. The services `reset_ctrl`, `clear_error`, and `emergency_stop`
are also provided per arm.

`ee_pose` represents the bare flange pose from the SDK kinematic model. The
driver does not configure or apply the controller's active Tool/TCP offset
(`m_ToolKine`). The vendor control application may automatically restore a
previously selected Tool when it connects, so its displayed Cartesian pose can
differ from `ee_pose` by that Tool transform. Clear and apply a zero Tool before
comparing flange poses, or account for the configured Tool transform when
comparing against a TCP pose.

The per-arm `reset_ctrl` services block until the requested arm reaches its
configured reset position or the operation fails. Left and right requests may
run concurrently:

```bash
ros2 service call /robot/right/reset_ctrl std_srvs/srv/Trigger '{}'
```

The aggregate service validates both arms before moving either one, starts both
trajectories with the same timestamp, and blocks until both complete:

```bash
ros2 service call /robot/reset_ctrl std_srvs/srv/Trigger '{}'
```

If either arm fails during an aggregate reset, the peer reset is aborted and
the response reports both results.

Reset is accepted only in position or joint impedance mode. It is rejected in
idle or joint drag mode, while the requested arm is moving, or while that arm
already has a reset in progress. The reset trajectory uses smooth joint-space
interpolation and does not perform collision planning.

Motion parameters are configured in `config/marvin_m6s.yaml`. In particular,
`velocity_ratio` and `acceleration_ratio` use the SDK percentage range
`0..100`; the SDK applies `1` when either value is `0`. For teleoperation, `50`
is a conservative starting value for both parameters and they can be adjusted
according to operator preference. The validated configuration currently uses:

```yaml
velocity_ratio: 80
acceleration_ratio: 80
```

Higher values generally improve teleoperation responsiveness, but also produce
faster and more aggressive motion. Reset positions and
`reset_goal_tolerance_rad` are expressed in radians. Reset speed is controlled
separately by `reset_duration_s`: a smaller duration produces a faster reset,
while `reset_timeout_s` only bounds the blocking request.

The Driver logs the ToolDyn values reported by both arms whenever it connects.
When `apply_tool_dynamics_on_startup` is enabled, it preserves each arm's
current six-value ToolKine, applies the configured `left_tool_dynamics` and
`right_tool_dynamics`, and verifies the controller readback before any optional
auto-enable. Startup fails if either arm cannot apply or verify its parameters.
The ten ToolDyn values use the vendor ordering
`[m, mx, my, mz, ixx, ixy, ixz, iyy, iyz, izz]` without unit conversion.
The default `marvin_m6s.yaml` leaves ToolDyn application disabled with zero
values. For the calibrated Wuji hands, select the dedicated configuration:

```bash
ros2 launch robo_orchard_marvin_ros2 marvin_driver.launch.py \
  config_file:=$(ros2 pkg prefix robo_orchard_marvin_ros2)/share/robo_orchard_marvin_ros2/config/marvin_m6s_with_wujihand.yaml
```

Only one process may own the Marvin SDK connection. Confirm the robot workspace
is clear and start with conservative speed and acceleration ratios.
