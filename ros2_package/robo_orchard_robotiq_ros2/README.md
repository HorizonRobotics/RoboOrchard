# Standalone Robotiq 2F-85 driver

An `ament_cmake` C++17 node using the separately installed official Robotiq
`grippers` SDK **1.1.0**, not `ros2_control` or a ROS action driver. This
package controls one serial 2F-85 with standard fingers in parallel mode.
It remains independent of Pico and Franka; optional
[combined teleoperation](../robo_orchard_teleop_ros2/README.md#franka-and-robotiq-combined-teleoperation)
is owned by the teleop package. No automatic activation or fault recovery
occurs in the driver.

## Install and build

Supported baseline: Ubuntu 22.04, ROS 2 Humble, CMake 3.16+, C++17. Install
the SDK separately; this package does not download or vendor SDK sources.
The tested compatible release is
[v1.1.0](https://github.com/robotiq/grippers/releases/tag/v1.1.0), required
exactly by CMake. Hosted serial and fake support must stay enabled.

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libserialport-dev \
  ros-humble-diagnostic-msgs ros-humble-ament-cmake-gtest
git clone --branch v1.1.0 --depth 1 https://github.com/robotiq/grippers.git
cmake -S grippers/sdk_cpp -B grippers-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DGRIPPERS_BUILD_TESTS=OFF -DGRIPPERS_BUILD_EXAMPLES=OFF \
  -DGRIPPERS_HOSTED=ON -DGRIPPERS_BUILD_DEFAULT_SERIAL=ON \
  -DGRIPPERS_BUILD_FAKE=ON
cmake --build grippers-build -j
sudo cmake --install grippers-build
```

SDK install exports headers, library and `grippers-config.cmake` with
`Robotiq::grippers`; Threads and libserialport are resolved by the exported
target. For a non-system prefix, add it to `CMAKE_PREFIX_PATH`.
From the RoboOrchard repository root, use the repository's build/test targets
with package selection (other robot SDKs are not needed):

```bash
source /opt/ros/humble/setup.bash
make ros2-build extra_args="--packages-select robo_orchard_robotiq_ros2"
make ros2-test extra_args="--packages-select robo_orchard_robotiq_ros2"
source ros2_package/install/setup.bash
```

For Docker, use the public `ros:humble` base, install the apt dependencies
above, then build/install the pinned SDK before the ROS package. No project
Dockerfile change is necessary. Pass only the selected serial device with
`--device=/dev/ttyUSB0`; give the container user the host device's group via
`--group-add`, rather than running privileged. SDK FTDI latency configuration
may require host-side setup; do not grant broad sysfs write access to a
container. On the host, the user needs serial-device access (usually dialout).

## Bring-up and validation

Use the SDK fake first; it does not access hardware:

```bash
ros2 launch robo_orchard_robotiq_ros2 robotiq_driver.launch.py use_fake:=true
ros2 topic echo /robotiq/status
ros2 topic echo /robotiq/joint_states
ros2 service call /robotiq/activate std_srvs/srv/Trigger '{}'
ros2 topic pub --once /robotiq/joint_commands sensor_msgs/msg/JointState \
  '{name: [robotiq_gripper], position: [0.04]}'
ros2 service call /robotiq/stop std_srvs/srv/Trigger '{}'
```

For real hardware, first verify the actual **24 V** supply, FTDI RS485 wiring,
slave 9 and 115200 8N1 configuration. Clear the full finger sweep, remove any
held object, and have an independent safe hardware shutdown available. Start
only when already stopped: SDK construction can briefly resend an inherited
target before this node confirms stop. Use a stable serial-device path in a
copy of `config/robotiq_2f85.yaml`, then launch with
`config_file:=/path/to/robotiq.yaml` and default `use_fake:=false`.
Inspect online/fault status before explicitly activating. Activation and
recovery can open/close fully; they are not safe grip-preserving operations.
Begin with low speed/force and a small opening change, not full closure.

`auto_enable` defaults to `false`. To activate once during driver startup,
pass `auto_enable:=true` to `robotiq_driver.launch.py`. This uses the same
activation handshake as `/robotiq/activate`; a healthy already-active device
skips calibration. Clear the finger sweep **before launching**, because an
inactive device may fully open/close. Commands remain disabled until the
handshake succeeds. Failure leaves the node running but disabled, with an
error log and status available for diagnosis. There is no automatic retry,
fault recovery, or re-enable after stop/disconnection. Launch arguments
override the corresponding YAML values.

`recover` is an explicit reset-and-reactivate operation, not return-to-home:

```bash
# Only after clearing the fingers and diagnosing the fault:
ros2 service call /robotiq/recover std_srvs/srv/Trigger '{}'
```

The non-hardware tests cover conversions, cache freshness, fake handshakes,
zero-timeout cleanup, and ROS service/command/state behavior. A zero-timeout
test does not inject a disconnect or exercise SDK mid-handshake restoration.
Fake tests cannot validate physical timing, object detection, holding force or
link-loss behavior. Hardware acceptance must include: safe startup/activation,
small moves and stop, status current/fault/object codes, unplug/replug with
no automatic command resumption, and controlled shutdown. Do not perform
disconnect/recovery tests while holding anything that can fall or cause harm.

## Control Manager wiring

`config/control_manager.example.yaml` shows a normal generic JointState
channel and `activate` in `enable_services`. Projects own launch/config
wiring; no special gripper branch is needed in Control Manager. It is not
loaded by the standalone launch. Keep producers consistent with the joint
name/metre convention and ensure only Manager publishes to the driver command
topic. Direct publishers are suitable only for isolated bring-up.

Manager STOP disables forwarding; it **does not call this driver's stop and
does not cancel an already issued move**. Call `/robotiq/stop` explicitly when
that is required. A stopped driver rejects commands until `activate`.
Manager reset has an empty downstream reset list in this example: it does
not reset or calibrate the gripper. Do not map `stop` as a fake hardware-reset
service. Projects may intentionally put `/robotiq/recover` in `reset_services`
only if their reset workflow is allowed to release an object and sweep the
fingers, and their service timeout covers the configured activation timeout
plus two stop-exchange waits. A Manager AUTO transition invokes
`enable_services` and may activate/calibrate when the gripper is not already
active. TAKEOVER does not invoke `enable_services` and does not re-enable a
stopped driver.

## ROS contract

All names below are relative to the node namespace.

| Name | Type | Contract |
|---|---|---|
| `joint_commands` | `sensor_msgs/msg/JointState` | Exactly one name (`robotiq_gripper` by default) and one position: full finger opening in metres, 0 closed, 0.085 open. Empty velocity/effort required. Invalid commands are rejected. |
| `joint_states` | `sensor_msgs/msg/JointState` | Approximate measured parallel opening, from new live SDK exchanges only. Velocity/effort empty. Stamp estimates reception, not a device clock. |
| `status` | `diagnostic_msgs/msg/DiagnosticArray` | Online/freshness, enabled/busy, activation, raw fault codes, approximate motor current in A, gGTO and raw object detection (unknown when gGTO is false). Offline fields are omitted. |
| `activate` | `std_srvs/srv/Trigger` | Explicit activation; may fully open/close. Healthy already-active device skips calibration but is stopped before accepting commands. |
| `recover` | `std_srvs/srv/Trigger` | Explicit reset (clear rACT) followed by activation/calibration; may release an object. Never an automatic fallback. |
| `stop` | `std_srvs/srv/Trigger` | Disable command acceptance and clear rGTO; retain rACT, target and force settings. Not emergency auto-release, electrical isolation or a safety-rated stop. |

Services acknowledge completed SDK handshakes/exchanges, not future motion
completion. A dedicated SDK command writer handles them without blocking the
state publisher. Concurrent service requests return busy, including `stop`
during activation/recovery: stop cannot interrupt the calibration sweep.
The SDK starts exchanging during construction and may briefly resend an
inherited motion target with SDK-default speed/force before the node can set
its initial stop. Start only with the device already stopped and safely clear;
the node confirms its stop before exposing command/services. It stops before
and after activation/recovery, including failures, so a timed-out recovery
cannot restore an old moving command image.
Commands are latest-target streaming commands, not queued
trajectories. Commands received while disabled, offline or during a service
operation are dropped. Parameters are startup-only; do not change them at runtime.

Disconnected/stale status disables commands and replaces the cached motion
with a stop. The SDK continues attempting exchanges, but the node never
automatically re-enables or recalibrates after reconnection, even with
`auto_enable=true`. Call `activate` explicitly after checking the hardware;
activation can release a grip if no longer active.
Connection/initial-stop failure exits rather than repeatedly opening the port.
On shutdown the node attempts a bounded stop without releasing or deactivating;
without a reply the hardware state remains unknown. Software cannot ensure an
object remains held after link loss, power loss or a hardware fault.

The SI mapping is approximate and invalid in encompassing mode or with custom
fingers. `force_fraction` (default 0.1, range 0..1) sets the SDK force register,
not newtons or calibrated force control. `speed_mps` defaults to 0.05 (allowed
0.02..0.15). JointState effort is deliberately not motor current or force.

Other startup parameters: `port=/dev/ttyUSB0`, `baud_rate=115200`,
`slave_address=9`, `serial_timeout_ms=500`, `exchange_rate_hz=100`,
`state_rate_hz=30`, `status_timeout_ms=1000`, `activation_timeout_ms=15000`,
`joint_name=robotiq_gripper`, `use_fake=false`, `auto_enable=false`.
Timeouts must be positive; activation timeout is at most 60 seconds.
SDK uses 8N1, an FC 0x17 combined
read/write exchange, and its default FTDI latency setting. Only one process
may own the serial link: do not run mbpoll or another driver concurrently.

The SDK must include hosted serial and fake support (default build options).
`use_fake:=true` requires no serial device; fake tests verify the API and
handshake but not timing, grip forces, fault injection or physical safety.
