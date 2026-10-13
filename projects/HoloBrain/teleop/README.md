The default HoloBrain teleoperation setup targets ALOHA hardware. Pico VR
uses a separate DAgger entry point.

| Script | Purpose |
| --- | --- |
| `aloha_dagger.sh` | ALOHA DAgger control through the global manager. |
| `pico_dagger.sh` | Pico VR DAgger control through the global manager. |
| `hexfellow_pico_dagger.sh` | Pico VR with dual Piper arms and Hexfellow chassis/lift through one global manager. |

All launches start exactly one Control Manager. To customize topics, reset
scope, or replay behavior, pass `control_manager_config_file:=<path>` with a
validated Control Manager configuration.

For the default generated configuration, edit the command topics,
`enable_services`, `reset_services`,
`inference_disable_services`, `inference_node_candidates`, and `replay_time_s`
values in `gen_control_manager_config.py`. The `aloha` and `pico` branches list their
hardware service scopes explicitly. Model-server fields belong to the
inference configuration, not the Manager's runtime logic.

For the combined robot, set `TELEOP_SOURCE=pico` and `WITH_HEXFELLOW=true` in
`../launch/launch.yaml`, and supply `PIPER_URDF_PATH`, `CHASSIS_URL`, and
`LIFT_URL` there. The template defaults to `WITH_HEXFELLOW=false`, preserving
the ALOHA and Piper-only Pico launches. ALOHA cannot run with Hexfellow.
Alternatively, run `hexfellow_pico_dagger.sh` directly with these three
environment variables (or pass the Piper URDF path as its first argument).
The chassis and lift addresses are forwarded to the vendor driver launch;
install its external `hex_bridge`, `hex_device_msgs`, and
`hex_device_ros_wrapper` packages before starting the combined robot. Stop
other Hexfellow driver instances first to avoid duplicate connections.

The generated combined configuration routes the two arm `JointState`
channels plus chassis `Twist` and lift `JointState` channels:

| Channel | Autonomous input | Pico override | Driver output |
| --- | --- | --- | --- |
| chassis | `/chassis_algo_cmd` | `/hexfellow_teleop/cmd_vel` | `/chassis/cmd_vel` |
| lift | `/lift_algo_cmd` | `/hexfellow_teleop/lift_joint_cmd` | `/lift/joint_cmd` |

The Hexfellow autonomous inputs are reserved for future Deploy channels;
existing Deploy configs publish only arm commands. The configuration uses
`replay_time_s=0` (including the Piper-only paths) so velocity control does
not replay stale chassis commands. Reset services remain the two Piper arm
services; the Hexfellow vendor drivers have no corresponding configured
reset service. Request `/robot/control/takeover` before driving the robot.

The data recorder service uses the same `TELEOP_SOURCE` and
`WITH_HEXFELLOW` environment switches. Pico sessions capture VR input and
both routed Piper commands; combined sessions also capture Hexfellow states
and routed chassis/lift commands. See the data collection documentation for
the recording readiness and timestamp rules.

`REPLAY_TIME_S` is no longer a launch override and is ignored by these
scripts. Set `replay_time_s` in the selected Control Manager configuration
instead.

## Dependency setup

Run in Docker with the RoboOrchard virtual environment activated and
`ROBO_ORCHARD_PATH` set, before compiling ROS2 packages. These instructions
target Ubuntu 22.04 amd64. Older images need `xdg-utils` for Pico and
Rust/Cargo, Clang/libclang, and protoc 27.1 for Hexfellow; the updated
[Dockerfile](../dev/Dockerfile) provides them.

### Pico VR

Install the PC Service and patched Python bindings:

```bash
mkdir -p "$ROBO_ORCHARD_PATH/../deps"
cd "$ROBO_ORCHARD_PATH/../deps"
curl -fL -o XRoboToolkit_PC_Service.deb \
  https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service.deb
git clone --branch v1.0.2 \
  https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git
cd XRoboToolkit-PC-Service-Pybind
cp "$ROBO_ORCHARD_PATH/ros2_package/robo_orchard_teleop_ros2/patches/py_bindings.cpp" \
  bindings/py_bindings.cpp
python -m pip install pybind11
bash setup_ubuntu.sh
python -c 'import xrobotoolkit_sdk as xrt; assert hasattr(xrt, "get_latest_message")'
```

If dpkg reports missing dependencies, run `sudo apt-get --fix-broken install`.
Keep the Pybind checkout, including `lib/`, on a host-mounted volume; the
installed extension needs that SDK library at runtime.

Connect the headset using the [official instructions](https://github.com/XR-Robotics/XRoboToolkit-PC-Service).
The launch template starts `/opt/apps/roboticsservice/runService.sh` for Pico;
when launching a teleop script directly, run it with `bash` first.

### Hexfellow chassis/lift

Clone the ROS packages with their protocol submodules and install the Python
SDK dependencies:

```bash
mkdir -p "$ROBO_ORCHARD_PATH/ros2_package/third_party/hexfellow"
cd "$ROBO_ORCHARD_PATH/ros2_package/third_party/hexfellow"
git clone --recursive https://github.com/hexfellow/hex_bridge.git
git clone --recursive https://github.com/hexfellow/hex_device_msgs.git
git clone --recursive https://github.com/hexfellow/hex_device_ros_wrapper.git
python -m pip install -r hex_device_ros_wrapper/requirements.txt
```

Follow the [vendor instructions](https://github.com/hexfellow/hex_device_ros_wrapper)
for device setup. The combined launch starts these drivers; do not run a
second driver instance against the same hardware.

## Reset with an optional Deploy node

The default Manager configuration lists the fully qualified sync and async
Deploy node names in `inference_node_candidates`. These are alternative names
for the separately launched inference participant, not nodes the Manager
starts itself. Update them when changing the Deploy node name or namespace.

On every accepted reset, Manager first enters `RESETTING` and gates commands.
It then checks the currently discovered nodes and inference-disable services
without waiting for Deploy to appear. If neither a candidate node nor a
configured disable service is discovered, it logs the skip and immediately
proceeds to the configured hardware resets.
If either is discovered, **all** `inference_disable_services` must succeed,
even if inference is already paused. An unavailable service, failed response,
or response timeout blocks hardware reset. The Manager ends in `STOP` on both
success and failure; it does not restart inference or resume takeover.
`service_wait_timeout_s` still applies when waiting for a required service,
not when checking whether optional Deploy is present.

An empty or omitted `inference_node_candidates` list keeps all configured
disable services mandatory, preserving existing Manager configurations. Use
that strict setting when Deploy must be present. Discovery only describes the
local ROS graph; it cannot prove a remote process is stopped during a network
partition or before discovery has completed. Do not start Deploy concurrently
with reset; use strict configuration if its presence must be guaranteed.
Presence is checked again on each reset, so a later-started Deploy is not
permanently skipped. Configuring candidate names requires at least one
disable service.

This logic belongs to Manager, so reset behaves the same whether requested by
the App, Pico, keyboard, or another service client. Regenerate the Manager
configuration and restart the Manager to apply these settings.
