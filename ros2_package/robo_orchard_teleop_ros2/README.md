# RoboOrchard Teleop ROS 2

Teleoperation for RoboOrchard robots using leader arms, Pico VR controllers,
or Wuji gloves. The package also provides teleoperation input for DAgger.

## Teleoperation modes

- **Aloha leader-follower** — control a Piper follower from a leader arm.
- **Pico VR** — control arm poses and grippers; use the controller sticks
  for Hexfellow chassis motion and lift position.
- **Wuji glove** — control Wuji Hand1 or Wuji Hand2.

| Robot | Aloha | Pico VR | Wuji glove |
| --- | :---: | :---: | :---: |
| Piper | ✅ | ✅ | — |
| Marvin | — | ✅ | — |
| Franka FR3 | — | ✅ | — |
| Hexfellow chassis/lift | — | ✅ | — |
| Wuji Hand1 / Wuji Hand2 | — | — | ✅ |
| Marvin + Wuji Hand1 / Wuji Hand2 | — | ✅ | ✅ |

## Requirements

Building this package requires Eigen, pybind11, Python development headers,
Pinocchio and URDFDOM. On Ubuntu 22.04, the following installs the C++
dependencies used by the [development image](../../projects/HoloBrain/dev/Dockerfile):

```bash
sudo apt-get install libeigen3-dev pybind11-dev python3-dev
python3 -m pip install --target /path/to/teleop-deps \
  "pin==2.7.0" "cmeel-eigen==3.4.1" "cmeel-urdfdom==3.1.1.1" \
  "cmeel-urdfdom-headers==1.1.2"
export CMAKE_PREFIX_PATH="/path/to/teleop-deps/cmeel.prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
```

Replace `/path/to/teleop-deps` with a persistent dependency directory, keep it
available at runtime, and build in the same shell. Environments with these
dependencies already installed only need their prefix on `CMAKE_PREFIX_PATH`.
Build and source the workspace using the [ROS 2 workspace instructions](../README.md).
Install the additional official dependencies for your devices:

| Device | Dependencies / setup |
| --- | --- |
| Piper | [Piper driver and SDK](../robo_orchard_piper_ros2/README.md) |
| Marvin | [Marvin control and kinematics SDKs](../robo_orchard_marvin_ros2/README.md) |
| Franka FR3 | [`libfranka` and `franka_ros2`](../robo_orchard_franka_ros2/README.md#dependencies) |
| Hexfellow chassis/lift | `hex_bridge`, `hex_device_msgs`, `hex_device_ros_wrapper` |
| Wuji Hand1 | [`wujihand_bringup`](https://github.com/wuji-technology/wujihandros2) |
| Wuji Hand2 | [Wuji SDK C 2026.10.26](../robo_orchard_wuji2_ros2/README.md) |
| Wuji glove | [Wuji Python SDK 2026.10.26 and glove setup](../robo_orchard_wuji_glove_ros2/README.md#vendor-sdk) |
| Pico VR | [XRoboToolkit-PC-Service-Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind), with [this binding patch](patches/py_bindings.cpp) replacing `bindings/py_bindings.cpp` |

## Quick start

Managed teleoperation requires one Control Manager, configured through
`control_manager_config_file`. Choose a launch from [launch/](launch/) and
use `--show-args` to view its parameters and defaults:

```bash
ros2 launch robo_orchard_teleop_ros2 <launch_file> --show-args
```

Wuji glove launches (including DAgger and Marvin combinations) require
`hand_serial_number` for each Hand2 when starting multiple Hand2 devices.
For example, use `hand_model:=wuji_hand_2`, `hand_side:=left,right` and
`hand_serial_number:=000123,000456`. A single Hand2 supports automatic discovery.

Pico launches accept the startup parameter `teleop_frequency_hz` for the
control loop (IK where applicable and command publication). Defaults remain
30 Hz, or 90 Hz for Franka; its optional Robotiq uses
`robotiq_teleop_frequency_hz` (30 Hz). Existing Robotiq/Hexfellow node
configurations, engagement nodes and Hexfellow launch commands must replace
`publish_hz` with `teleop_frequency_hz`; Franka/Marvin teleop configurations
must replace `control_frequency_hz` with the same name. This does not change
the Pico bridge's input rate or the Marvin driver's separate control frequency.
Franka + Robotiq command forwarding uses the arm's `teleop_frequency_hz`.

The built package's `ik_tests` target requires IKPy for independent model
comparisons (`python3 -m pip install "ikpy==4.5.0"`). Missing extensions or
test dependencies fail this target. Source-only checks under `tests/` do not
validate the compiled IK implementation in `test/test_ik.py`.

## DAgger Control

Use Control Manager to switch between autonomous inference and human takeover.
