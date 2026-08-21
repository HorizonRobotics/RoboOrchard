# RoboOrchard Teleop ROS 2

Teleoperation for RoboOrchard robots. The package turns operator input into
robot commands, and provides the take-over (DAgger) plumbing used during data
collection.

## Teleoperation modes

- **Aloha leader-follower** — a leader arm drives the follower over CAN.
  Piper only.
- **Pico VR** — a Pico headset streams controller poses and the arm follows
  through IK. Piper and Marvin. The bridge reads the raw device message, which
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
| Wuji hand          |   —   |    —    |     ✅      |
| Marvin + Wuji hand |   —   |    ✅    |     ✅      |

## Quick start

Each command starts a complete stack: robot driver, input bridge and teleop
node. Append `--show-args` to any launch file to list its parameters.

Aloha, Piper:

```bash
ros2 launch robo_orchard_teleop_ros2 piper_aloha_compat.launch.py
```

Pico VR, Piper:

```bash
ros2 launch robo_orchard_teleop_ros2 piper_pico_teleop_compat.launch.py
```

Pico VR, Marvin:

```bash
ros2 launch robo_orchard_teleop_ros2 marvin_pico_teleop.launch.py
```

Wuji glove and hand:

```bash
ros2 launch robo_orchard_teleop_ros2 wuji_glove_teleop_compat.launch.py \
  hand_side:=left,right \
  hand_name:=hand_left,hand_right \
  hand_serial_number:=<left-hand-serial>,<right-hand-serial> \
  glove_serial_number:=<left-glove-serial>,<right-glove-serial>
```

Every glove argument takes one comma-separated entry per instance. Serial
numbers are required once both sides are connected, because handedness alone
no longer identifies a device.

Marvin with the Wuji hand:

```bash
ros2 launch robo_orchard_teleop_ros2 marvin_wuji_keyboard_teleop.launch.py \
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
  ros2 launch robo_orchard_teleop_ros2 marvin_pico_teleop.launch.py
```

`taskset` keeps the stack on the performance cores. The control loop is
latency-sensitive and drifts when the scheduler moves it onto efficiency
cores. The core numbers depend on the machine.

The thread-count variables keep NumPy's BLAS pool from spinning about four
cores, which drags the 30 Hz control loop down to 20 Hz.

## Engaging teleop

Teleop drives the robot only while the operator engages it. Two input sources
are available, selected with `operator_input_source`:

- `pico` (default) — hold the grip button on the Pico controller.
- `keyboard` — hold `T` for one second to engage, release to stop; hold `R`
  for one second to request a reset.

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

The `*_dagger_compat.launch.py` files add a mux that lets a policy drive the
robot and hands control to the operator mid-episode. Piper and the glove have
one; Marvin does not yet.

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
