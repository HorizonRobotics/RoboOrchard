# RoboOrchard Franka ROS 2

This package provides the RoboOrchard ros2_control integration for a Franka
FR3 arm. It starts the official Franka hardware interface and state
broadcasters together with a RoboOrchard Ruckig-based joint position
controller. The native Franka gripper can optionally run through the official
`franka_gripper` driver in a separate process.

The launch selects the robot description and controller configuration from the
`arm_id` argument. Currently, only `arm_id:=fr3` is supported.

## Dependencies

Use the following dependencies:

- ROS 2 Humble
- [`libfranka` 0.18.0](https://github.com/frankarobotics/libfranka/tree/0.18.0)
- [`franka_ros2` v2.1.0](https://github.com/frankarobotics/franka_ros2/tree/v2.1.0)

Install and build these dependencies first. Source ROS 2 and the Franka
workspace before building this package:

```bash
source /opt/ros/humble/setup.bash
source <path-to-franka_ros2-workspace>/install/setup.bash
```

After building RoboOrchard, source its workspace before launching:

```bash
source <path-to-roboorchard-workspace>/install/setup.bash
```

The launch defaults to `load_gripper:=false` for arm-only operation:

```bash
ros2 launch robo_orchard_franka_ros2 joint_position_control.launch.py \
  robot_ip:=172.16.0.2 \
  arm_id:=fr3 \
  use_rviz:=false
```

Reset the arm to the home joint position configured by
`reset_joint_position` in `config/fr3_controllers.yaml`:

```bash
ros2 service call /robot/franka/reset_ctrl std_srvs/srv/Trigger "{}"
```

Reset discards queued arm commands. Send fresh targets after the reset service
returns; commands sent before or during reset are not resumed.

## Optional native Franka gripper

Build the official `franka_gripper` package from the same `franka_ros2`
workspace. With a native gripper attached, enable its driver and hand model:

```bash
ros2 launch robo_orchard_franka_ros2 joint_position_control.launch.py \
  robot_ip:=172.16.0.2 load_gripper:=true
```

This starts the official `franka_gripper_node` directly, using its shipped
`config/franka_gripper_node.yaml`, robot IP, and finger joint names. The
node is named `fr3_gripper` for FR3. It runs separately from the arm's
`ros2_control_node`; no gripper calls run inside the arm's 1000 Hz update
loop. The arm command and `/franka/joint_states` topics still contain only
seven arm joints; the joint state publisher combines arm and finger
feedback on `/joint_states` for TF.
Process separation does not guarantee real-time performance. Check real-time
performance on the target machine before simultaneous arm and gripper motion.

With the default FR3 launch, the unmodified official interfaces are:

| Interface | Type | Purpose |
| --- | --- | --- |
| `/fr3_gripper/homing` | `franka_msgs/action/Homing` | Calibrate the gripper |
| `/fr3_gripper/move` | `franka_msgs/action/Move` | Set total opening width (m) and speed (m/s) |
| `/fr3_gripper/grasp` | `franka_msgs/action/Grasp` | Grasp with width, speed, force (N), and tolerances |
| `/fr3_gripper/gripper_action` | `control_msgs/action/GripperCommand` | Standard gripper action; goal position is half the total opening width |
| `/fr3_gripper/stop` | `std_srvs/srv/Trigger` | Stop gripper motion |
| `/fr3_gripper/joint_states` | `sensor_msgs/msg/JointState` | Two finger positions, each half the opening width |

The gripper is separate from arm control and is not affected by arm reset.
Control it directly through the official interfaces. Keep
`load_gripper:=false` when using Robotiq instead of the native gripper.

For teleoperation and DAgger setup, see the
[Franka teleoperation guide](../robo_orchard_teleop_ros2/README.md#franka-managed-teleoperation-and-dagger).
