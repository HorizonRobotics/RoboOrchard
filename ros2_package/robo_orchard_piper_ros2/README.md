# robo_orchard_piper_ros2

This package includes the ros2 control node of the piper arm.

Adapt from the original [piper_ros](https://github.com/agilexrobotics/piper_ros)

## Single-arm feedback and TF

`single_ctrl` publishes the SDK end-effector feedback on `ee_pose`
(`geometry_msgs/PoseStamped`) and, by default, broadcasts the same pose as a
dynamic transform on `/tf`. Both use the same timestamp from the node clock
in the existing 200 Hz feedback callback.

| Node parameter | Default | Meaning |
| --- | --- | --- |
| `base_frame_id` | `base_link` | Reference frame of the SDK pose and TF parent. |
| `ee_frame_id` | `end_effector` | SDK end-effector frame and TF child. |
| `publish_ee_tf` | `true` | Enable dynamic end-effector TF broadcasting. |

Frame IDs must be non-empty and different. `ee_pose.header.frame_id` is always
set to `base_frame_id`, including when `publish_ee_tf` is false. Disabling TF
does not disable any feedback topics or change arm control behavior.

The frame names label the SDK coordinate systems; they do not transform the
pose into another base frame or apply a tool/TCP offset. Use additional
transforms if the robot description or tool frames differ from the SDK frames.
ROS namespaces and topic remappings do not prefix these frame IDs. Disable
this broadcaster if another node already owns the same end-effector TF frame.

## Mainline dual-arm launches

The teleop package's `piper_control_compat.launch.py`,
`piper_dagger_compat.launch.py`, `piper_aloha_compat.launch.py`,
`piper_pico_dagger_compat.launch.py`, and
`piper_pico_teleop_compat.launch.py` expose the driver frame parameters with
these role-specific defaults:

| Role | Base frame | End-effector frame | Publish TF |
| --- | --- | --- | --- |
| Left follower | `left_base_link` | `left_end_effector` | `true` |
| Right follower | `right_base_link` | `right_end_effector` | `true` |
| Left master | `left_master_base_link` | `left_master_end_effector` | `false` |
| Right master | `right_master_base_link` | `right_master_end_effector` | `false` |

Override follower frames with `left_base_frame_id`, `left_ee_frame_id`,
`right_base_frame_id`, and `right_ee_frame_id`. `publish_ee_tf` controls TF for
both followers. The four-arm Aloha/Dagger launches also expose
`left_master_base_frame_id`, `left_master_ee_frame_id`,
`right_master_base_frame_id`, `right_master_ee_frame_id`, and
`publish_master_ee_tf`. The Aloha wrapper forwards all these arguments to
Dagger. Master pose topics still carry their own base frame when TF is off.

For hand-eye calibration, match `base_frame_name` to the driver's
`base_frame_id` and `end_effector_frame_name` to its `ee_frame_id`. With
calibration TF publishing enabled, eye-in-hand forms the chain
`base -> end_effector -> camera`; eye-to-hand connects the camera and the
moving end effector to the same base. The driver publishes only the dynamic
`base -> end_effector` edge, not calibration results or the transform between
the two arm bases.

The separate `aloha_ctrl` and `aloha_raw_ctrl` implementations, including
`piper_aloha_raw_compat.launch.py`, are unchanged and do not gain this TF
broadcaster.
