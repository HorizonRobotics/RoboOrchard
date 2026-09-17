# robo_orchard_piper_ros2

This package includes the ros2 control node of the piper arm.

Adapt from the original [piper_ros](https://github.com/agilexrobotics/piper_ros)

## Single-arm feedback and TF

`single_ctrl` publishes the SDK end-effector feedback on `ee_pose`
(`geometry_msgs/PoseStamped`) and, by default, broadcasts the same pose as a
dynamic transform on `/tf`.

| Node parameter | Default | Meaning |
| --- | --- | --- |
| `base_frame_id` | `base_link` | Reference frame of the SDK pose and TF parent. |
| `ee_frame_id` | `end_effector` | SDK end-effector frame and TF child. |
| `publish_ee_tf` | `true` | Enable dynamic end-effector TF broadcasting. |

The frame names label the SDK coordinate systems; they do not transform the
pose into another base frame or apply a tool/TCP offset.

Set `publish_ee_tf` to `false` if another node already publishes the same
end-effector TF frame.
