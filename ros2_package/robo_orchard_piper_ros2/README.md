# robo_orchard_piper_ros2

This package includes the ros2 control node of the piper arm.

Adapt from the original [piper_ros](https://github.com/agilexrobotics/piper_ros)

## Joint names

`single_ctrl`, `aloha_ctrl`, and `aloha_raw_ctrl` accept the string-array
parameter `joint_names`. It must contain seven unique, non-empty names in
hardware order: arm joints 1 through 6, followed by the gripper. Defaults are
`[joint1, joint2, joint3, joint4, joint5, joint6, gripper]`. Every name is
configurable, including the gripper; no prefix or naming pattern is inferred.
This changes labels only, not hardware order, units, values, or ROS topics.
Namespaces and topic remappings do not modify joint names inside messages.

The Piper compatibility launches expose `left_joint_names` and
`right_joint_names` as lists. Their defaults are `left_joint1` through
`left_joint6`, `left_gripper`, and the corresponding `right_` names. Dagger
uses the same list for the master and follower on each side because the
master's feedback is also a command for that follower and both receive the
same command topic. These names identify the controlled left/right joint
layout; they do not create a separate naming namespace for the master arm.
Aloha applies its side's list to both sets of ROS feedback without changing
its SDK-level following loop.

The Pico launches pass the same lists to the drivers and to Pico's
`left_joint_names` / `right_joint_names` parameters. For Pico, the first six
entries must correspond to the IK solver's joint order and to hardware joints
1 through 6; the seventh is the gripper. Feedback must match this configured
order before seeding IK or labeling an IK result. Pico accepts arbitrary
names, not just `joint1`-style strings, but rejects reordered or foreign
feedback instead of guessing joint identity. A custom URDF must retain this
hardware/IK order. Configurations must describe the real mapping; name
validation cannot detect an incorrectly assigned physical joint.

For example, a project can supply:

```yaml
joint_names: [shoulder, upper, elbow, forearm, wrist, tool, opening]
```

Command messages must use the same names as the receiving controller.
Names and positions may be reordered together; the bridge maps them to
hardware order, including the gripper. Missing, duplicate, foreign, or
non-finite joint commands are rejected before any SDK control call. All six
arm joints are required. When `gripper_exist` is false, the gripper may be
omitted and no gripper command is sent. Reset uses the same naming contract.

### Migration

- Update command publishers, joint selectors, and Pico's expected order to
  match the driver's configured list. HoloBrain's generators match the
  dual-arm launches' default lists.
- The default gripper label is `gripper` (or `left_gripper` / `right_gripper`
  in dual-arm launches), not the historical command alias `joint7`. A
  different label is valid only when explicitly configured consistently.
- Standalone nodes retain native default names. Dual-arm launch overrides
  must provide complete lists, not prefixes. Quote names that YAML could
  otherwise interpret as booleans or numbers.
- The earlier, unmerged `joint_name_prefix` parameter and corresponding
  launch arguments are replaced by these ordered name lists.
- Host-side configuration generators must be updated together with the ROS
  runtime; topic remapping alone cannot make old command names compatible.
