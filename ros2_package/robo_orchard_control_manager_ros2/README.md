# RoboOrchard Control Manager

`AUTO` forwards autonomous commands, `TAKEOVER` forwards override commands,
and `STOP` and `RESETTING` block both. Forwarded messages are unchanged.

Each channel needs at least one input. Use separate channels and outputs
for inputs of different ROS types:

```yaml
channels:
  - name: motion_pose
    msg_type: nav_msgs/msg/Path
    autonomous_topic: /motion/auto_pose
    output_topic: /motion/routed_pose
  - name: motion_override
    msg_type: geometry_msgs/msg/Twist
    override_topic: /motion/override_velocity
    output_topic: /motion/routed_velocity
```

- Twist outputs receive zero at startup, on mode changes, and on explicit
  STOP requests. Other message types receive no neutral command.
- On AUTO → TAKEOVER, replay applies only to channels with autonomous input,
  excluding Twist, PoseStamped, and Path. With positive `replay_time_s`,
  every eligible channel must have a command at least that old, or none are
  replayed. Set `replay_time_s: 0` to disable replay.
- STOP is not an emergency stop. It does not disable hardware or cancel
  position targets already accepted by a driver. Drivers need their own
  command watchdogs.
