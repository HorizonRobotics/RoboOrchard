# RoboOrchard Marvin ROS 2 Interfaces

This package defines the public ROS 2 service interfaces used by
`robo_orchard_marvin_ros2`.

`SetControlMode.srv` exposes the driver-level modes `IDLE`, `POSITION`,
`JOINT_IMPEDANCE`, and `JOINT_DRAG` without leaking the Marvin SDK's separate
arm-state, impedance-type, and drag-space fields to command producers.
