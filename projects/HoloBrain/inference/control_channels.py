# Project RoboOrchard
#
# Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

from dataclasses import dataclass

JOINT_COMMAND_MSG_TYPE = "sensor_msgs/msg/JointState"
PIPER_JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
)


@dataclass(frozen=True)
class CommandChannelSpec:
    """Project command wiring shared by Deploy and Control Manager.

    ``name`` is the Manager's channel label. ``side`` supplies the prefix
    for joint names and the default observation and teleop topic patterns.
    Model-server field names are explicit, independent of the channel label.
    """

    side: str
    name: str
    server_output_key: str
    server_remaining_key: str
    autonomous_topic: str
    output_topic: str
    msg_type: str = JOINT_COMMAND_MSG_TYPE


COMMAND_CHANNELS = (
    CommandChannelSpec(
        side="left",
        name="left_arm",
        server_output_key="left_arm_actions",
        server_remaining_key="left_arm_remaining_actions",
        autonomous_topic="/left_algo_cmd",
        output_topic="/robot/left/joint_cmd",
    ),
    CommandChannelSpec(
        side="right",
        name="right_arm",
        server_output_key="right_arm_actions",
        server_remaining_key="right_arm_remaining_actions",
        autonomous_topic="/right_algo_cmd",
        output_topic="/robot/right/joint_cmd",
    ),
)


def override_topic(channel: CommandChannelSpec, teleop_source: str) -> str:
    """Return the operator command topic for a project runtime."""
    side = channel.side
    if teleop_source == "aloha":
        return f"/master/joint_{side}"
    if teleop_source == "pico":
        return f"/pico_teleop/joint_{side}"
    raise ValueError(
        f"Unsupported teleop source {teleop_source!r}; expected aloha or pico"
    )
