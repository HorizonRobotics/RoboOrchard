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
    """Deploy action fields and topics shared by sync and async generators."""

    side: str
    server_output_key: str
    server_remaining_key: str
    autonomous_topic: str


COMMAND_CHANNELS = (
    CommandChannelSpec(
        side="left",
        server_output_key="left_arm_actions",
        server_remaining_key="left_arm_remaining_actions",
        autonomous_topic="/left_algo_cmd",
    ),
    CommandChannelSpec(
        side="right",
        server_output_key="right_arm_actions",
        server_remaining_key="right_arm_remaining_actions",
        autonomous_topic="/right_algo_cmd",
    ),
)
