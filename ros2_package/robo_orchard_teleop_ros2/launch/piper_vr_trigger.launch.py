# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # --- Node Definitions ---
    trigger_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="pico_vr_trigger",
        name="pico_vr_trigger",
        namespace="/pico_vr_trigger",
        output="screen",
        emulate_tty=True,  # Ensures logs are displayed properly
        parameters=[
            {
                "key": "trigger",
                "action_services": [
                    "/robot/left/takeover_muxer/trigger_takeover",
                    "/robot/right/takeover_muxer/trigger_takeover",
                ],
            }
        ],
    )
    release_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="pico_vr_trigger",
        name="pico_vr_release",
        namespace="/pico_vr_release",
        output="screen",
        emulate_tty=True,  # Ensures logs are displayed properly
        parameters=[
            {
                "key": "gripper",
                "action_services": [
                    "/robot/left/takeover_muxer/release_control",
                    "/robot/right/takeover_muxer/release_control",
                ],
            }
        ],
    )

    # --- Create the Launch Description ---
    # The launch description is a container for all the actions to be executed.
    return LaunchDescription(
        [
            trigger_node,
            release_node,
        ]
    )
