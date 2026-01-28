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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the complete human takeover system, including the muxer and the hardware controller."""  # noqa: E501
    # --- Declare Launch Arguments ---
    left_can_port_arg = DeclareLaunchArgument(
        "left_can_port",
        default_value="can_left",
        description="CAN port for the left slave arm.",
    )
    right_can_port_arg = DeclareLaunchArgument(
        "right_can_port",
        default_value="can_right",
        description="CAN port for the right slave arm.",
    )

    # --- Node Definitions ---
    left_node = Node(
        package="robo_orchard_piper_ros2",
        executable="aloha_raw_ctrl",
        name="robot_left_aloha_raw_controller",
        namespace="/robot/left",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("left_can_port_arg"),
            }
        ],
        remappings=[
            ("/robot/left/ee_pose", "/puppet/end_pose_left"),
            ("/robot/left/joint_state", "/puppet/joint_left"),
            ("/robot/left/status", "/puppet/status_left"),
            ("/robot/left/master/joint_state", "/master/joint_left"),
        ],
    )
    right_node = Node(
        package="robo_orchard_piper_ros2",
        executable="aloha_raw_ctrl",
        name="robot_right_aloha_raw_controller",
        namespace="/robot/right",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("right_can_port_arg"),
            }
        ],
        remappings=[
            ("/robot/right/ee_pose", "/puppet/end_pose_right"),
            ("/robot/right/joint_state", "/puppet/joint_right"),
            ("/robot/right/status", "/puppet/status_right"),
            ("/robot/right/master/joint_state", "/master/joint_right"),
        ],
    )

    # --- Create the Launch Description ---
    # The launch description is a container for all the actions to be executed.
    return LaunchDescription(
        [
            # Add the declared arguments
            left_can_port_arg,
            right_can_port_arg,
            # Add the nodes to be launched
            left_node,
            right_node,
        ]
    )
