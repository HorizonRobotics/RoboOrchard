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
    left_master_can_port_arg = DeclareLaunchArgument(
        "left_master_can_port",
        default_value="can_left_mst",
        description="CAN port for the left master arm.",
    )
    left_slave_can_port_arg = DeclareLaunchArgument(
        "left_slave_can_port",
        default_value="can_left",
        description="CAN port for the left slave arm.",
    )
    right_master_can_port_arg = DeclareLaunchArgument(
        "right_master_can_port",
        default_value="can_right_mst",
        description="CAN port for the right master arm.",
    )
    right_slave_can_port_arg = DeclareLaunchArgument(
        "right_slave_can_port",
        default_value="can_right",
        description="CAN port for the right slave arm.",
    )
    enable_mit_ctrl_arg = DeclareLaunchArgument(
        "enable_mit_ctrl",
        default_value="true",
        description="Whether enable mit control mode or not.",
    )
    enable_master_mit_ctrl_arg = DeclareLaunchArgument(
        "enable_master_mit_ctrl",
        default_value="false",
        description="Whether enable mit control mode or not.",
    )

    # --- Node Definitions ---
    left_aloha_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="aloha_ctrl",
        name="robot_left_aloha_controller",
        namespace="/robot/left",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "master_can_port": LaunchConfiguration("left_master_can_port"),
                "slave_can_port": LaunchConfiguration("left_slave_can_port"),
                "gripper_exist": True,
                "sync_frequency": 200.0,
                "enable_mit_ctrl": LaunchConfiguration("enable_mit_ctrl"),
                "enable_master_ctrl": True,
                "enable_master_mit_ctrl": LaunchConfiguration(
                    "enable_master_mit_ctrl"
                ),
            }
        ],
        remappings=[
            ("/robot/left/ee_pose", "/puppet/end_pose_left"),
            ("/robot/left/joint_state", "/puppet/joint_left"),
            ("/robot/left/status", "/puppet/status_left"),
            ("/robot/left/master/ee_pose", "/master/end_pose_left"),
            ("/robot/left/master/joint_state", "/master/joint_left"),
            ("/robot/left/master/status", "/master/status_left"),
        ],
    )
    right_aloha_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="aloha_ctrl",
        name="robot_right_aloha_controller",
        namespace="/robot/right",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "master_can_port": LaunchConfiguration(
                    "right_master_can_port"
                ),
                "slave_can_port": LaunchConfiguration("right_slave_can_port"),
                "gripper_exist": True,
                "sync_frequency": 200.0,
                "enable_mit_ctrl": LaunchConfiguration("enable_mit_ctrl"),
                "enable_master_ctrl": True,
                "enable_master_mit_ctrl": LaunchConfiguration(
                    "enable_master_mit_ctrl"
                ),
            }
        ],
        remappings=[
            ("/robot/right/ee_pose", "/puppet/end_pose_right"),
            ("/robot/right/joint_state", "/puppet/joint_right"),
            ("/robot/right/status", "/puppet/status_right"),
            ("/robot/right/master/ee_pose", "/master/end_pose_right"),
            ("/robot/right/master/joint_state", "/master/joint_right"),
            ("/robot/right/master/status", "/master/status_right"),
        ],
    )

    # --- Create the Launch Description ---
    # The launch description is a container for all the actions to be executed.
    return LaunchDescription(
        [
            # Add the declared arguments
            left_master_can_port_arg,
            left_slave_can_port_arg,
            right_master_can_port_arg,
            right_slave_can_port_arg,
            enable_mit_ctrl_arg,
            enable_master_mit_ctrl_arg,
            # Add the nodes to be launched
            left_aloha_controller_node,
            right_aloha_controller_node,
        ]
    )
