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

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch Pico teleop with algo-topic outputs and dual-arm debug."""
    left_can_port_arg = DeclareLaunchArgument(
        "left_can_port",
        default_value="can_left",
        description="CAN port for the left arm.",
    )
    right_can_port_arg = DeclareLaunchArgument(
        "right_can_port",
        default_value="can_right",
        description="CAN port for the right arm.",
    )
    enable_mit_control_mode_arg = DeclareLaunchArgument(
        "enable_mit_control_mode",
        default_value="true",
        description="Whether enable mit control mode or not.",
    )
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value="",
        description="URDF path used by the Pico VR teleop solver.",
    )
    left_reset_joint_position_arg = DeclareLaunchArgument(
        "left_reset_joint_position",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description=(
            "Left arm reset target: 6 joint angles (rad) + gripper. Used "
            "by the single_ctrl reset_ctrl service."
        ),
    )
    right_reset_joint_position_arg = DeclareLaunchArgument(
        "right_reset_joint_position",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description=(
            "Right arm reset target: 6 joint angles (rad) + gripper. See "
            "left_reset_joint_position."
        ),
    )

    pico_bridge_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="pico_bridge",
        name="pico_bridge",
        namespace="/pico_bridge",
        output="screen",
        emulate_tty=True,
    )

    pico_teleop_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="piper_pico_vr_teleop",
        name="piper_pico_vr_teleop",
        namespace="/pico_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "urdf_path": LaunchConfiguration("urdf_path"),
                "ee_link_name": "link6",
                "base_link_name": "base_link",
                "left_reset_service": "/robot/left/reset_ctrl",
                "right_reset_service": "/robot/right/reset_ctrl",
            }
        ],
        remappings=[
            ("/robot/left/joint_cmd", "/left_algo_cmd"),
            ("/robot/right/joint_cmd", "/right_algo_cmd"),
            ("/robot/left/ee_pose", "/puppet/end_pose_left"),
            ("/robot/left/joint_state", "/puppet/joint_left"),
            ("/robot/right/ee_pose", "/puppet/end_pose_right"),
            ("/robot/right/joint_state", "/puppet/joint_right"),
        ],
    )

    left_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="single_ctrl",
        name="robot_left_single_controller",
        namespace="/robot/left",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("left_can_port"),
                "auto_enable_arm_ctrl": True,
                "gripper_exist": True,
                "enable_mit_ctrl": LaunchConfiguration(
                    "enable_mit_control_mode"
                ),
                "reset_joint_position": ParameterValue(
                    LaunchConfiguration("left_reset_joint_position"),
                    value_type=List[float],
                ),
            }
        ],
        remappings=[
            ("/robot/left/joint_cmd", "/left_algo_cmd"),
            ("/robot/left/status", "/puppet/status_left"),
            ("/robot/left/ee_pose", "/puppet/end_pose_left"),
            ("/robot/left/joint_state", "/puppet/joint_left"),
        ],
    )

    right_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="single_ctrl",
        name="robot_right_single_controller",
        namespace="/robot/right",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("right_can_port"),
                "auto_enable_arm_ctrl": True,
                "gripper_exist": True,
                "enable_mit_ctrl": LaunchConfiguration(
                    "enable_mit_control_mode"
                ),
                "reset_joint_position": ParameterValue(
                    LaunchConfiguration("right_reset_joint_position"),
                    value_type=List[float],
                ),
            }
        ],
        remappings=[
            ("/robot/right/joint_cmd", "/right_algo_cmd"),
            ("/robot/right/status", "/puppet/status_right"),
            ("/robot/right/ee_pose", "/puppet/end_pose_right"),
            ("/robot/right/joint_state", "/puppet/joint_right"),
        ],
    )

    return LaunchDescription(
        [
            left_can_port_arg,
            right_can_port_arg,
            enable_mit_control_mode_arg,
            urdf_path_arg,
            left_reset_joint_position_arg,
            right_reset_joint_position_arg,
            pico_bridge_node,
            pico_teleop_node,
            left_controller_node,
            right_controller_node,
        ]
    )
