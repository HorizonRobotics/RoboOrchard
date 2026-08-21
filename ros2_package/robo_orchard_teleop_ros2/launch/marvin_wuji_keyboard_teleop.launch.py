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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from robo_orchard_teleop_ros2.wuji_teleop_launch import (
    build_wuji_pair_actions,
    declare_wuji_teleop_arguments,
    resolve_wuji_launch_instances,
)


def _launch_source(package: str, launch_file: str):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare(package), "launch", launch_file]
        )
    )


def _launch_instances(context):
    actions = []
    for instance in resolve_wuji_launch_instances(context, dagger=False):
        hand_command_topic = f"/{instance.hand_name}/joint_commands"
        actions.extend(build_wuji_pair_actions(instance, hand_command_topic))
    return actions


def generate_launch_description():
    marvin = IncludeLaunchDescription(
        _launch_source(
            "robo_orchard_teleop_ros2", "marvin_pico_teleop.launch.py"
        ),
        launch_arguments={
            "urdf_path": LaunchConfiguration("urdf_path"),
            "driver_config_file": LaunchConfiguration("driver_config_file"),
            "auto_enable_side": LaunchConfiguration("auto_enable_side"),
            "auto_enable_mode": LaunchConfiguration("auto_enable_mode"),
            "keyboard_control_side": LaunchConfiguration(
                "keyboard_control_side"
            ),
            # Hardcoded rather than exposed: the gloves occupy the operator's
            # hands, so the Pico grip button cannot be reached and keyboard
            # is the only activation source that works in this setup.
            "operator_input_source": "keyboard",
        }.items(),
    )
    keyboard = IncludeLaunchDescription(
        _launch_source(
            "robo_orchard_teleop_ros2", "teleop_keyboard.launch.py"
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("keyboard_config_file"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robo_orchard_marvin_ros2"),
                        "urdf",
                        "marvin_m6s_ccs_696_v4.urdf",
                    ]
                ),
                description=(
                    "Dual-arm Marvin URDF used for IK. Defaults to the model "
                    "shipped with the driver. An override must contain "
                    "robot_stand, TCP_Link_L, and TCP_Link_R."
                ),
            ),
            DeclareLaunchArgument(
                "driver_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robo_orchard_marvin_ros2"),
                        "config",
                        "marvin_m6s_with_wujihand.yaml",
                    ]
                ),
                description=(
                    "Marvin driver profile. Defaults to the one carrying the "
                    "Wuji Hand tool dynamics."
                ),
            ),
            DeclareLaunchArgument("auto_enable_side", default_value="both"),
            DeclareLaunchArgument(
                "auto_enable_mode", default_value="joint_impedance"
            ),
            DeclareLaunchArgument(
                "keyboard_control_side",
                default_value="both",
                description="Keyboard-controlled arm: left, right, or both.",
            ),
            DeclareLaunchArgument(
                "keyboard_config_file",
                description=(
                    "Keyboard device and key mapping configuration. Required: "
                    "create it from config/teleop_keyboard.example.yaml and "
                    "fill in the device IDs of the intended interface."
                ),
            ),
            *declare_wuji_teleop_arguments(dagger=False),
            marvin,
            keyboard,
            OpaqueFunction(function=_launch_instances),
        ]
    )
