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
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Compose Hexfellow and dual Piper with the shared Pico runtime.

    Required arguments are forwarded explicitly. Each group inherits optional
    caller configuration without leaking child defaults to the other group.
    """
    launch_directory = PathJoinSubstitution(
        [FindPackageShare("robo_orchard_teleop_ros2"), "launch"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "chassis_url", description="Chassis device address."
            ),
            DeclareLaunchArgument(
                "lift_url", description="Lift device address."
            ),
            DeclareLaunchArgument(
                "control_manager_config_file",
                description="Caller configuration for all four channels.",
            ),
            DeclareLaunchArgument(
                "max_linear_x",
                default_value="0.2",
                description="Maximum chassis forward/backward speed in m/s.",
            ),
            DeclareLaunchArgument(
                "max_angular_z",
                default_value="0.2",
                description="Maximum chassis yaw rate in rad/s.",
            ),
            DeclareLaunchArgument(
                "position_rate",
                default_value="0.08",
                description="Lift target position change rate in m/s.",
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    launch_directory,
                                    "piper_pico_teleop_compat.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "control_manager_config_file": LaunchConfiguration(
                                "control_manager_config_file"
                            ),
                        }.items(),
                    )
                ],
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    launch_directory,
                                    "hexfellow_pico_teleop.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "chassis_url": LaunchConfiguration("chassis_url"),
                            "lift_url": LaunchConfiguration("lift_url"),
                            "max_linear_x": LaunchConfiguration(
                                "max_linear_x"
                            ),
                            "max_angular_z": LaunchConfiguration(
                                "max_angular_z"
                            ),
                            "position_rate": LaunchConfiguration(
                                "position_rate"
                            ),
                            "control_manager_config_file": LaunchConfiguration(
                                "control_manager_config_file"
                            ),
                            "start_pico_bridge": "false",
                            "start_control_manager": "false",
                        }.items(),
                    )
                ],
            ),
        ]
    )
