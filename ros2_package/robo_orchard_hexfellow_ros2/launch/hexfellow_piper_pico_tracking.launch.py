# Project RoboOrchard
#
# Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
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
    """Compose existing Pico/Piper runtime with the optional tracking wrapper.

    Scoped groups inherit optional arguments such as CAN ports and tracker
    configuration without leaking child defaults between the two runtimes.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "chassis_url", description="Chassis host:port."
            ),
            DeclareLaunchArgument("lift_url", description="Lift host:port."),
            DeclareLaunchArgument(
                "control_manager_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robo_orchard_hexfellow_ros2"),
                        "config",
                        "control_manager.example.yaml",
                    ]
                ),
                description="Five-channel routing configuration.",
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare(
                                        "robo_orchard_teleop_ros2"
                                    ),
                                    "launch",
                                    "hexfellow_piper_pico_teleop_compat.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "chassis_url": LaunchConfiguration("chassis_url"),
                            "lift_url": LaunchConfiguration("lift_url"),
                            "control_manager_config_file": LaunchConfiguration(
                                "control_manager_config_file"
                            ),
                            "start_drivers": "false",
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
                                    FindPackageShare(
                                        "robo_orchard_hexfellow_ros2"
                                    ),
                                    "launch",
                                    "hexfellow_tracking.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "chassis_url": LaunchConfiguration("chassis_url"),
                            "lift_url": LaunchConfiguration("lift_url"),
                        }.items(),
                    )
                ],
            ),
        ]
    )
