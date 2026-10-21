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


from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Launch one driver with optional startup activation."""
    config = (
        Path(get_package_share_directory("robo_orchard_robotiq_ros2"))
        / "config"
        / "robotiq_2f85.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=str(config)),
            DeclareLaunchArgument("namespace", default_value="robotiq"),
            DeclareLaunchArgument("use_fake", default_value="false"),
            DeclareLaunchArgument(
                "auto_enable",
                default_value="false",
                description=(
                    "Activate once at startup; fingers may fully open/close."
                ),
            ),
            Node(
                package="robo_orchard_robotiq_ros2",
                executable="robotiq_driver_node",
                name="robotiq_driver",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "use_fake": ParameterValue(
                            LaunchConfiguration("use_fake"), value_type=bool
                        ),
                        "auto_enable": ParameterValue(
                            LaunchConfiguration("auto_enable"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
