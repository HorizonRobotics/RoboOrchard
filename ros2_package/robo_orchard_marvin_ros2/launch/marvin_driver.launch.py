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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robo_orchard_marvin_ros2")
    default_config = os.path.join(package_share, "config", "marvin_m6s.yaml")

    config_file = LaunchConfiguration("config_file")
    auto_enable_side = LaunchConfiguration("auto_enable_side")
    auto_enable_mode = LaunchConfiguration("auto_enable_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("auto_enable_side", default_value="none"),
            DeclareLaunchArgument("auto_enable_mode", default_value=""),
            Node(
                package="robo_orchard_marvin_ros2",
                executable="marvin_driver_node",
                name="marvin_driver_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "auto_enable_side": auto_enable_side,
                        "auto_enable_mode": auto_enable_mode,
                    },
                ],
            ),
        ]
    )
