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
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Launch one Hand 2, disabled by default, in its own namespace."""
    config = (
        Path(get_package_share_directory("robo_orchard_wuji2_ros2"))
        / "config"
        / "wuji_hand2.yaml"
    )
    selectors = {
        "endpoint": "",
        "serial_number": "",
        "hand_side": "right",
    }
    options = {
        "connect_timeout": ("10.0", float),
        "auto_enable": ("false", bool),
        "publish_joint_diagnostics": ("true", bool),
        "publish_imu": ("true", bool),
        "publish_fingertip_tactile": ("false", bool),
        "reset_duration_s": ("2.0", float),
        "reset_timeout_s": ("5.0", float),
        "reset_feedback_timeout_s": ("0.2", float),
        "reset_position_tolerance": ("0.2", float),
        "reset_joint_position": (str([0.0] * 20), List[float]),
    }
    arguments = [
        DeclareLaunchArgument("hand_name", default_value="hand_right"),
    ]
    arguments.extend(
        DeclareLaunchArgument(name, default_value=value)
        for name, value in selectors.items()
    )
    arguments.extend(
        DeclareLaunchArgument(name, default_value=value)
        for name, (value, _) in options.items()
    )
    overrides = {
        name: ParameterValue(LaunchConfiguration(name), value_type=str)
        for name in selectors
    }
    overrides.update(
        {
            name: ParameterValue(
                LaunchConfiguration(name), value_type=value_type
            )
            for name, (_, value_type) in options.items()
        }
    )
    return LaunchDescription(
        [
            *arguments,
            Node(
                package="robo_orchard_wuji2_ros2",
                executable="wuji_hand2_node",
                name="wuji_hand2_driver",
                namespace=LaunchConfiguration("hand_name"),
                output="screen",
                parameters=[str(config), overrides],
            ),
        ]
    )
