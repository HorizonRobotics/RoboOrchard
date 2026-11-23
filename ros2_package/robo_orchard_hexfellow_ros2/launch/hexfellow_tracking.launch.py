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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Wrap the official chassis/lift drivers with one tracking adapter."""
    connection_arguments = {
        "enable_chassis_bridge": "true",
        "enable_lift_bridge": "true",
        "chassis_is_kcp": "true",
        "lift_is_kcp": "true",
        "chassis_read_only": "false",
        "lift_read_only": "false",
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "chassis_url", description="Chassis device host:port."
            ),
            DeclareLaunchArgument(
                "lift_url", description="Lift device host:port."
            ),
            DeclareLaunchArgument(
                "start_drivers",
                default_value="true",
                description="Disable only when official drivers already run.",
            ),
            *[
                DeclareLaunchArgument(name, default_value=default)
                for name, default in connection_arguments.items()
            ],
            DeclareLaunchArgument("body_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "tracker_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robo_orchard_hexfellow_ros2"),
                        "config",
                        "tracker.yaml",
                    ]
                ),
                description="ROS parameter YAML for hexfellow_tracker.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("hex_device_ros_wrapper"),
                            "launch",
                            "ros2",
                            "multi_bringup.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_drivers")),
                launch_arguments={
                    **{
                        name: LaunchConfiguration(name)
                        for name in (
                            "chassis_url",
                            "lift_url",
                            *connection_arguments,
                        )
                    },
                    "frame_id": LaunchConfiguration("body_frame"),
                    "simple_mode": "true",
                    "enable_ros_clock": "true",
                }.items(),
            ),
            Node(
                package="robo_orchard_hexfellow_ros2",
                executable="hexfellow_tracker",
                name="hexfellow_tracker",
                output="screen",
                parameters=[
                    LaunchConfiguration("tracker_config_file"),
                    {"body_frame": LaunchConfiguration("body_frame")},
                ],
            ),
        ]
    )
