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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "left_command_topic", default_value="/left_algo_cmd"
            ),
            DeclareLaunchArgument(
                "right_command_topic", default_value="/right_algo_cmd"
            ),
            DeclareLaunchArgument(
                "left_state_topic", default_value="/puppet/joint_left"
            ),
            DeclareLaunchArgument(
                "right_state_topic", default_value="/puppet/joint_right"
            ),
            DeclareLaunchArgument("publish_left", default_value="true"),
            DeclareLaunchArgument("publish_right", default_value="true"),
            DeclareLaunchArgument("use_current_state", default_value="true"),
            DeclareLaunchArgument("mirror_right", default_value="false"),
            DeclareLaunchArgument("rate_hz", default_value="100.0"),
            DeclareLaunchArgument("start_delay_s", default_value="1.0"),
            DeclareLaunchArgument("start_trigger_file", default_value=""),
            DeclareLaunchArgument("ready_file", default_value=""),
            DeclareLaunchArgument(
                "min_command_subscribers", default_value="0"
            ),
            DeclareLaunchArgument(
                "command_subscriber_wait_timeout_s", default_value="2.0"
            ),
            DeclareLaunchArgument("duration_s", default_value="10.0"),
            DeclareLaunchArgument("amplitude_scale", default_value="1.0"),
            DeclareLaunchArgument("frequency_scale", default_value="1.0"),
            Node(
                package="robo_orchard_teleop_ros2",
                executable="scripted_joint_master",
                name="scripted_joint_master",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "left_command_topic": LaunchConfiguration(
                            "left_command_topic"
                        ),
                        "right_command_topic": LaunchConfiguration(
                            "right_command_topic"
                        ),
                        "left_state_topic": LaunchConfiguration(
                            "left_state_topic"
                        ),
                        "right_state_topic": LaunchConfiguration(
                            "right_state_topic"
                        ),
                        "publish_left": ParameterValue(
                            LaunchConfiguration("publish_left"), value_type=bool
                        ),
                        "publish_right": ParameterValue(
                            LaunchConfiguration("publish_right"), value_type=bool
                        ),
                        "use_current_state": ParameterValue(
                            LaunchConfiguration("use_current_state"),
                            value_type=bool,
                        ),
                        "mirror_right": ParameterValue(
                            LaunchConfiguration("mirror_right"), value_type=bool
                        ),
                        "rate_hz": ParameterValue(
                            LaunchConfiguration("rate_hz"), value_type=float
                        ),
                        "start_delay_s": ParameterValue(
                            LaunchConfiguration("start_delay_s"), value_type=float
                        ),
                        "start_trigger_file": LaunchConfiguration(
                            "start_trigger_file"
                        ),
                        "ready_file": LaunchConfiguration("ready_file"),
                        "min_command_subscribers": ParameterValue(
                            LaunchConfiguration("min_command_subscribers"),
                            value_type=int,
                        ),
                        "command_subscriber_wait_timeout_s": ParameterValue(
                            LaunchConfiguration(
                                "command_subscriber_wait_timeout_s"
                            ),
                            value_type=float,
                        ),
                        "duration_s": ParameterValue(
                            LaunchConfiguration("duration_s"), value_type=float
                        ),
                        "amplitude_scale": ParameterValue(
                            LaunchConfiguration("amplitude_scale"), value_type=float
                        ),
                        "frequency_scale": ParameterValue(
                            LaunchConfiguration("frequency_scale"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
