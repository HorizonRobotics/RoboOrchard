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

from __future__ import annotations
from typing import Any

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from robo_orchard_teleop_ros2.wuji_teleop_instances import (
    WujiTeleopInstance,
    resolve_wuji_teleop_instances,
)


def declare_wuji_teleop_arguments(
    *, dagger: bool
) -> list[DeclareLaunchArgument]:
    """Declare arguments shared by Wuji direct and DAgger launches."""
    arguments = [
        DeclareLaunchArgument(
            "hand_side",
            default_value="right",
            description="Comma-separated glove and hand sides: left or right.",
        ),
        DeclareLaunchArgument(
            "hand_name",
            default_value="",
            description=(
                "Optional comma-separated wujihandros2 namespaces. Empty "
                "values derive from hand_side; repeated sides require names."
            ),
        ),
        DeclareLaunchArgument(
            "hand_serial_number",
            default_value="",
            description=(
                "Optional comma-separated Wuji Hand serial numbers. Repeated "
                "sides require serial numbers."
            ),
        ),
        DeclareLaunchArgument(
            "glove_namespace",
            default_value="",
            description=(
                "Optional comma-separated Wuji Glove namespaces. Empty values "
                "derive from hand_name."
            ),
        ),
        DeclareLaunchArgument(
            "glove_serial_number",
            default_value="",
            description=(
                "Optional comma-separated Wuji Glove serial numbers. Repeated "
                "sides require serial numbers."
            ),
        ),
        DeclareLaunchArgument(
            "glove_device_name",
            default_value="",
            description=(
                "Optional comma-separated Wuji SDK aliases. Empty values "
                "derive from hand_name."
            ),
        ),
        DeclareLaunchArgument(
            "glove_frame_prefix",
            default_value="",
            description=(
                "Optional comma-separated frame prefixes. A left/right pair "
                "shares its single configured prefix; other empty values "
                "derive from glove_device_name."
            ),
        ),
        DeclareLaunchArgument(
            "glove_sdk_user",
            default_value="default",
            description=(
                "One Wuji SDK user broadcast to every Glove, or one "
                "comma-separated user per instance."
            ),
        ),
        DeclareLaunchArgument(
            "glove_hand_model_path",
            default_value="",
            description=(
                "Directory containing per-user calibrated hand models."
            ),
        ),
        DeclareLaunchArgument(
            "glove_stream_profile",
            default_value="teleop_minimal",
            description=(
                "Wuji Glove stream profile. Use configured for full "
                "observation collection."
            ),
        ),
    ]
    if dagger:
        arguments.extend(
            [
                DeclareLaunchArgument(
                    "algo_topic",
                    default_value="",
                    description=(
                        "Optional comma-separated autonomous command topics. "
                        "Empty values derive from hand_name."
                    ),
                ),
                DeclareLaunchArgument(
                    "replay_time_s",
                    default_value="0.0",
                    description="Mux algorithm-command replay time.",
                ),
            ]
        )
    arguments.extend(
        [
            DeclareLaunchArgument(
                "hand_publish_rate",
                default_value="1000.0",
                description="Wuji Hand joint-state publish rate.",
            ),
            DeclareLaunchArgument(
                "hand_filter_cutoff_freq",
                default_value="10.0",
                description="Wuji Hand command low-pass cutoff frequency.",
            ),
            DeclareLaunchArgument(
                "hand_diagnostics_rate",
                default_value="10.0",
                description="Wuji Hand diagnostics publish rate.",
            ),
        ]
    )
    return arguments


def resolve_wuji_launch_instances(
    context: Any, *, dagger: bool
) -> list[WujiTeleopInstance]:
    """Resolve all per-instance launch arguments from a launch context."""
    argument_names = (
        "hand_side",
        "hand_name",
        "hand_serial_number",
        "glove_namespace",
        "glove_serial_number",
        "glove_device_name",
        "glove_frame_prefix",
        "glove_sdk_user",
    )
    values = {
        name: LaunchConfiguration(name).perform(context)
        for name in argument_names
    }
    if dagger:
        values["algo_topic"] = LaunchConfiguration("algo_topic").perform(
            context
        )
    return resolve_wuji_teleop_instances(**values)


def build_wuji_pair_actions(
    instance: WujiTeleopInstance, command_topic: str
) -> tuple[IncludeLaunchDescription, IncludeLaunchDescription]:
    """Build the hand and glove launch includes for one instance."""
    hand = IncludeLaunchDescription(
        _launch_source("wujihand_bringup", "wujihand.launch.py"),
        launch_arguments={
            "hand_name": instance.hand_name,
            "serial_number": instance.hand_serial_number,
            "hand_side": instance.hand_side,
            "publish_rate": LaunchConfiguration("hand_publish_rate"),
            "filter_cutoff_freq": LaunchConfiguration(
                "hand_filter_cutoff_freq"
            ),
            "diagnostics_rate": LaunchConfiguration("hand_diagnostics_rate"),
            "rviz": "false",
            "foxglove": "false",
        }.items(),
    )
    glove = IncludeLaunchDescription(
        _launch_source("robo_orchard_wuji_glove_ros2", "wuji_glove.launch.py"),
        launch_arguments={
            "glove_namespace": instance.glove_namespace,
            "glove_serial_number": instance.glove_serial_number,
            "hand_side": instance.hand_side,
            "device_name": instance.glove_device_name,
            "frame_prefix": instance.glove_frame_prefix,
            "sdk_user": instance.glove_sdk_user,
            "hand_model_path": LaunchConfiguration("glove_hand_model_path"),
            "stream_profile": LaunchConfiguration("glove_stream_profile"),
            "hand_model": "wuji_hand",
            "command_topic": command_topic,
        }.items(),
    )
    return hand, glove


def _launch_source(
    package: str, filename: str
) -> PythonLaunchDescriptionSource:
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), "launch", filename])
    )
