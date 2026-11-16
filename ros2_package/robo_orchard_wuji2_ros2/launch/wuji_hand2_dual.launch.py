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
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _validate_selectors(context: LaunchContext) -> None:
    for side in ("left", "right"):
        if not any(
            LaunchConfiguration(f"{side}_{selector}").perform(context).strip()
            for selector in ("endpoint", "serial_number")
        ):
            raise RuntimeError(
                f"Dual Hand 2 launch requires {side}_endpoint or "
                f"{side}_serial_number; automatic discovery is only "
                "supported by the single-hand launch."
            )


def generate_launch_description() -> LaunchDescription:
    """Launch one left and one right Hand 2, both disabled by default."""
    single_launch = (
        Path(get_package_share_directory("robo_orchard_wuji2_ros2"))
        / "launch"
        / "wuji_hand2.launch.py"
    )
    options = {
        "connect_timeout": "10.0",
        "auto_enable": "false",
        "publish_joint_diagnostics": "true",
        "publish_imu": "true",
        "publish_fingertip_tactile": "false",
        "reset_duration_s": "2.0",
        "reset_timeout_s": "5.0",
        "reset_feedback_timeout_s": "0.2",
        "reset_position_tolerance": "0.2",
        "reset_joint_position": str([0.0] * 20),
    }
    arguments = [
        DeclareLaunchArgument(f"{side}_{selector}", default_value="")
        for side in ("left", "right")
        for selector in ("endpoint", "serial_number")
    ]
    arguments.extend(
        DeclareLaunchArgument(name, default_value=value)
        for name, value in options.items()
    )
    hands = [
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(single_launch)),
                    launch_arguments={
                        "hand_name": f"hand_{side}",
                        "hand_side": side,
                        "endpoint": LaunchConfiguration(f"{side}_endpoint"),
                        "serial_number": LaunchConfiguration(
                            f"{side}_serial_number"
                        ),
                        **{
                            name: LaunchConfiguration(name) for name in options
                        },
                    }.items(),
                ),
            ],
        )
        for side in ("left", "right")
    ]
    return LaunchDescription(
        [*arguments, OpaqueFunction(function=_validate_selectors), *hands]
    )
