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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robo_orchard_teleop_ros2.wuji_teleop_launch import (
    build_wuji_pair_actions,
    declare_wuji_teleop_arguments,
    resolve_wuji_launch_instances,
)


def _launch_instances(context):
    actions = []
    for instance in resolve_wuji_launch_instances(context, dagger=False):
        override_topic = f"/{instance.hand_name}/control/override"
        actions.extend(build_wuji_pair_actions(instance, override_topic))
    return actions


def generate_launch_description():
    arguments = declare_wuji_teleop_arguments(dagger=False)
    manager_config = DeclareLaunchArgument(
        "control_manager_config_file",
        description=(
            "Control Manager configuration. Custom hand names need a "
            "matching channel configuration."
        ),
    )
    manager = Node(
        package="robo_orchard_control_manager_ros2",
        executable="control_manager_node",
        name="control_manager",
        namespace="/robot/control",
        output="screen",
        parameters=[
            {"config_file": LaunchConfiguration("control_manager_config_file")}
        ],
    )
    return LaunchDescription(
        [
            *arguments,
            manager_config,
            manager,
            OpaqueFunction(function=_launch_instances),
        ]
    )
