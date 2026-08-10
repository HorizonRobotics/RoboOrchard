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
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robo_orchard_teleop_ros2.wuji_teleop_launch import (
    build_wuji_pair_actions,
    declare_wuji_teleop_arguments,
    resolve_wuji_launch_instances,
)


def _launch_instances(context):
    actions = []
    for instance in resolve_wuji_launch_instances(context, dagger=True):
        hand_command_topic = f"/{instance.hand_name}/joint_commands"
        glove_command_topic = (
            f"{instance.glove_namespace.rstrip('/')}/retargeted_joint_commands"
        )
        hand, glove = build_wuji_pair_actions(instance, glove_command_topic)
        mux = Node(
            package="robo_orchard_teleop_ros2",
            executable="take_over",
            name="wuji_hand_takeover_muxer",
            namespace=f"/{instance.hand_name}/takeover_muxer",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "message_type": "sensor_msgs/msg/JointState",
                    "algo_topic": instance.algo_topic,
                    "override_topic": glove_command_topic,
                    "output_topic": hand_command_topic,
                    "override_mode_behavior": "forward",
                    "replay_time_s": LaunchConfiguration("replay_time_s"),
                }
            ],
        )
        actions.extend((hand, glove, mux))
    return actions


def generate_launch_description():
    arguments = declare_wuji_teleop_arguments(dagger=True)
    return LaunchDescription(
        [*arguments, OpaqueFunction(function=_launch_instances)]
    )
