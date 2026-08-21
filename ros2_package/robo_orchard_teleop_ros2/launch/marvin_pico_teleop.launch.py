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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = LaunchConfiguration("urdf_path")
    driver_config_file = LaunchConfiguration("driver_config_file")
    auto_enable_side = LaunchConfiguration("auto_enable_side")
    auto_enable_mode = LaunchConfiguration("auto_enable_mode")
    control_frequency_hz = LaunchConfiguration("control_frequency_hz")
    translation_scale_factor = LaunchConfiguration("translation_scale_factor")
    pose_low_pass_alpha = LaunchConfiguration("pose_low_pass_alpha")
    operator_input_source = LaunchConfiguration("operator_input_source")
    keyboard_control_side = LaunchConfiguration("keyboard_control_side")
    keyboard_activation_topic = LaunchConfiguration(
        "keyboard_activation_topic"
    )
    keyboard_reset_topic = LaunchConfiguration("keyboard_reset_topic")
    keyboard_activation_timeout_s = LaunchConfiguration(
        "keyboard_activation_timeout_s"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robo_orchard_marvin_ros2"),
                        "urdf",
                        "marvin_m6s_ccs_696_v4.urdf",
                    ]
                ),
                description=(
                    "Dual-arm Marvin URDF used for IK. Defaults to the model "
                    "shipped with the driver. An override must contain "
                    "robot_stand, TCP_Link_L, and TCP_Link_R."
                ),
            ),
            DeclareLaunchArgument(
                "driver_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robo_orchard_marvin_ros2"),
                        "config",
                        "marvin_m6s.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("auto_enable_side", default_value="both"),
            DeclareLaunchArgument(
                "auto_enable_mode", default_value="joint_impedance"
            ),
            DeclareLaunchArgument(
                "control_frequency_hz", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "translation_scale_factor", default_value="1.0"
            ),
            DeclareLaunchArgument("pose_low_pass_alpha", default_value="0.25"),
            DeclareLaunchArgument(
                "operator_input_source",
                default_value="pico",
                description="Operator input source: pico or keyboard.",
            ),
            DeclareLaunchArgument(
                "keyboard_control_side",
                default_value="both",
                description="Keyboard-controlled arm: left, right, or both.",
            ),
            DeclareLaunchArgument(
                "keyboard_activation_topic",
                default_value="/teleop/activation/state",
            ),
            DeclareLaunchArgument(
                "keyboard_reset_topic",
                default_value="/teleop/reset",
            ),
            DeclareLaunchArgument(
                "keyboard_activation_timeout_s",
                default_value="0.2",
            ),
            Node(
                package="robo_orchard_marvin_ros2",
                executable="marvin_driver_node",
                name="marvin_driver_node",
                output="screen",
                parameters=[
                    driver_config_file,
                    {
                        "auto_enable_side": auto_enable_side,
                        "auto_enable_mode": auto_enable_mode,
                    },
                ],
            ),
            Node(
                package="robo_orchard_teleop_ros2",
                executable="pico_bridge",
                name="pico_bridge",
                namespace="/pico_bridge",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="robo_orchard_teleop_ros2",
                executable="marvin_pico_vr_teleop",
                name="marvin_pico_vr_teleop",
                namespace="/pico_bridge",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "urdf_path": urdf_path,
                        "base_link_name": "robot_stand",
                        "left_ee_link_name": "TCP_Link_L",
                        "right_ee_link_name": "TCP_Link_R",
                        "control_frequency_hz": ParameterValue(
                            control_frequency_hz, value_type=float
                        ),
                        "translation_scale_factor": ParameterValue(
                            translation_scale_factor, value_type=float
                        ),
                        "pose_low_pass_alpha": ParameterValue(
                            pose_low_pass_alpha, value_type=float
                        ),
                        "operator_input_source": operator_input_source,
                        "keyboard_control_side": keyboard_control_side,
                        "keyboard_activation_topic": (
                            keyboard_activation_topic
                        ),
                        "keyboard_reset_topic": keyboard_reset_topic,
                        "keyboard_activation_timeout_s": ParameterValue(
                            keyboard_activation_timeout_s,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
