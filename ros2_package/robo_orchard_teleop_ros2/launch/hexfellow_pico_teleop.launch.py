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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Launch Hexfellow drivers, Pico teleop and Control Manager together."""
    vr_topic = LaunchConfiguration("vr_topic")
    publish_hz = ParameterValue(
        LaunchConfiguration("publish_hz"), value_type=float
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "chassis_url",
                description="Chassis device address (host:port for KCP).",
            ),
            DeclareLaunchArgument(
                "lift_url",
                description="Lift device address (host:port for KCP).",
            ),
            DeclareLaunchArgument(
                "enable_chassis_bridge", default_value="true"
            ),
            DeclareLaunchArgument("enable_lift_bridge", default_value="true"),
            DeclareLaunchArgument("chassis_is_kcp", default_value="true"),
            DeclareLaunchArgument("lift_is_kcp", default_value="true"),
            DeclareLaunchArgument("chassis_read_only", default_value="false"),
            DeclareLaunchArgument("lift_read_only", default_value="false"),
            DeclareLaunchArgument("start_pico_bridge", default_value="true"),
            DeclareLaunchArgument(
                "start_control_manager", default_value="true"
            ),
            DeclareLaunchArgument(
                "control_manager_config_file",
                description=(
                    "Caller-provided Control Manager configuration mapping "
                    "Hexfellow override candidates to driver topics."
                ),
            ),
            DeclareLaunchArgument(
                "vr_topic", default_value="/pico_bridge/vr_state"
            ),
            DeclareLaunchArgument(
                "chassis_override_topic",
                default_value="/hexfellow_teleop/cmd_vel",
            ),
            DeclareLaunchArgument(
                "lift_override_topic",
                default_value="/hexfellow_teleop/lift_joint_cmd",
            ),
            DeclareLaunchArgument(
                "lift_state_topic", default_value="/lift/motor_states"
            ),
            DeclareLaunchArgument("publish_hz", default_value="30.0"),
            DeclareLaunchArgument(
                "max_linear_x",
                default_value="0.2",
                description="Maximum chassis forward/backward speed in m/s.",
            ),
            DeclareLaunchArgument(
                "max_angular_z",
                default_value="0.2",
                description="Maximum chassis yaw rate in rad/s.",
            ),
            DeclareLaunchArgument(
                "position_rate",
                default_value="0.08",
                description="Lift target position change rate in m/s.",
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
                launch_arguments={
                    "chassis_url": LaunchConfiguration("chassis_url"),
                    "lift_url": LaunchConfiguration("lift_url"),
                    "enable_chassis_bridge": LaunchConfiguration(
                        "enable_chassis_bridge"
                    ),
                    "enable_lift_bridge": LaunchConfiguration(
                        "enable_lift_bridge"
                    ),
                    "chassis_is_kcp": LaunchConfiguration("chassis_is_kcp"),
                    "lift_is_kcp": LaunchConfiguration("lift_is_kcp"),
                    "chassis_read_only": LaunchConfiguration(
                        "chassis_read_only"
                    ),
                    "lift_read_only": LaunchConfiguration("lift_read_only"),
                    "simple_mode": "true",
                }.items(),
            ),
            Node(
                package="robo_orchard_teleop_ros2",
                executable="pico_bridge",
                condition=IfCondition(
                    LaunchConfiguration("start_pico_bridge")
                ),
                name="pico_bridge",
                namespace="/pico_bridge",
                output="screen",
                emulate_tty=True,
                remappings=[("vr_state", vr_topic)],
            ),
            Node(
                package="robo_orchard_teleop_ros2",
                executable="hexfellow_chassis_pico_vr_teleop",
                name="hexfellow_chassis_pico_vr_teleop",
                namespace="/pico_bridge",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "vr_topic": vr_topic,
                        "cmd_vel_topic": LaunchConfiguration(
                            "chassis_override_topic"
                        ),
                        "max_linear_x": ParameterValue(
                            LaunchConfiguration("max_linear_x"),
                            value_type=float,
                        ),
                        "max_angular_z": ParameterValue(
                            LaunchConfiguration("max_angular_z"),
                            value_type=float,
                        ),
                        "publish_hz": publish_hz,
                    }
                ],
            ),
            Node(
                package="robo_orchard_teleop_ros2",
                executable="hexfellow_lift_pico_vr_teleop",
                name="hexfellow_lift_pico_vr_teleop",
                namespace="/pico_bridge",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "vr_topic": vr_topic,
                        "joint_cmd_topic": LaunchConfiguration(
                            "lift_override_topic"
                        ),
                        "state_topic": LaunchConfiguration("lift_state_topic"),
                        "position_rate": ParameterValue(
                            LaunchConfiguration("position_rate"),
                            value_type=float,
                        ),
                        "publish_hz": publish_hz,
                    }
                ],
            ),
            Node(
                package="robo_orchard_control_manager_ros2",
                executable="control_manager_node",
                condition=IfCondition(
                    LaunchConfiguration("start_control_manager")
                ),
                name="control_manager",
                namespace="/robot/control",
                output="screen",
                parameters=[
                    {
                        "config_file": LaunchConfiguration(
                            "control_manager_config_file"
                        )
                    }
                ],
            ),
        ]
    )
