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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_ip = LaunchConfiguration("robot_ip")
    use_rviz = LaunchConfiguration("use_rviz")
    load_gripper = LaunchConfiguration("load_gripper")
    urdf_path = LaunchConfiguration("urdf_path")
    source_type = LaunchConfiguration("source_type")
    control_frequency_hz = LaunchConfiguration("control_frequency_hz")
    stale_timeout_s = LaunchConfiguration("stale_timeout_s")
    ik_regularization_parameter = LaunchConfiguration(
        "ik_regularization_parameter"
    )
    control_manager_config_file = LaunchConfiguration(
        "control_manager_config_file"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                description="Hostname or IP address of the Franka robot.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Visualize the robot in RViz.",
            ),
            DeclareLaunchArgument(
                "load_gripper",
                default_value="false",
                description="Start the native gripper without Pico control.",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("franka_description"),
                        "robots",
                        "fr3",
                        "fr3.urdf.xacro",
                    ]
                ),
                description="FR3 URDF or xacro used by IK.",
            ),
            DeclareLaunchArgument(
                "source_type",
                default_value="right",
                description="Pico controller side: left or right.",
            ),
            DeclareLaunchArgument(
                "control_frequency_hz",
                default_value="90.0",
                description="IK and arm command publication frequency.",
            ),
            DeclareLaunchArgument(
                "stale_timeout_s",
                default_value="0.3",
                description="Maximum Pico and arm feedback age in seconds.",
            ),
            DeclareLaunchArgument(
                "ik_regularization_parameter",
                default_value="0.005",
                description="IK seed regularization used for Franka.",
            ),
            DeclareLaunchArgument(
                "control_manager_config_file",
                description="Project-owned Control Manager configuration.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("robo_orchard_franka_ros2"),
                            "launch",
                            "joint_position_control.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "robot_ip": robot_ip,
                    "arm_id": "fr3",
                    "use_rviz": use_rviz,
                    "load_gripper": load_gripper,
                }.items(),
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
                executable="franka_pico_vr_teleop",
                name="franka_pico_vr_teleop",
                namespace="/pico_bridge",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "urdf_path": urdf_path,
                        "source_type": source_type,
                        "base_link_name": "fr3_link0",
                        "ee_link_name": "fr3_link8",
                        "joint_state_topic": "/franka/joint_states",
                        "joint_command_topic": "/robot/franka/joint_cmd",
                        "reset_service": "/robot/control/reset",
                        "control_frequency_hz": ParameterValue(
                            control_frequency_hz,
                            value_type=float,
                        ),
                        "stale_timeout_s": ParameterValue(
                            stale_timeout_s,
                            value_type=float,
                        ),
                        "ik_regularization_parameter": ParameterValue(
                            ik_regularization_parameter,
                            value_type=float,
                        ),
                    }
                ],
                remappings=[
                    (
                        "/robot/franka/joint_cmd",
                        "/franka_teleop/joint_franka",
                    ),
                ],
            ),
            Node(
                package="robo_orchard_control_manager_ros2",
                executable="control_manager_node",
                name="control_manager",
                namespace="/robot/control",
                output="screen",
                parameters=[{"config_file": control_manager_config_file}],
            ),
        ]
    )
