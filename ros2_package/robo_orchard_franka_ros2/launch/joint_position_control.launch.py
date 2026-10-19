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

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_robot_nodes(
    context: LaunchContext,
    robot_ip: LaunchConfiguration,
    arm_id: LaunchConfiguration,
    load_gripper: LaunchConfiguration,
) -> list:
    arm_id_value = context.perform_substitution(arm_id)
    gripper_enabled = IfCondition(load_gripper).evaluate(context)
    robot_description_path = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        arm_id_value,
        f"{arm_id_value}.urdf.xacro",
    )
    robot_description = xacro.process_file(
        robot_description_path,
        mappings={
            "arm_id": arm_id_value,
            "robot_ip": context.perform_substitution(robot_ip),
            "hand": "true" if gripper_enabled else "false",
            "ros2_control": "true",
            "use_fake_hardware": "false",
            "fake_sensor_commands": "false",
        },
    ).toprettyxml(indent="  ")
    controllers_path = os.path.join(
        get_package_share_directory("robo_orchard_franka_ros2"),
        "config",
        f"{arm_id_value}_controllers.yaml",
    )

    joint_state_sources = ["franka/joint_states"]
    if gripper_enabled:
        joint_state_sources.append(f"{arm_id_value}_gripper/joint_states")

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                controllers_path,
                {"robot_description": robot_description},
                {"arm_id": arm_id},
            ],
            remappings=[("joint_states", "franka/joint_states")],
            output={"stdout": "screen", "stderr": "screen"},
            on_exit=Shutdown(),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[{"source_list": joint_state_sources, "rate": 30}],
        ),
    ]
    if gripper_enabled:
        nodes.append(
            Node(
                package="franka_gripper",
                executable="franka_gripper_node",
                name=f"{arm_id_value}_gripper",
                parameters=[
                    os.path.join(
                        get_package_share_directory("franka_gripper"),
                        "config",
                        "franka_gripper_node.yaml",
                    ),
                    {
                        "robot_ip": context.perform_substitution(robot_ip),
                        "joint_names": [
                            f"{arm_id_value}_finger_joint1",
                            f"{arm_id_value}_finger_joint2",
                        ],
                    },
                ],
                output="screen",
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    robot_ip = LaunchConfiguration("robot_ip")
    arm_id = LaunchConfiguration("arm_id")
    use_rviz = LaunchConfiguration("use_rviz")
    load_gripper = LaunchConfiguration("load_gripper")

    rviz_config = os.path.join(
        get_package_share_directory("franka_description"),
        "rviz",
        "visualize_franka.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip",
                description="Hostname or IP address of the Franka robot.",
            ),
            DeclareLaunchArgument(
                "arm_id",
                default_value="fr3",
                description="Franka robot model and joint name prefix.",
            ),
            DeclareLaunchArgument(
                "load_gripper",
                default_value="false",
                description="Load the native Franka gripper and its driver.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Visualize the robot in RViz.",
            ),
            OpaqueFunction(
                function=_create_robot_nodes,
                args=[robot_ip, arm_id, load_gripper],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["franka_robot_state_broadcaster"],
                parameters=[{"arm_id": arm_id}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_position_command_controller"],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
