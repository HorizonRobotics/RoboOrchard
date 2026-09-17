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
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Aloha teleoperation through one Control Manager."""
    joint_names_args = [
        DeclareLaunchArgument(
            f"{side}_joint_names",
            default_value=(
                f"[{side}_joint1, {side}_joint2, {side}_joint3, "
                f"{side}_joint4, {side}_joint5, {side}_joint6, "
                f"{side}_gripper]"
            ),
            description="Names in hardware order: six joints, then gripper.",
        )
        for side in ("left", "right")
    ]
    arguments = [
        DeclareLaunchArgument(
            "left_master_can_port",
            default_value="can_left_mst",
            description="CAN port for the left master arm.",
        ),
        DeclareLaunchArgument(
            "left_slave_can_port",
            default_value="can_left",
            description="CAN port for the left slave arm.",
        ),
        DeclareLaunchArgument(
            "right_master_can_port",
            default_value="can_right_mst",
            description="CAN port for the right master arm.",
        ),
        DeclareLaunchArgument(
            "right_slave_can_port",
            default_value="can_right",
            description="CAN port for the right slave arm.",
        ),
        DeclareLaunchArgument(
            "enable_mit_ctrl",
            default_value="true",
            description="Whether to enable MIT control on follower arms.",
        ),
        DeclareLaunchArgument(
            "enable_master_mit_ctrl",
            default_value="false",
            description="Whether to enable MIT control on master arms.",
        ),
        DeclareLaunchArgument(
            "control_manager_config_file",
            description="Control Manager configuration file.",
        ),
        DeclareLaunchArgument(
            "left_base_frame_id",
            default_value="left_base_link",
            description="Base frame of the left follower arm.",
        ),
        DeclareLaunchArgument(
            "left_ee_frame_id",
            default_value="left_end_effector",
            description="End-effector frame of the left follower arm.",
        ),
        DeclareLaunchArgument(
            "right_base_frame_id",
            default_value="right_base_link",
            description="Base frame of the right follower arm.",
        ),
        DeclareLaunchArgument(
            "right_ee_frame_id",
            default_value="right_end_effector",
            description="End-effector frame of the right follower arm.",
        ),
        DeclareLaunchArgument(
            "left_master_base_frame_id",
            default_value="left_master_base_link",
            description="Base frame of the left master arm.",
        ),
        DeclareLaunchArgument(
            "left_master_ee_frame_id",
            default_value="left_master_end_effector",
            description="End-effector frame of the left master arm.",
        ),
        DeclareLaunchArgument(
            "right_master_base_frame_id",
            default_value="right_master_base_link",
            description="Base frame of the right master arm.",
        ),
        DeclareLaunchArgument(
            "right_master_ee_frame_id",
            default_value="right_master_end_effector",
            description="End-effector frame of the right master arm.",
        ),
        DeclareLaunchArgument(
            "publish_ee_tf",
            default_value="true",
            description="Publish dynamic end-effector TF for follower arms.",
        ),
        DeclareLaunchArgument(
            "publish_master_ee_tf",
            default_value="false",
            description="Publish dynamic end-effector TF for master arms.",
        ),
    ]

    managed_aloha = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("robo_orchard_teleop_ros2"),
                    "launch",
                    "piper_dagger_compat.launch.py",
                ]
            )
        ),
        launch_arguments={
            "left_joint_names": LaunchConfiguration("left_joint_names"),
            "right_joint_names": LaunchConfiguration("right_joint_names"),
            "left_master_can_port": LaunchConfiguration(
                "left_master_can_port"
            ),
            "left_slave_can_port": LaunchConfiguration("left_slave_can_port"),
            "right_master_can_port": LaunchConfiguration(
                "right_master_can_port"
            ),
            "right_slave_can_port": LaunchConfiguration(
                "right_slave_can_port"
            ),
            "enable_mit_control_mode": LaunchConfiguration("enable_mit_ctrl"),
            "enable_master_mit_control_mode": LaunchConfiguration(
                "enable_master_mit_ctrl"
            ),
            "control_manager_config_file": LaunchConfiguration(
                "control_manager_config_file"
            ),
            "left_base_frame_id": LaunchConfiguration("left_base_frame_id"),
            "left_ee_frame_id": LaunchConfiguration("left_ee_frame_id"),
            "right_base_frame_id": LaunchConfiguration("right_base_frame_id"),
            "right_ee_frame_id": LaunchConfiguration("right_ee_frame_id"),
            "left_master_base_frame_id": LaunchConfiguration(
                "left_master_base_frame_id"
            ),
            "left_master_ee_frame_id": LaunchConfiguration(
                "left_master_ee_frame_id"
            ),
            "right_master_base_frame_id": LaunchConfiguration(
                "right_master_base_frame_id"
            ),
            "right_master_ee_frame_id": LaunchConfiguration(
                "right_master_ee_frame_id"
            ),
            "publish_ee_tf": LaunchConfiguration("publish_ee_tf"),
            "publish_master_ee_tf": LaunchConfiguration(
                "publish_master_ee_tf"
            ),
        }.items(),
    )
    return LaunchDescription([*joint_names_args, *arguments, managed_aloha])
