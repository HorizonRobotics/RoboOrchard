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

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch two Piper drivers behind one Control Manager."""
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
    # --- Declare Launch Arguments ---
    left_can_port_arg = DeclareLaunchArgument(
        "left_can_port",
        default_value="can_left",
        description="CAN port for the left arm.",
    )
    right_can_port_arg = DeclareLaunchArgument(
        "right_can_port",
        default_value="can_right",
        description="CAN port for the right arm.",
    )
    left_base_frame_id_arg = DeclareLaunchArgument(
        "left_base_frame_id",
        default_value="left_base_link",
        description="Base frame of the left follower arm.",
    )
    left_ee_frame_id_arg = DeclareLaunchArgument(
        "left_ee_frame_id",
        default_value="left_end_effector",
        description="End-effector frame of the left follower arm.",
    )
    right_base_frame_id_arg = DeclareLaunchArgument(
        "right_base_frame_id",
        default_value="right_base_link",
        description="Base frame of the right follower arm.",
    )
    right_ee_frame_id_arg = DeclareLaunchArgument(
        "right_ee_frame_id",
        default_value="right_end_effector",
        description="End-effector frame of the right follower arm.",
    )
    publish_ee_tf_arg = DeclareLaunchArgument(
        "publish_ee_tf",
        default_value="true",
        description="Publish dynamic end-effector TF for follower arms.",
    )
    enable_mit_control_mode_arg = DeclareLaunchArgument(
        "enable_mit_control_mode",
        default_value="true",
        description="Whether enable mit control mode or not.",
    )
    left_reset_joint_position_arg = DeclareLaunchArgument(
        "left_reset_joint_position",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description=(
            "Left arm reset target: 6 joint angles (rad) + gripper. Used "
            "by the single_ctrl reset_ctrl service."
        ),
    )
    right_reset_joint_position_arg = DeclareLaunchArgument(
        "right_reset_joint_position",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description=(
            "Right arm reset target: 6 joint angles (rad) + gripper. See "
            "left_reset_joint_position."
        ),
    )
    control_manager_config_file_arg = DeclareLaunchArgument(
        "control_manager_config_file",
        description="Control Manager configuration file.",
    )

    # --- Node Definitions ---
    control_manager_node = Node(
        package="robo_orchard_control_manager_ros2",
        executable="control_manager_node",
        name="control_manager",
        namespace="/robot/control",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"config_file": LaunchConfiguration("control_manager_config_file")}
        ],
    )
    left_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="single_ctrl",
        name="robot_left_single_controller",
        namespace="/robot/left",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "joint_names": ParameterValue(
                    LaunchConfiguration("left_joint_names"),
                    value_type=List[str],
                ),
                "can_port": LaunchConfiguration("left_can_port"),
                "base_frame_id": ParameterValue(
                    LaunchConfiguration("left_base_frame_id"), value_type=str
                ),
                "ee_frame_id": ParameterValue(
                    LaunchConfiguration("left_ee_frame_id"), value_type=str
                ),
                "publish_ee_tf": ParameterValue(
                    LaunchConfiguration("publish_ee_tf"), value_type=bool
                ),
                "auto_enable_arm_ctrl": True,
                "gripper_exist": True,
                "enable_mit_ctrl": LaunchConfiguration(
                    "enable_mit_control_mode"
                ),
                "reset_joint_position": ParameterValue(
                    LaunchConfiguration("left_reset_joint_position"),
                    value_type=List[float],
                ),
            }
        ],
        remappings=[
            ("/robot/left/status", "/puppet/status_left"),
            ("/robot/left/ee_pose", "/puppet/end_pose_left"),
            ("/robot/left/joint_state", "/puppet/joint_left"),
        ],
    )
    right_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="single_ctrl",
        name="robot_right_single_controller",
        namespace="/robot/right",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "joint_names": ParameterValue(
                    LaunchConfiguration("right_joint_names"),
                    value_type=List[str],
                ),
                "can_port": LaunchConfiguration("right_can_port"),
                "base_frame_id": ParameterValue(
                    LaunchConfiguration("right_base_frame_id"), value_type=str
                ),
                "ee_frame_id": ParameterValue(
                    LaunchConfiguration("right_ee_frame_id"), value_type=str
                ),
                "publish_ee_tf": ParameterValue(
                    LaunchConfiguration("publish_ee_tf"), value_type=bool
                ),
                "auto_enable_arm_ctrl": True,
                "gripper_exist": True,
                "enable_mit_ctrl": LaunchConfiguration(
                    "enable_mit_control_mode"
                ),
                "reset_joint_position": ParameterValue(
                    LaunchConfiguration("right_reset_joint_position"),
                    value_type=List[float],
                ),
            }
        ],
        remappings=[
            ("/robot/right/status", "/puppet/status_right"),
            ("/robot/right/ee_pose", "/puppet/end_pose_right"),
            ("/robot/right/joint_state", "/puppet/joint_right"),
        ],
    )

    # --- Create the Launch Description ---
    # The launch description is a container for all the actions to be executed.
    return LaunchDescription(
        [
            *joint_names_args,
            # Add the declared arguments
            left_can_port_arg,
            right_can_port_arg,
            left_base_frame_id_arg,
            left_ee_frame_id_arg,
            right_base_frame_id_arg,
            right_ee_frame_id_arg,
            publish_ee_tf_arg,
            enable_mit_control_mode_arg,
            left_reset_joint_position_arg,
            right_reset_joint_position_arg,
            control_manager_config_file_arg,
            # Add the nodes to be launched
            control_manager_node,
            left_controller_node,
            right_controller_node,
        ]
    )
