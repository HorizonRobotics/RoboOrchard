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
    """Launch the complete VR + Dagger takeover system."""
    left_algo_topic_arg = DeclareLaunchArgument(
        "left_algo_topic",
        default_value="/left_algo_cmd",
        description="Algorithm command topic for the left arm.",
    )
    right_algo_topic_arg = DeclareLaunchArgument(
        "right_algo_topic",
        default_value="/right_algo_cmd",
        description="Algorithm command topic for the right arm.",
    )
    left_slave_can_port_arg = DeclareLaunchArgument(
        "left_slave_can_port",
        default_value="can_left",
        description="CAN port for the left slave arm.",
    )
    right_slave_can_port_arg = DeclareLaunchArgument(
        "right_slave_can_port",
        default_value="can_right",
        description="CAN port for the right slave arm.",
    )
    enable_mit_control_mode_arg = DeclareLaunchArgument(
        "enable_mit_control_mode",
        default_value="true",
        description="Whether enable mit control mode or not.",
    )
    replay_time_s_arg = DeclareLaunchArgument(
        "replay_time_s",
        default_value="0.0",
        description="Replay time (in seconds).",
    )
    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value="",
        description="URDF path used by the Pico VR teleop IK solver.",
    )
    match_tolerance_arg = DeclareLaunchArgument(
        "match_tolerance",
        default_value="0.1",
        description=(
            "Normalised trigger-vs-gripper match tolerance for "
            "match-before-engage handoff."
        ),
    )
    left_reset_joint_position_arg = DeclareLaunchArgument(
        "left_reset_joint_position",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description=(
            "Left arm reset target: 6 joint angles (rad) + gripper. "
            "All reset sources (VR reset gesture, frontend button) "
            "resolve to the single_ctrl reset_ctrl service, which uses "
            "this value."
        ),
    )
    right_reset_joint_position_arg = DeclareLaunchArgument(
        "right_reset_joint_position",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description=(
            "Right arm reset target: 6 joint angles (rad) + gripper. "
            "See left_reset_joint_position."
        ),
    )

    left_takeover_muxer_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="take_over",
        name="robot_left_takeover_muxer",
        namespace="/robot/left/takeover_muxer",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "message_type": "sensor_msgs/msg/JointState",
                "algo_topic": LaunchConfiguration("left_algo_topic"),
                "override_topic": "/pico_teleop/joint_left",
                "output_topic": "/robot/left/joint_cmd",
                "override_mode_behavior": "forward",
                "replay_time_s": LaunchConfiguration("replay_time_s"),
            }
        ],
    )
    right_takeover_muxer_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="take_over",
        name="robot_right_takeover_muxer",
        namespace="/robot/right/takeover_muxer",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "message_type": "sensor_msgs/msg/JointState",
                "algo_topic": LaunchConfiguration("right_algo_topic"),
                "override_topic": "/pico_teleop/joint_right",
                "output_topic": "/robot/right/joint_cmd",
                "override_mode_behavior": "forward",
                "replay_time_s": LaunchConfiguration("replay_time_s"),
            }
        ],
    )

    left_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="single_ctrl",
        name="robot_left_controller",
        namespace="/robot/left",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("left_slave_can_port"),
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
        name="robot_right_controller",
        namespace="/robot/right",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("right_slave_can_port"),
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

    left_vr_orchestrator_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="vr_orchestrator",
        name="robot_left_vr_orchestrator",
        namespace="/robot/left/vr_orchestrator",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "enable_services": ["/robot/left/enable_ctrl"],
                "muxer_release_service": (
                    "/robot/left/takeover_muxer/release_control"
                ),
                "muxer_takeover_service": (
                    "/robot/left/takeover_muxer/trigger_takeover"
                ),
            }
        ],
    )
    right_vr_orchestrator_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="vr_orchestrator",
        name="robot_right_vr_orchestrator",
        namespace="/robot/right/vr_orchestrator",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "enable_services": ["/robot/right/enable_ctrl"],
                "muxer_release_service": (
                    "/robot/right/takeover_muxer/release_control"
                ),
                "muxer_takeover_service": (
                    "/robot/right/takeover_muxer/trigger_takeover"
                ),
            }
        ],
    )

    pico_bridge_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="pico_bridge",
        name="pico_bridge",
        namespace="/pico_bridge",
        output="screen",
        emulate_tty=True,
    )

    piper_pico_vr_teleop_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="piper_pico_vr_teleop",
        name="piper_pico_vr_teleop",
        namespace="/pico_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "urdf_path": LaunchConfiguration("urdf_path"),
                "ee_link_name": "link6",
                "base_link_name": "base_link",
                "left_reset_service": "/robot/left/reset_ctrl",
                "right_reset_service": "/robot/right/reset_ctrl",
                "match_tolerance": LaunchConfiguration("match_tolerance"),
            }
        ],
        remappings=[
            ("/robot/left/joint_cmd", "/pico_teleop/joint_left"),
            ("/robot/right/joint_cmd", "/pico_teleop/joint_right"),
            ("/robot/left/ee_pose", "/puppet/end_pose_left"),
            ("/robot/left/joint_state", "/puppet/joint_left"),
            ("/robot/right/ee_pose", "/puppet/end_pose_right"),
            ("/robot/right/joint_state", "/puppet/joint_right"),
        ],
    )

    return LaunchDescription(
        [
            left_algo_topic_arg,
            right_algo_topic_arg,
            left_slave_can_port_arg,
            right_slave_can_port_arg,
            enable_mit_control_mode_arg,
            replay_time_s_arg,
            urdf_path_arg,
            match_tolerance_arg,
            left_reset_joint_position_arg,
            right_reset_joint_position_arg,
            left_takeover_muxer_node,
            right_takeover_muxer_node,
            left_controller_node,
            right_controller_node,
            left_vr_orchestrator_node,
            right_vr_orchestrator_node,
            pico_bridge_node,
            piper_pico_vr_teleop_node,
        ]
    )
