# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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
    """Launch the complete human takeover system, including the muxer and the hardware controller."""  # noqa: E501
    # --- Declare Launch Arguments ---
    left_algo_topic_arg = DeclareLaunchArgument(
        "left_algo_topic",
        default_value="/left_algo_cmd",
        description="Algorithm command topic for the left arm.",
    )
    left_master_can_port_arg = DeclareLaunchArgument(
        "left_master_can_port",
        default_value="can_left_mst",
        description="CAN port for the left master arm.",
    )
    left_slave_can_port_arg = DeclareLaunchArgument(
        "left_slave_can_port",
        default_value="can_left",
        description="CAN port for the left slave arm.",
    )

    right_algo_topic_arg = DeclareLaunchArgument(
        "right_algo_topic",
        default_value="/right_algo_cmd",
        description="Algorithm command topic for the right arm.",
    )
    right_master_can_port_arg = DeclareLaunchArgument(
        "right_master_can_port",
        default_value="can_right_mst",
        description="CAN port for the right master arm.",
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
    enable_master_mit_control_mode_arg = DeclareLaunchArgument(
        "enable_master_mit_control_mode",
        default_value="true",
        description="Whether enable mit control mode or not.",
    )
    left_reset_joint_position_arg = DeclareLaunchArgument(
        "left_reset_joint_position",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description=(
            "Left follower arm reset target: 6 joint angles (rad) + "
            "gripper. Used by the single_ctrl reset_ctrl service that all "
            "reset sources resolve to."
        ),
    )
    right_reset_joint_position_arg = DeclareLaunchArgument(
        "right_reset_joint_position",
        default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
        description=(
            "Right follower arm reset target: 6 joint angles (rad) + "
            "gripper. See left_reset_joint_position."
        ),
    )
    master_mit_kp_arg = DeclareLaunchArgument(
        "master_mit_kp",
        default_value="10.0",
        description="MIT kp for the master arms.",
    )
    master_mit_kd_arg = DeclareLaunchArgument(
        "master_mit_kd",
        default_value="0.8",
        description="MIT kd for the master arms.",
    )
    master_mit_vel_ref_arg = DeclareLaunchArgument(
        "master_mit_vel_ref",
        default_value="45.0",
        description="MIT velocity reference for the master arms.",
    )
    master_mit_torque_ref_arg = DeclareLaunchArgument(
        "master_mit_torque_ref",
        default_value="0.0",
        description="MIT torque reference for the master arms.",
    )
    follower_mit_kp_arg = DeclareLaunchArgument(
        "follower_mit_kp",
        default_value="10.0",
        description="MIT kp for the follower arms.",
    )
    follower_mit_kd_arg = DeclareLaunchArgument(
        "follower_mit_kd",
        default_value="0.8",
        description="MIT kd for the follower arms.",
    )
    follower_mit_vel_ref_arg = DeclareLaunchArgument(
        "follower_mit_vel_ref",
        default_value="45.0",
        description="MIT velocity reference for the follower arms.",
    )
    follower_mit_torque_ref_arg = DeclareLaunchArgument(
        "follower_mit_torque_ref",
        default_value="0.0",
        description="MIT torque reference for the follower arms.",
    )
    follower_gravity_compensation_enabled_arg = DeclareLaunchArgument(
        "follower_gravity_compensation_enabled",
        default_value="false",
        description=(
            "Master switch for follower MIT gravity compensation. The scale "
            "and per-joint parameters have no effect unless this is true."
        ),
    )
    follower_gravity_compensation_urdf_path_arg = DeclareLaunchArgument(
        "follower_gravity_compensation_urdf_path",
        default_value="/data/holobrain/urdf/piper_x_description.urdf",
        description="URDF path used for follower MIT gravity compensation.",
    )
    follower_gravity_compensation_scale_arg = DeclareLaunchArgument(
        "follower_gravity_compensation_scale",
        default_value="1.0",
        description="Scale applied to follower gravity compensation torque.",
    )
    follower_gravity_compensation_scale_per_joint_arg = DeclareLaunchArgument(
        "follower_gravity_compensation_scale_per_joint",
        default_value="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]",
        description=(
            "Per-joint multiplier (on top of the global scale) for follower "
            "gravity compensation."
        ),
    )
    follower_gravity_compensation_max_t_ref_arg = DeclareLaunchArgument(
        "follower_gravity_compensation_max_t_ref",
        default_value="8.0",
        description="Absolute clamp for follower gravity compensation t_ref.",
    )

    replay_time_s_arg = DeclareLaunchArgument(
        "replay_time_s",
        default_value="0.0",
        description="Replay time (in seconds)",
    )

    # --- Node Definitions ---

    # 1. Takeover Muxer Node
    left_takeover_muxer_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="take_over",
        name="robot_left_takeover_muxer",
        namespace="/robot/left/takeover_muxer",
        output="screen",
        emulate_tty=True,  # Ensures logs are displayed properly
        parameters=[
            {
                "message_type": "sensor_msgs/msg/JointState",
                "algo_topic": LaunchConfiguration("left_algo_topic"),
                "override_topic": "/master/joint_left",
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
        emulate_tty=True,  # Ensures logs are displayed properly
        parameters=[
            {
                "message_type": "sensor_msgs/msg/JointState",
                "algo_topic": LaunchConfiguration("right_algo_topic"),
                "override_topic": "/master/joint_right",
                "output_topic": "/robot/right/joint_cmd",
                "override_mode_behavior": "forward",
                "replay_time_s": LaunchConfiguration("replay_time_s"),
            }
        ],
    )
    left_aloha_orchestrator_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="aloha_orchestrator",
        name="robot_left_aloha_orchestrator",
        namespace="/robot/left/aloha_orchestrator",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "enable_services": [
                    "/robot/left/enable_ctrl",
                    "/robot/left_master/enable_ctrl",
                ],
                "muxer_release_service": "/robot/left/takeover_muxer/release_control",  # noqa: E501
                "muxer_takeover_service": "/robot/left/takeover_muxer/trigger_takeover",  # noqa: E501
            }
        ],
    )
    right_aloha_orchestrator_node = Node(
        package="robo_orchard_teleop_ros2",
        executable="aloha_orchestrator",
        name="robot_right_aloha_orchestrator",
        namespace="/robot/right/aloha_orchestrator",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "enable_services": [
                    "/robot/right/enable_ctrl",
                    "/robot/right_master/enable_ctrl",
                ],
                "muxer_release_service": "/robot/right/takeover_muxer/release_control",  # noqa: E501
                "muxer_takeover_service": "/robot/right/takeover_muxer/trigger_takeover",  # noqa: E501
            }
        ],
    )

    # 2. Aloha Hardware Controller Node
    left_master_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="single_ctrl",
        name="robot_left_master_controller",
        namespace="/robot/left_master",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("left_master_can_port"),
                "auto_enable_arm_ctrl": True,
                "gripper_exist": True,
                "enable_mit_ctrl": LaunchConfiguration(
                    "enable_master_mit_control_mode"
                ),
                "mit_kp": ParameterValue(
                    LaunchConfiguration("master_mit_kp"), value_type=float
                ),
                "mit_kd": ParameterValue(
                    LaunchConfiguration("master_mit_kd"), value_type=float
                ),
                "mit_vel_ref": ParameterValue(
                    LaunchConfiguration("master_mit_vel_ref"),
                    value_type=float,
                ),
                "mit_torque_ref": ParameterValue(
                    LaunchConfiguration("master_mit_torque_ref"),
                    value_type=float,
                ),
            }
        ],
        remappings=[
            ("/robot/left_master/joint_cmd", "/robot/left/joint_cmd"),
            ("/robot/left_master/status", "/master/status_left"),
            ("/robot/left_master/ee_pose", "/master/end_pose_left"),
            ("/robot/left_master/joint_state", "/master/joint_left"),
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
                "mit_kp": ParameterValue(
                    LaunchConfiguration("follower_mit_kp"), value_type=float
                ),
                "mit_kd": ParameterValue(
                    LaunchConfiguration("follower_mit_kd"), value_type=float
                ),
                "mit_vel_ref": ParameterValue(
                    LaunchConfiguration("follower_mit_vel_ref"),
                    value_type=float,
                ),
                "mit_torque_ref": ParameterValue(
                    LaunchConfiguration("follower_mit_torque_ref"),
                    value_type=float,
                ),
                "mit_gravity_compensation_enabled": ParameterValue(
                    LaunchConfiguration(
                        "follower_gravity_compensation_enabled"
                    ),
                    value_type=bool,
                ),
                "mit_gravity_compensation_urdf_path": LaunchConfiguration(
                    "follower_gravity_compensation_urdf_path"
                ),
                "mit_gravity_compensation_scale": ParameterValue(
                    LaunchConfiguration(
                        "follower_gravity_compensation_scale"
                    ),
                    value_type=float,
                ),
                "mit_gravity_compensation_scale_per_joint": ParameterValue(
                    LaunchConfiguration(
                        "follower_gravity_compensation_scale_per_joint"
                    ),
                    value_type=List[float],
                ),
                "mit_gravity_compensation_max_t_ref": ParameterValue(
                    LaunchConfiguration(
                        "follower_gravity_compensation_max_t_ref"
                    ),
                    value_type=float,
                ),
            }
        ],
        remappings=[
            ("/robot/left/status", "/puppet/status_left"),
            ("/robot/left/ee_pose", "/puppet/end_pose_left"),
            ("/robot/left/joint_state", "/puppet/joint_left"),
        ],
    )
    right_master_controller_node = Node(
        package="robo_orchard_piper_ros2",
        executable="single_ctrl",
        name="robot_right_master_controller",
        namespace="/robot/right_master",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_port": LaunchConfiguration("right_master_can_port"),
                "auto_enable_arm_ctrl": True,
                "gripper_exist": True,
                "enable_mit_ctrl": LaunchConfiguration(
                    "enable_master_mit_control_mode"
                ),
                "mit_kp": ParameterValue(
                    LaunchConfiguration("master_mit_kp"), value_type=float
                ),
                "mit_kd": ParameterValue(
                    LaunchConfiguration("master_mit_kd"), value_type=float
                ),
                "mit_vel_ref": ParameterValue(
                    LaunchConfiguration("master_mit_vel_ref"),
                    value_type=float,
                ),
                "mit_torque_ref": ParameterValue(
                    LaunchConfiguration("master_mit_torque_ref"),
                    value_type=float,
                ),
            }
        ],
        remappings=[
            ("/robot/right_master/joint_cmd", "/robot/right/joint_cmd"),
            ("/robot/right_master/status", "/master/status_right"),
            ("/robot/right_master/ee_pose", "/master/end_pose_right"),
            ("/robot/right_master/joint_state", "/master/joint_right"),
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
                "mit_kp": ParameterValue(
                    LaunchConfiguration("follower_mit_kp"), value_type=float
                ),
                "mit_kd": ParameterValue(
                    LaunchConfiguration("follower_mit_kd"), value_type=float
                ),
                "mit_vel_ref": ParameterValue(
                    LaunchConfiguration("follower_mit_vel_ref"),
                    value_type=float,
                ),
                "mit_torque_ref": ParameterValue(
                    LaunchConfiguration("follower_mit_torque_ref"),
                    value_type=float,
                ),
                "mit_gravity_compensation_enabled": ParameterValue(
                    LaunchConfiguration(
                        "follower_gravity_compensation_enabled"
                    ),
                    value_type=bool,
                ),
                "mit_gravity_compensation_urdf_path": LaunchConfiguration(
                    "follower_gravity_compensation_urdf_path"
                ),
                "mit_gravity_compensation_scale": ParameterValue(
                    LaunchConfiguration(
                        "follower_gravity_compensation_scale"
                    ),
                    value_type=float,
                ),
                "mit_gravity_compensation_scale_per_joint": ParameterValue(
                    LaunchConfiguration(
                        "follower_gravity_compensation_scale_per_joint"
                    ),
                    value_type=List[float],
                ),
                "mit_gravity_compensation_max_t_ref": ParameterValue(
                    LaunchConfiguration(
                        "follower_gravity_compensation_max_t_ref"
                    ),
                    value_type=float,
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
            # Add the declared arguments
            left_algo_topic_arg,
            left_master_can_port_arg,
            left_slave_can_port_arg,
            right_algo_topic_arg,
            right_master_can_port_arg,
            right_slave_can_port_arg,
            enable_mit_control_mode_arg,
            enable_master_mit_control_mode_arg,
            master_mit_kp_arg,
            master_mit_kd_arg,
            master_mit_vel_ref_arg,
            master_mit_torque_ref_arg,
            follower_mit_kp_arg,
            follower_mit_kd_arg,
            follower_mit_vel_ref_arg,
            follower_mit_torque_ref_arg,
            follower_gravity_compensation_enabled_arg,
            follower_gravity_compensation_urdf_path_arg,
            follower_gravity_compensation_scale_arg,
            follower_gravity_compensation_scale_per_joint_arg,
            follower_gravity_compensation_max_t_ref_arg,
            replay_time_s_arg,
            left_reset_joint_position_arg,
            right_reset_joint_position_arg,
            # Add the nodes to be launched
            left_takeover_muxer_node,
            left_master_controller_node,
            left_controller_node,
            right_takeover_muxer_node,
            right_master_controller_node,
            right_controller_node,
            left_aloha_orchestrator_node,
            right_aloha_orchestrator_node,
        ]
    )
