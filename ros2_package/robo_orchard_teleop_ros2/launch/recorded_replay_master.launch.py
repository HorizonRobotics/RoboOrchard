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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("reference_csv", default_value=""),
            DeclareLaunchArgument("tracking_reference_csv", default_value=""),
            DeclareLaunchArgument(
                "left_command_topic", default_value="/left_algo_cmd"
            ),
            DeclareLaunchArgument(
                "right_command_topic", default_value="/right_algo_cmd"
            ),
            DeclareLaunchArgument(
                "left_state_topic", default_value="/puppet/joint_left"
            ),
            DeclareLaunchArgument(
                "right_state_topic", default_value="/puppet/joint_right"
            ),
            DeclareLaunchArgument("publish_left", default_value="true"),
            DeclareLaunchArgument("publish_right", default_value="true"),
            DeclareLaunchArgument("rate_hz", default_value="100.0"),
            DeclareLaunchArgument("start_delay_s", default_value="1.0"),
            DeclareLaunchArgument("start_trigger_file", default_value=""),
            DeclareLaunchArgument("ready_file", default_value=""),
            DeclareLaunchArgument(
                "min_command_subscribers", default_value="0"
            ),
            DeclareLaunchArgument(
                "command_subscriber_wait_timeout_s", default_value="2.0"
            ),
            DeclareLaunchArgument("reset_before_start", default_value="false"),
            DeclareLaunchArgument(
                "reset_pose",
                default_value="0.0,0.10,-0.70,0.0,0.0,0.0,0.0",
            ),
            DeclareLaunchArgument(
                "reset_param_nodes",
                default_value=(
                    "/robot/left/robot_left_controller,"
                    "/robot/right/robot_right_controller"
                ),
            ),
            DeclareLaunchArgument(
                "reset_services",
                default_value="/robot/left/reset_ctrl,/robot/right/reset_ctrl",
            ),
            DeclareLaunchArgument(
                "reset_service_timeout_s", default_value="25.0"
            ),
            DeclareLaunchArgument("reset_settle_s", default_value="0.5"),
            DeclareLaunchArgument(
                "replay_speed_path_s_per_s", default_value="0.25"
            ),
            DeclareLaunchArgument("use_demo_timing", default_value="false"),
            DeclareLaunchArgument("demo_speed_scale", default_value="1.0"),
            DeclareLaunchArgument("reverse", default_value="false"),
            DeclareLaunchArgument("bidirectional", default_value="false"),
            DeclareLaunchArgument("approach_start_s", default_value="0.0"),
            DeclareLaunchArgument(
                "approach_start_max_error_rad", default_value="1.0"
            ),
            DeclareLaunchArgument("hold_start_s", default_value="2.0"),
            DeclareLaunchArgument("hold_turnaround_s", default_value="2.0"),
            DeclareLaunchArgument("hold_end_s", default_value="2.0"),
            DeclareLaunchArgument("loop", default_value="false"),
            DeclareLaunchArgument("kp", default_value="10.0"),
            DeclareLaunchArgument("kd", default_value="0.8"),
            DeclareLaunchArgument("torque_ref", default_value="0.0"),
            DeclareLaunchArgument("use_message_velocity", default_value="false"),
            DeclareLaunchArgument("state_timeout_s", default_value="0.25"),
            DeclareLaunchArgument(
                "tracking_error_stop_rad", default_value="0.25"
            ),
            DeclareLaunchArgument(
                "tracking_error_warn_rad", default_value="0.12"
            ),
            DeclareLaunchArgument(
                "reference_feedback_gain", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "reference_feedback_max_offset_rad", default_value="0.0"
            ),
            DeclareLaunchArgument("output_csv", default_value=""),
            DeclareLaunchArgument("log_decimation", default_value="1"),
            Node(
                package="robo_orchard_teleop_ros2",
                executable="recorded_replay_master",
                name="recorded_replay_master",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "reference_csv": LaunchConfiguration("reference_csv"),
                        "tracking_reference_csv": LaunchConfiguration(
                            "tracking_reference_csv"
                        ),
                        "left_command_topic": LaunchConfiguration(
                            "left_command_topic"
                        ),
                        "right_command_topic": LaunchConfiguration(
                            "right_command_topic"
                        ),
                        "left_state_topic": LaunchConfiguration(
                            "left_state_topic"
                        ),
                        "right_state_topic": LaunchConfiguration(
                            "right_state_topic"
                        ),
                        "publish_left": ParameterValue(
                            LaunchConfiguration("publish_left"), value_type=bool
                        ),
                        "publish_right": ParameterValue(
                            LaunchConfiguration("publish_right"), value_type=bool
                        ),
                        "rate_hz": ParameterValue(
                            LaunchConfiguration("rate_hz"), value_type=float
                        ),
                        "start_delay_s": ParameterValue(
                            LaunchConfiguration("start_delay_s"), value_type=float
                        ),
                        "start_trigger_file": LaunchConfiguration(
                            "start_trigger_file"
                        ),
                        "ready_file": LaunchConfiguration("ready_file"),
                        "min_command_subscribers": ParameterValue(
                            LaunchConfiguration("min_command_subscribers"),
                            value_type=int,
                        ),
                        "command_subscriber_wait_timeout_s": ParameterValue(
                            LaunchConfiguration(
                                "command_subscriber_wait_timeout_s"
                            ),
                            value_type=float,
                        ),
                        "reset_before_start": ParameterValue(
                            LaunchConfiguration("reset_before_start"),
                            value_type=bool,
                        ),
                        "reset_pose": LaunchConfiguration("reset_pose"),
                        "reset_param_nodes": LaunchConfiguration(
                            "reset_param_nodes"
                        ),
                        "reset_services": LaunchConfiguration(
                            "reset_services"
                        ),
                        "reset_service_timeout_s": ParameterValue(
                            LaunchConfiguration("reset_service_timeout_s"),
                            value_type=float,
                        ),
                        "reset_settle_s": ParameterValue(
                            LaunchConfiguration("reset_settle_s"),
                            value_type=float,
                        ),
                        "replay_speed_path_s_per_s": ParameterValue(
                            LaunchConfiguration("replay_speed_path_s_per_s"),
                            value_type=float,
                        ),
                        "use_demo_timing": ParameterValue(
                            LaunchConfiguration("use_demo_timing"),
                            value_type=bool,
                        ),
                        "demo_speed_scale": ParameterValue(
                            LaunchConfiguration("demo_speed_scale"),
                            value_type=float,
                        ),
                        "reverse": ParameterValue(
                            LaunchConfiguration("reverse"), value_type=bool
                        ),
                        "bidirectional": ParameterValue(
                            LaunchConfiguration("bidirectional"),
                            value_type=bool,
                        ),
                        "approach_start_s": ParameterValue(
                            LaunchConfiguration("approach_start_s"),
                            value_type=float,
                        ),
                        "approach_start_max_error_rad": ParameterValue(
                            LaunchConfiguration("approach_start_max_error_rad"),
                            value_type=float,
                        ),
                        "hold_start_s": ParameterValue(
                            LaunchConfiguration("hold_start_s"), value_type=float
                        ),
                        "hold_turnaround_s": ParameterValue(
                            LaunchConfiguration("hold_turnaround_s"),
                            value_type=float,
                        ),
                        "hold_end_s": ParameterValue(
                            LaunchConfiguration("hold_end_s"), value_type=float
                        ),
                        "loop": ParameterValue(
                            LaunchConfiguration("loop"), value_type=bool
                        ),
                        "kp": ParameterValue(
                            LaunchConfiguration("kp"), value_type=float
                        ),
                        "kd": ParameterValue(
                            LaunchConfiguration("kd"), value_type=float
                        ),
                        "torque_ref": ParameterValue(
                            LaunchConfiguration("torque_ref"), value_type=float
                        ),
                        "use_message_velocity": ParameterValue(
                            LaunchConfiguration("use_message_velocity"),
                            value_type=bool,
                        ),
                        "state_timeout_s": ParameterValue(
                            LaunchConfiguration("state_timeout_s"),
                            value_type=float,
                        ),
                        "tracking_error_stop_rad": ParameterValue(
                            LaunchConfiguration("tracking_error_stop_rad"),
                            value_type=float,
                        ),
                        "tracking_error_warn_rad": ParameterValue(
                            LaunchConfiguration("tracking_error_warn_rad"),
                            value_type=float,
                        ),
                        "reference_feedback_gain": ParameterValue(
                            LaunchConfiguration("reference_feedback_gain"),
                            value_type=float,
                        ),
                        "reference_feedback_max_offset_rad": ParameterValue(
                            LaunchConfiguration(
                                "reference_feedback_max_offset_rad"
                            ),
                            value_type=float,
                        ),
                        "output_csv": LaunchConfiguration("output_csv"),
                        "log_decimation": ParameterValue(
                            LaunchConfiguration("log_decimation"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
