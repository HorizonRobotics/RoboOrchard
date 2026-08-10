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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _set_side_defaults(context):
    side = LaunchConfiguration("hand_side").perform(context).strip()
    if side not in ("left", "right"):
        raise ValueError("hand_side must be 'left' or 'right'")

    defaults = {
        "glove_namespace": f"/wuji_glove/{side}",
        "device_name": f"wuji_glove_{side}",
    }
    return [
        SetLaunchConfiguration(name, default)
        for name, default in defaults.items()
        if not LaunchConfiguration(name).perform(context).strip()
    ]


def generate_launch_description():
    package_share = get_package_share_directory("robo_orchard_wuji_glove_ros2")
    default_config = f"{package_share}/config/wuji_glove.yaml"

    arguments = [
        DeclareLaunchArgument(
            "glove_namespace",
            default_value="",
            description=(
                "Namespace for one Wuji Glove and its retargeter. "
                "Empty derives from hand_side."
            ),
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="ROS parameter file for the Wuji Glove nodes.",
        ),
        DeclareLaunchArgument(
            "stream_profile",
            default_value="configured",
            description=(
                "SDK stream profile: configured uses config_file stream "
                "flags; teleop_minimal publishes only hand_skeleton."
            ),
        ),
        DeclareLaunchArgument(
            "glove_serial_number",
            default_value="",
            description="Glove serial number; empty selects by handedness.",
        ),
        DeclareLaunchArgument(
            "hand_side",
            default_value="right",
            description="Glove and target-hand side: left or right.",
        ),
        DeclareLaunchArgument(
            "device_name",
            default_value="",
            description=(
                "Local Wuji SDK device alias. Empty derives from hand_side."
            ),
        ),
        DeclareLaunchArgument(
            "frame_prefix",
            default_value="",
            description=(
                "Prefix for all published frame IDs. Empty derives from "
                "device_name."
            ),
        ),
        DeclareLaunchArgument(
            "sdk_user",
            default_value="default",
            description="Wuji SDK user ID or unique display name.",
        ),
        DeclareLaunchArgument(
            "hand_model_path",
            default_value="",
            description=(
                "Directory containing per-user calibrated hand models. "
                "Empty uses the SDK's configured users directory."
            ),
        ),
        DeclareLaunchArgument(
            "hand_model",
            default_value="wuji_hand",
            description="Retarget model. This integration uses wuji_hand.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="retargeted_joint_commands",
            description="Retargeted sensor_msgs/JointState output topic.",
        ),
    ]

    serial_number = ParameterValue(
        LaunchConfiguration("glove_serial_number"), value_type=str
    )
    hand_side = ParameterValue(
        LaunchConfiguration("hand_side"), value_type=str
    )
    device_name = ParameterValue(
        LaunchConfiguration("device_name"), value_type=str
    )
    frame_prefix = ParameterValue(
        LaunchConfiguration("frame_prefix"), value_type=str
    )
    sdk_user = ParameterValue(LaunchConfiguration("sdk_user"), value_type=str)
    hand_model_path = ParameterValue(
        LaunchConfiguration("hand_model_path"), value_type=str
    )
    hand_model = ParameterValue(
        LaunchConfiguration("hand_model"), value_type=str
    )
    stream_profile = ParameterValue(
        LaunchConfiguration("stream_profile"), value_type=str
    )

    driver = Node(
        package="robo_orchard_wuji_glove_ros2",
        executable="wuji_glove_driver_node",
        name="wuji_glove_driver",
        namespace=LaunchConfiguration("glove_namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "serial_number": serial_number,
                "hand_side": hand_side,
                "device_name": device_name,
                "frame_prefix": frame_prefix,
                "sdk_user": sdk_user,
                "hand_model_path": hand_model_path,
                "stream_profile": stream_profile,
            },
        ],
    )

    retarget = Node(
        package="robo_orchard_wuji_glove_ros2",
        executable="wuji_glove_retarget_node",
        name="wuji_glove_retarget",
        namespace=LaunchConfiguration("glove_namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config_file"),
            {"hand_side": hand_side, "hand_model": hand_model},
        ],
        remappings=[
            (
                "retargeted_joint_commands",
                LaunchConfiguration("command_topic"),
            ),
        ],
    )

    side_defaults = OpaqueFunction(function=_set_side_defaults)
    return LaunchDescription([*arguments, side_defaults, driver, retarget])
