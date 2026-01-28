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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    aruco_marker_id_arg = DeclareLaunchArgument(
        "aruco_marker_id",
        description="ID of the ArUco marker to detect",
    )
    aruco_marker_size_arg = DeclareLaunchArgument(
        "aruco_marker_size",
        description="Size of the ArUco marker in meters",
    )
    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame_name",
        description="Camera frame",
    )
    aruco_marker_frame_arg = DeclareLaunchArgument(
        "aruco_marker_frame_name",
        description="Aruco marker frame",
    )
    handeye_calib_config_arg = DeclareLaunchArgument(
        "handeye_calib_config_file",
        description="Path to the hand-eye calibration configuration file",
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        description="Camera info topic",
    )
    camera_raw_topic_arg = DeclareLaunchArgument(
        "camera_raw_topic",
        description="Camera raw image topic",
    )

    aruco_single_node = Node(
        package="aruco_ros",
        namespace="handeye_calib",
        executable="single",
        name="aruco_single_node",
        output="screen",
        remappings=[
            ("/camera_info", LaunchConfiguration("camera_info_topic")),
            ("/image", LaunchConfiguration("camera_raw_topic")),
        ],
        parameters=[
            {
                "marker_size": LaunchConfiguration("aruco_marker_size"),
                "marker_id": LaunchConfiguration("aruco_marker_id"),
                "camera_frame": LaunchConfiguration("camera_frame_name"),
                "marker_frame": LaunchConfiguration("aruco_marker_frame_name"),
            }
        ],
    )
    handeye_calib_node = Node(
        package="robo_orchard_handeye_calib_ros2",
        namespace="handeye_calib",
        executable="robo_orchard_handeye_calib_node",
        name="handeye_calib_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"config_file": LaunchConfiguration("handeye_calib_config_file")}
        ],
    )

    return LaunchDescription(
        [
            aruco_marker_id_arg,
            aruco_marker_size_arg,
            camera_frame_arg,
            aruco_marker_frame_arg,
            handeye_calib_config_arg,
            camera_info_topic_arg,
            camera_raw_topic_arg,
            aruco_single_node,
            handeye_calib_node,
        ]
    )
