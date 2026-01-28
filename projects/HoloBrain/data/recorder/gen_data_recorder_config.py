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

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from robo_orchard_data_ros2.mcap.config import (
    FrameRateMonitor,
    QosProfile,
    RecordConfig,
    TopicSpec,
)


def main():
    camera_namespace = "/agilex"

    config = RecordConfig(
        include_patterns=[
            # env center camera
            "/middle_camera/color/image_raw/compressed_data",  # image
            "/middle_camera/aligned_depth_to_color/image_raw/compressed_data",  # depth  # noqa: E501
            f"{camera_namespace}/middle_camera/color/camera_info",  # camera info # noqa: E501
            f"{camera_namespace}/middle_camera/aligned_depth_to_color/camera_info",  # camera info # noqa: E501
            f"{camera_namespace}/middle_camera/extrinsics/depth_to_color",
            # left hand camera
            "/left_camera/color/image_raw/compressed_data",  # image
            "/left_camera/aligned_depth_to_color/image_raw/compressed_data",  # depth # noqa: E501
            f"{camera_namespace}/left_camera/color/camera_info",  # camera info # noqa: E501
            f"{camera_namespace}/left_camera/aligned_depth_to_color/camera_info",  # camera info # noqa: E501
            f"{camera_namespace}/left_camera/extrinsics/depth_to_color",
            # right hand camera
            "/right_camera/color/image_raw/compressed_data",  # image
            "/right_camera/aligned_depth_to_color/image_raw/compressed_data",  # depth # noqa: E501
            f"{camera_namespace}/right_camera/color/camera_info",  # camera info # noqa: E501
            f"{camera_namespace}/right_camera/aligned_depth_to_color/camera_info",  # camera info # noqa: E501
            f"{camera_namespace}/right_camera/extrinsics/depth_to_color",
            # robot left master hand
            "/master/joint_left",
            "/master/status_left",
            # robot left hand
            "/puppet/joint_left",
            "/puppet/end_pose_left",
            "/puppet/status_left",
            # robot right master hand
            "/master/joint_right",
            "/master/status_right",
            # robot right hand
            "/puppet/joint_right",
            "/puppet/end_pose_right",
            "/puppet/status_right",
            # trigger events
            "/robot/left/takeover_muxer/control_mode",
            "/robot/left/takeover_muxer/events",
            "/robot/right/takeover_muxer/control_mode",
            "/robot/right/takeover_muxer/events",
            # others
            "/tf_static",
            "/tf",
            "/diagnostics",
            "/rosout",
            "/parameter_events",
        ],
        default_topic_spec=TopicSpec(
            qos_profile=QosProfile(
                depth=1024,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            stamp_type="msg_header_stamp",
        ),
        topic_spec={
            # env center camera
            "/middle_camera/aligned_depth_to_color/image_raw/compressed_data": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/middle/depth_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            "/middle_camera/color/image_raw/compressed_data": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/middle/color_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            f"{camera_namespace}/middle_camera/color/camera_info": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/middle/color_image/camera_info",
            ),
            f"{camera_namespace}/middle_camera/aligned_depth_to_color/camera_info": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/middle/depth_image/camera_info",
            ),
            f"{camera_namespace}/middle_camera/extrinsics/depth_to_color": TopicSpec(  # noqa: E501
                rename_topic="/observation/cameras/middle/depth_image/tf",
                qos_profile=QosProfile(
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            ),
            # left hand camera
            "/left_camera/aligned_depth_to_color/image_raw/compressed_data": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/left/depth_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            "/left_camera/color/image_raw/compressed_data": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/left/color_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            f"{camera_namespace}/left_camera/color/camera_info": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/left/color_image/camera_info",
            ),
            f"{camera_namespace}/left_camera/aligned_depth_to_color/camera_info": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/left/depth_image/camera_info",
            ),
            f"{camera_namespace}/left_camera/extrinsics/depth_to_color": TopicSpec(  # noqa: E501
                rename_topic="/observation/cameras/left/depth_image/tf",
                qos_profile=QosProfile(
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            ),
            # right hand camera
            "/right_camera/aligned_depth_to_color/image_raw/compressed_data": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/right/depth_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            "/right_camera/color/image_raw/compressed_data": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/right/color_image/image_raw",
                frame_rate_monitor=FrameRateMonitor(min_hz=25),
            ),
            f"{camera_namespace}/right_camera/color/camera_info": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/right/color_image/camera_info",
            ),
            f"{camera_namespace}/right_camera/aligned_depth_to_color/camera_info": TopicSpec(  # noqa: E501
                stamp_type="msg_header_stamp",
                rename_topic="/observation/cameras/right/depth_image/camera_info",
            ),
            f"{camera_namespace}/right_camera/extrinsics/depth_to_color": TopicSpec(  # noqa: E501
                rename_topic="/observation/cameras/right/depth_image/tf",
                qos_profile=QosProfile(
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            ),
            # robot left master hand
            "/master/joint_left": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/left_master/joint",
            ),
            "/master/status_left": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/left_master/status",
            ),
            # robot left hand
            "/puppet/joint_left": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/left/joint",
            ),
            "/puppet/end_pose_left": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/left/end_pose",
            ),
            "/puppet/status_left": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/left/status",
            ),
            # robot right master hand
            "/master/joint_right": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/right_master/joint",
            ),
            "/master/status_right": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/right_master/status",
            ),
            # robot right hand
            "/puppet/joint_right": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/right/joint",
            ),
            "/puppet/end_pose_right": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/right/end_pose",
            ),
            "/puppet/status_right": TopicSpec(
                stamp_type="msg_header_stamp",
                rename_topic="/observation/robot_state/right/status",
            ),
            "/tf_static": TopicSpec(
                qos_profile=QosProfile(
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                )
            ),
        },
        wait_for_topics=set(
            (
                # wait for all camera message to be ok
                "/middle_camera/color/image_raw/compressed_data",
                "/middle_camera/aligned_depth_to_color/image_raw/compressed_data",
                "/right_camera/color/image_raw/compressed_data",
                "/right_camera/aligned_depth_to_color/image_raw/compressed_data",
                "/left_camera/color/image_raw/compressed_data",
                "/left_camera/aligned_depth_to_color/image_raw/compressed_data",
                "/puppet/joint_left",
                "/puppet/end_pose_left",
                "/puppet/joint_right",
                "/puppet/end_pose_right",
            )
        ),
        static_topics=[
            f"{camera_namespace}/middle_camera/extrinsics/depth_to_color",
            f"{camera_namespace}/left_camera/extrinsics/depth_to_color",
            f"{camera_namespace}/right_camera/extrinsics/depth_to_color",
            "/tf_static",
        ],
        max_timestamp_difference_ns=0.5 * 1e9,  # 1s
    )

    with open(
        os.path.join(os.path.dirname(__file__), "data_recorder.json"), "w"
    ) as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
