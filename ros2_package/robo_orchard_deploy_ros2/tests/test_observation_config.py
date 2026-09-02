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

import pytest
from pydantic import ValidationError
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from robo_orchard_deploy_ros2.config import (
    CameraInfoChannel,
    ImageChannel,
    JointStateChannel,
    ObservationConfig,
    QosProfile,
)


def _color_channel(key="head_color", topic="/head/color/image_raw"):
    return ImageChannel(server_input_key=key, topic=topic)


def test_channels_keep_their_declared_order():
    config = ObservationConfig(
        channels=[
            _color_channel(),
            JointStateChannel(
                server_input_key="left_arm_state",
                topic="/robot/left/joint_state",
            ),
        ]
    )

    assert [channel.server_input_key for channel in config.channels] == [
        "head_color",
        "left_arm_state",
    ]
    assert config.sync_slop == 0.1
    assert config.sync_queue_size == 1


def test_channels_are_parsed_back_by_kind():
    config = ObservationConfig(
        channels=[
            _color_channel(),
            CameraInfoChannel(
                server_input_key="head_intrinsic",
                topic="/head/color/camera_info",
            ),
            JointStateChannel(
                server_input_key="left_arm_state",
                topic="/robot/left/joint_state",
                joint_names=["Joint1_L"],
                qos_profile=QosProfile(
                    depth=5,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                ),
            ),
        ]
    )

    reloaded = ObservationConfig.model_validate_json(config.model_dump_json())

    assert isinstance(reloaded.channels[0], ImageChannel)
    assert isinstance(reloaded.channels[1], CameraInfoChannel)
    assert isinstance(reloaded.channels[2], JointStateChannel)
    assert reloaded.channels[2].joint_names == ["Joint1_L"]
    assert reloaded.channels[2].qos_profile == QosProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def test_default_message_types_match_the_channel_kind():
    assert _color_channel().msg_type == "sensor_msgs/msg/Image"
    assert (
        CameraInfoChannel(
            server_input_key="head_intrinsic", topic="/head/color/camera_info"
        ).msg_type
        == "sensor_msgs/msg/CameraInfo"
    )
    assert (
        JointStateChannel(
            server_input_key="left_arm_state",
            topic="/robot/left/joint_state",
        ).msg_type
        == "sensor_msgs/msg/JointState"
    )
    assert _color_channel().qos_profile == QosProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def test_at_least_one_channel_is_required():
    with pytest.raises(ValidationError):
        ObservationConfig(channels=[])


def test_duplicated_server_input_key_is_rejected():
    with pytest.raises(ValidationError, match="Duplicated server_input_key"):
        ObservationConfig(
            channels=[
                _color_channel(topic="/left/color/image_raw"),
                _color_channel(topic="/right/color/image_raw"),
            ]
        )


def test_the_same_topic_may_feed_two_keys():
    config = ObservationConfig(
        channels=[
            _color_channel(key="left_color", topic="/head/color/image_raw"),
            _color_channel(key="right_color", topic="/head/color/image_raw"),
        ]
    )

    assert len(config.channels) == 2
