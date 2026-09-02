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

import types

import pytest

from robo_orchard_deploy_ros2 import codec
from robo_orchard_deploy_ros2.config import (
    CameraInfoChannel,
    ImageChannel,
    JointStateChannel,
    ObsChannelBase,
)


def test_image_channel_forwards_its_encoding():
    channel = ImageChannel(
        server_input_key="head_depth",
        topic="/head/depth/image_raw",
        encoding="passthrough",
    )

    decoded = codec.decode(channel, types.SimpleNamespace(data="raw"))

    assert decoded.tolist() == [["raw", "passthrough"]]


def test_camera_info_channel_returns_the_projection_matrix():
    channel = CameraInfoChannel(
        server_input_key="head_intrinsic", topic="/head/color/camera_info"
    )
    msg = types.SimpleNamespace(p=list(range(12)))

    decoded = codec.decode(channel, msg)

    assert decoded.shape == (3, 4)
    assert decoded[1].tolist() == [4.0, 5.0, 6.0, 7.0]


def test_joint_state_channel_reorders_to_the_declared_joints():
    channel = JointStateChannel(
        server_input_key="left_arm_state",
        topic="/robot/left/joint_state",
        joint_names=["Joint2_L", "Joint1_L"],
    )
    msg = types.SimpleNamespace(
        name=["Joint1_L", "Joint2_L", "Joint3_L"], position=[0.1, 0.2, 0.3]
    )

    decoded = codec.decode(channel, msg)

    assert decoded.tolist() == [0.2, 0.1]


def test_joint_state_channel_without_names_keeps_the_published_order():
    channel = JointStateChannel(
        server_input_key="left_arm_state", topic="/robot/left/joint_state"
    )
    msg = types.SimpleNamespace(
        name=["Joint1_L", "Joint2_L"], position=[0.1, 0.2]
    )

    decoded = codec.decode(channel, msg)

    assert decoded.tolist() == [0.1, 0.2]


def test_missing_joint_names_the_channel_and_the_topic():
    channel = JointStateChannel(
        server_input_key="left_arm_state",
        topic="/robot/left/joint_state",
        joint_names=["Joint1_L", "gripper_L"],
    )
    msg = types.SimpleNamespace(name=["Joint1_L"], position=[0.1])

    with pytest.raises(KeyError) as error:
        codec.decode(channel, msg)

    assert "gripper_L" in str(error.value)
    assert "left_arm_state" in str(error.value)
    assert "/robot/left/joint_state" in str(error.value)


def test_unknown_channel_kind_is_reported():
    class UnsupportedChannel(ObsChannelBase):
        msg_type: str = "sensor_msgs/msg/Imu"

    channel = UnsupportedChannel(server_input_key="imu", topic="/imu")

    with pytest.raises(TypeError, match="UnsupportedChannel"):
        codec.decode(channel, types.SimpleNamespace())
