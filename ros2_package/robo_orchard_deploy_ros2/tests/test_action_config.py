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

from robo_orchard_deploy_ros2.config import (
    ControlConfig,
    JointCommandChannel,
)


def _channel(key="left_arm_actions", topic="/left_algo_cmd", **kwargs):
    kwargs.setdefault("joint_names", ["joint1", "joint2"])
    return JointCommandChannel(server_output_key=key, topic=topic, **kwargs)


def test_each_channel_carries_its_own_joint_names():
    config = ControlConfig(
        channels=[
            _channel(joint_names=["Joint1_L", "Joint2_L"]),
            _channel(
                key="right_arm_actions",
                topic="/right_algo_cmd",
                joint_names=["Joint1_R", "Joint2_R"],
            ),
        ]
    )

    assert config.channels[0].joint_names == ["Joint1_L", "Joint2_L"]
    assert config.channels[1].joint_names == ["Joint1_R", "Joint2_R"]


def test_channels_are_parsed_back_by_kind():
    config = ControlConfig(channels=[_channel()])

    reloaded = ControlConfig.model_validate_json(config.model_dump_json())

    assert isinstance(reloaded.channels[0], JointCommandChannel)
    assert reloaded.channels[0].msg_type == "sensor_msgs/msg/JointState"
    assert reloaded.channels[0].velocities is None


def test_velocities_must_match_the_joint_count():
    with pytest.raises(ValidationError, match="2 joints but 3 velocities"):
        _channel(velocities=[0.0, 0.0, 0.0])


def test_efforts_must_match_the_joint_count():
    with pytest.raises(ValidationError, match="2 joints but 1 efforts"):
        _channel(efforts=[0.0])


def test_joint_names_may_not_be_empty():
    with pytest.raises(ValidationError):
        _channel(joint_names=[])


def test_at_least_one_channel_is_required():
    with pytest.raises(ValidationError):
        ControlConfig(channels=[])


def test_control_frequency_keeps_its_default():
    config = ControlConfig(channels=[_channel()])

    assert config.control_frequency == 25.0
