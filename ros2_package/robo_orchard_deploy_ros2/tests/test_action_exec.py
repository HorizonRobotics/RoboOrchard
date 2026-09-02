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

import numpy as np
import pytest

from robo_orchard_deploy_ros2.action_exec import ActionExecutor
from robo_orchard_deploy_ros2.config import (
    ControlConfig,
    DeployConfig,
    ImageChannel,
    JointCommandChannel,
    ObservationConfig,
)


class _RecordingPublisher:
    def __init__(self, topic):
        self.topic = topic
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _RecordingLogger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def warning(self, message, **kwargs):
        self.warnings.append(message)

    def error(self, message, **kwargs):
        self.errors.append(message)

    def info(self, message, **kwargs):
        pass


class _FakeNode:
    def __init__(self):
        self.logger = _RecordingLogger()
        self.publishers_by_topic = {}

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: "stamp")
        )

    def create_publisher(self, msg_type_class, topic, depth):
        publisher = _RecordingPublisher(topic)
        self.publishers_by_topic[topic] = publisher
        return publisher


def _deploy_config(channels):
    return DeployConfig(
        observation_config=ObservationConfig(
            channels=[
                ImageChannel(server_input_key="color", topic="/color"),
            ]
        ),
        control_config=ControlConfig(channels=channels),
    )


def _dual_arm_channels():
    return [
        JointCommandChannel(
            server_output_key="left_arm_actions",
            topic="/left_algo_cmd",
            joint_names=["Joint1_L", "Joint2_L"],
            velocities=[0.0, 50.0],
            efforts=[0.0, 0.5],
        ),
        JointCommandChannel(
            server_output_key="right_arm_actions",
            topic="/right_algo_cmd",
            joint_names=["Joint1_R", "Joint2_R"],
        ),
    ]


@pytest.fixture
def node():
    return _FakeNode()


def test_each_arm_is_published_with_its_own_joint_names(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    executor.send_action(
        {
            "left_arm_actions": [[0.1, 0.2], [0.3, 0.4]],
            "right_arm_actions": [[1.1, 1.2], [1.3, 1.4]],
        },
        action_index=1,
    )

    left = node.publishers_by_topic["/left_algo_cmd"].published[-1]
    right = node.publishers_by_topic["/right_algo_cmd"].published[-1]
    assert left.name == ["Joint1_L", "Joint2_L"]
    assert left.position == [0.3, 0.4]
    assert right.name == ["Joint1_R", "Joint2_R"]
    assert right.position == [1.3, 1.4]


def test_velocities_and_efforts_are_optional(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    executor.send_action(
        {
            "left_arm_actions": [[0.1, 0.2]],
            "right_arm_actions": [[1.1, 1.2]],
        },
        action_index=0,
    )

    left = node.publishers_by_topic["/left_algo_cmd"].published[-1]
    right = node.publishers_by_topic["/right_algo_cmd"].published[-1]
    assert left.velocity == [0.0, 50.0]
    assert left.effort == [0.0, 0.5]
    assert right.velocity == []
    assert right.effort == []


def test_a_channel_missing_from_the_response_is_skipped(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    executor.send_action({"left_arm_actions": [[0.1, 0.2]]}, action_index=0)

    assert node.publishers_by_topic["/left_algo_cmd"].published
    assert not node.publishers_by_topic["/right_algo_cmd"].published
    assert any(
        "right_arm_actions" in message for message in node.logger.warnings
    )


def test_a_wrong_joint_count_is_not_published(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    executor.send_action(
        {
            "left_arm_actions": [[0.1, 0.2, 0.3]],
            "right_arm_actions": [[1.1, 1.2]],
        },
        action_index=0,
    )

    assert not node.publishers_by_topic["/left_algo_cmd"].published
    assert node.publishers_by_topic["/right_algo_cmd"].published
    assert any("expects 2 joints" in msg for msg in node.logger.errors)


def test_an_out_of_range_step_is_not_published(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    executor.send_action(
        {
            "left_arm_actions": [[0.1, 0.2]],
            "right_arm_actions": [[1.1, 1.2]],
        },
        action_index=5,
    )

    assert not node.publishers_by_topic["/left_algo_cmd"].published
    assert any("cannot send step 5" in msg for msg in node.logger.errors)


def test_a_none_action_step_is_not_published(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    executor.send_action(
        {
            "left_arm_actions": [None],
            "right_arm_actions": [[1.1, 1.2]],
        },
        action_index=0,
    )

    assert not node.publishers_by_topic["/left_algo_cmd"].published
    assert any(
        "no joint position" in message for message in node.logger.warnings
    )


def test_a_hand_channel_needs_no_extra_code(node):
    """A 20 dof hand is just one more channel."""
    channels = _dual_arm_channels() + [
        JointCommandChannel(
            server_output_key="right_hand_actions",
            topic="/right_hand_algo_cmd",
            joint_names=[f"right_finger_joint{i}" for i in range(20)],
        )
    ]
    executor = ActionExecutor(node, _deploy_config(channels))

    executor.send_action(
        {
            "left_arm_actions": [[0.1, 0.2]],
            "right_arm_actions": [[1.1, 1.2]],
            "right_hand_actions": [[float(i) for i in range(20)]],
        },
        action_index=0,
    )

    hand = node.publishers_by_topic["/right_hand_algo_cmd"].published[-1]
    assert len(hand.name) == 20
    assert len(hand.position) == 20


def test_the_step_count_is_the_length_of_the_action_arrays(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    count = executor.action_step_count(
        {
            "left_arm_actions": [[0.1, 0.2], [0.3, 0.4], [0.5, 0.6]],
            "right_arm_actions": [[1.1, 1.2], [1.3, 1.4], [1.5, 1.6]],
        }
    )

    assert count == 3


def test_channels_of_unequal_length_execute_nothing(node):
    """A step only some arms can reach must not be published."""
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    count = executor.action_step_count(
        {
            "left_arm_actions": [[0.1, 0.2], [0.3, 0.4]],
            "right_arm_actions": [[1.1, 1.2]],
        }
    )

    assert count == 0
    assert any("different step counts" in msg for msg in node.logger.errors)


def test_a_missing_channel_makes_the_step_count_zero(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    assert executor.action_step_count({"left_arm_actions": [[0.1, 0.2]]}) == 0


def test_an_empty_response_has_no_steps(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    assert executor.action_step_count({}) == 0
    assert node.logger.errors == []


def _channels_with_remaining():
    return [
        JointCommandChannel(
            server_output_key=f"{side}_arm_actions",
            server_remaining_key=f"{side}_arm_remaining_actions",
            topic=f"/{side}_algo_cmd",
            joint_names=["Joint1", "Joint2"],
        )
        for side in ("left", "right")
    ]


def _three_steps():
    return {
        "left_arm_actions": [[0.5, 1.0], [1.5, 2.0], [2.5, 3.0]],
        "right_arm_actions": [[10.5, 11.0], [11.5, 12.0], [12.5, 13.0]],
    }


def test_each_channel_sends_its_own_remaining_rows(node):
    """The server reassembles them, so no column order is decided here."""
    executor = ActionExecutor(node, _deploy_config(_channels_with_remaining()))

    remaining = executor.remaining_actions(_three_steps(), after_index=0)

    assert sorted(remaining) == [
        "left_arm_remaining_actions",
        "right_arm_remaining_actions",
    ]
    assert remaining["left_arm_remaining_actions"].tolist() == [
        [1.5, 2.0],
        [2.5, 3.0],
    ]
    assert remaining["right_arm_remaining_actions"].tolist() == [
        [11.5, 12.0],
        [12.5, 13.0],
    ]


def test_remaining_rows_are_arrays_the_request_can_carry(node):
    """_pack_request_data only packs ndarrays, so lists would vanish."""
    executor = ActionExecutor(node, _deploy_config(_channels_with_remaining()))

    remaining = executor.remaining_actions(_three_steps(), after_index=0)

    assert isinstance(remaining["left_arm_remaining_actions"], np.ndarray)


def test_a_channel_without_a_remaining_key_sends_nothing(node):
    executor = ActionExecutor(node, _deploy_config(_dual_arm_channels()))

    assert executor.remaining_actions(_three_steps(), after_index=0) == {}


def test_the_last_step_leaves_nothing_remaining(node):
    executor = ActionExecutor(node, _deploy_config(_channels_with_remaining()))

    assert executor.remaining_actions(_three_steps(), after_index=2) == {}
