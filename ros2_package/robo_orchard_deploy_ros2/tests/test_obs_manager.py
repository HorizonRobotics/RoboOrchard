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
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
    qos_profile_default,
    qos_profile_sensor_data,
)

from robo_orchard_deploy_ros2.config import (
    ControlConfig,
    DeployConfig,
    JointCommandChannel,
    JointStateChannel,
    ObservationConfig,
    QosProfile,
)
from robo_orchard_deploy_ros2.obs_manager import ObservationManager


class _RecordingLogger:
    def __init__(self):
        self.errors = []

    def error(self, message, **kwargs):
        self.errors.append(message)

    def warning(self, message, **kwargs):
        pass

    def info(self, message, **kwargs):
        pass

    def debug(self, message, **kwargs):
        pass


class _FakeTimer:
    def cancel(self):
        pass


class _FakeNode:
    def __init__(self):
        self.logger = _RecordingLogger()

    def get_logger(self):
        return self.logger

    def create_timer(self, period, callback):
        return _FakeTimer()

    def get_topic_names_and_types(self):
        return [("/puppet/joint_left", ["sensor_msgs/msg/JointState"])]


def _joint_state(positions, names=None):
    return types.SimpleNamespace(name=names or [], position=positions)


def _config(joint_names=None, qos_profile=None):
    channel_kwargs = {}
    if qos_profile is not None:
        channel_kwargs["qos_profile"] = qos_profile
    return DeployConfig(
        observation_config=ObservationConfig(
            channels=[
                JointStateChannel(
                    server_input_key="left_arm_state",
                    topic="/puppet/joint_left",
                    joint_names=joint_names,
                    **channel_kwargs,
                )
            ]
        ),
        control_config=ControlConfig(
            channels=[
                JointCommandChannel(
                    server_output_key="left_arm_actions",
                    topic="/left_algo_cmd",
                    joint_names=["joint1"],
                )
            ]
        ),
    )


@pytest.fixture
def node():
    return _FakeNode()


def test_nothing_is_served_before_the_first_frame(node):
    manager = ObservationManager(node, _config())

    assert manager.get_observations() is None


def test_default_qos_matches_the_previous_subscription(node):
    manager = ObservationManager(node, _config())

    manager._attempt_subscriptions()

    assert manager._subscribers[0].qos_profile == qos_profile_default


def test_sensor_data_values_are_forwarded_to_the_subscription(node):
    manager = ObservationManager(
        node,
        _config(
            qos_profile=QosProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
        ),
    )

    manager._attempt_subscriptions()

    assert manager._subscribers[0].qos_profile == qos_profile_sensor_data


def test_a_frame_is_served_once(node):
    manager = ObservationManager(node, _config())

    manager._observe_callback(_joint_state([0.1, 0.2]))

    served = manager.get_observations()
    assert served is not None
    assert list(served["left_arm_state"]) == [0.1, 0.2]
    assert manager.get_observations() is None


def test_every_new_frame_is_served_again(node):
    manager = ObservationManager(node, _config())

    manager._observe_callback(_joint_state([0.1, 0.2]))
    assert manager.get_observations() is not None
    manager._observe_callback(_joint_state([0.3, 0.4]))

    served = manager.get_observations()
    assert served is not None
    assert list(served["left_arm_state"]) == [0.3, 0.4]


def test_an_identical_frame_still_counts_as_new(node):
    """Freshness is per delivered frame, not per observation value.

    A robot holding still publishes unchanging joint positions, and those
    frames must keep reaching the model server.
    """
    manager = ObservationManager(node, _config())

    manager._observe_callback(_joint_state([0.1, 0.2]))
    assert manager.get_observations() is not None
    manager._observe_callback(_joint_state([0.1, 0.2]))

    assert manager.get_observations() is not None


def test_a_dropped_frame_is_never_served(node):
    """A frame that fails to decode must not count as delivered."""
    manager = ObservationManager(node, _config(joint_names=["joint1"]))

    manager._observe_callback(_joint_state([0.1], names=["other_joint"]))

    assert manager.get_observations() is None
    assert any("failed to decode" in msg for msg in node.logger.errors)


def test_a_dropped_frame_does_not_hide_the_next_one(node):
    manager = ObservationManager(node, _config(joint_names=["joint1"]))

    manager._observe_callback(_joint_state([0.1], names=["other_joint"]))
    manager._observe_callback(_joint_state([0.7], names=["joint1"]))

    served = manager.get_observations()
    assert served is not None
    assert list(served["left_arm_state"]) == [0.7]
