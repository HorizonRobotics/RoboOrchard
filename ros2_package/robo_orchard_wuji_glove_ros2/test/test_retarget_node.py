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

import time

import numpy as np
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from robo_orchard_wuji_glove_msg_ros2.msg import HandSkeleton
from robo_orchard_wuji_glove_ros2.retarget_node import (
    WujiGloveRetargetNode,
)


class _FakeRetargeter:
    def __init__(self):
        self.keypoints = None
        self.step_count = 0
        self.reset_count = 0
        self.fail_reset = False

    def step(self, keypoints):
        self.step_count += 1
        self.keypoints = keypoints.copy()
        return np.arange(20, dtype=np.float32) / 10.0

    def reset(self):
        self.reset_count += 1
        if self.fail_reset:
            raise RuntimeError("reset unavailable")


def _skeleton(confidence=0.9):
    skeleton = HandSkeleton()
    skeleton.header.stamp.sec = 123
    skeleton.header.stamp.nanosec = 456
    skeleton.header.frame_id = "r_wrist"
    for index, joint in enumerate(skeleton.joints):
        joint.pose.position.x = index * 0.001
        joint.pose.position.y = 0.02
        joint.pose.position.z = 0.03
        joint.confidence = confidence
    return skeleton


def test_retarget_node_publishes_complete_named_joint_command():
    rclpy.init()
    retargeter = _FakeRetargeter()
    node = WujiGloveRetargetNode(retargeter)
    peer = Node("wuji_glove_retarget_test_peer")
    publisher = peer.create_publisher(
        HandSkeleton, "hand_skeleton", qos_profile_sensor_data
    )
    received = []
    subscription = peer.create_subscription(
        JointState, "retargeted_joint_commands", received.append, 1
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    try:
        skeleton = _skeleton()

        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            publisher.publish(skeleton)
            executor.spin_once(timeout_sec=0.01)

        assert received
        command = received[0]
        assert command.header.stamp.sec == 123
        assert command.header.stamp.nanosec == 456
        assert command.header.frame_id == ""
        assert len(command.name) == 20
        assert len(command.position) == 20
        assert command.name[0] == "right_finger1_joint1"
        assert command.name[-1] == "right_finger5_joint4"
        assert command.position[-1] == pytest.approx(1.9)
        assert retargeter.keypoints.shape == (21, 3)
    finally:
        executor.remove_node(peer)
        executor.remove_node(node)
        peer.destroy_subscription(subscription)
        peer.destroy_publisher(publisher)
        peer.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_retarget_node_gates_low_confidence_and_recovers_after_reset():
    rclpy.init()
    retargeter = _FakeRetargeter()
    node = WujiGloveRetargetNode(retargeter)
    peer = Node("wuji_glove_tracking_test_peer")
    publisher = peer.create_publisher(
        HandSkeleton, "hand_skeleton", qos_profile_sensor_data
    )
    received = []
    diagnostics = []
    command_subscription = peer.create_subscription(
        JointState, "retargeted_joint_commands", received.append, 1
    )
    diagnostics_subscription = peer.create_subscription(
        DiagnosticArray, "diagnostics", diagnostics.append, 10
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    try:
        low_confidence = _skeleton(confidence=0.0)
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            publisher.publish(low_confidence)
            executor.spin_once(timeout_sec=0.01)
        assert not received

        valid = _skeleton()
        deadline = time.monotonic() + 2.0
        while len(received) < 1 and time.monotonic() < deadline:
            publisher.publish(valid)
            executor.spin_once(timeout_sec=0.01)
        assert len(received) == 1

        publisher.publish(low_confidence)
        deadline = time.monotonic() + 2.0
        while retargeter.reset_count < 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert retargeter.reset_count == 1
        assert len(received) == 1

        node._publish_diagnostics()
        deadline = time.monotonic() + 2.0
        while not diagnostics and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert diagnostics[-1].status[0].level == DiagnosticStatus.WARN
        assert (
            "confidence below threshold" in diagnostics[-1].status[0].message
        )

        deadline = time.monotonic() + 2.0
        while len(received) < 2 and time.monotonic() < deadline:
            publisher.publish(valid)
            executor.spin_once(timeout_sec=0.01)
        assert len(received) == 2
        assert retargeter.reset_count == 1

        node._last_valid_frame_monotonic = time.monotonic() - 1.0
        node._publish_diagnostics()
        assert retargeter.reset_count == 2
        assert not node._tracking_valid
    finally:
        executor.remove_node(peer)
        executor.remove_node(node)
        peer.destroy_subscription(command_subscription)
        peer.destroy_subscription(diagnostics_subscription)
        peer.destroy_publisher(publisher)
        peer.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_retarget_node_rejects_out_of_range_confidence():
    rclpy.init()
    retargeter = _FakeRetargeter()
    node = WujiGloveRetargetNode(retargeter)
    try:
        node._skeleton_callback(_skeleton(confidence=1.1))

        assert retargeter.step_count == 0
        assert not node._tracking_valid
        assert node._tracking_error == "invalid skeleton confidence values"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_retarget_node_requires_successful_reset_before_recovery():
    rclpy.init()
    retargeter = _FakeRetargeter()
    node = WujiGloveRetargetNode(retargeter)
    try:
        node._skeleton_callback(_skeleton())
        assert retargeter.step_count == 1
        assert node._tracking_valid

        retargeter.fail_reset = True
        node._skeleton_callback(_skeleton(confidence=0.0))
        assert retargeter.reset_count == 1
        assert node._reset_required
        assert not node._tracking_valid

        node._skeleton_callback(_skeleton())
        assert retargeter.reset_count == 2
        assert retargeter.step_count == 1
        assert node._reset_required
        assert not node._tracking_valid

        retargeter.fail_reset = False
        node._skeleton_callback(_skeleton())
        assert retargeter.reset_count == 3
        assert retargeter.step_count == 2
        assert not node._reset_required
        assert node._tracking_valid
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_retarget_node_resets_when_command_publish_fails():
    class _FailingPublisher:
        @staticmethod
        def publish(message):
            raise RuntimeError("publisher unavailable")

    rclpy.init()
    retargeter = _FakeRetargeter()
    node = WujiGloveRetargetNode(retargeter)
    node._command_publisher = _FailingPublisher()
    try:
        node._skeleton_callback(_skeleton())

        assert retargeter.step_count == 1
        assert retargeter.reset_count == 1
        assert not node._tracking_valid
        assert node._tracking_error.startswith("command publish failed")
    finally:
        node.destroy_node()
        rclpy.shutdown()
