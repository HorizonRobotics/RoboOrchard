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

import importlib.util
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
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


@pytest.mark.parametrize(
    ("override", "configured", "expected"),
    [("", 0.7, 0.7), ("0.0", 0.7, 0.0), ("", None, 0.5)],
)
def test_launch_threshold_preserves_yaml_unless_explicitly_overridden(
    tmp_path, override, configured, expected
):
    launch_path = (
        Path(__file__).resolve().parents[1] / "launch/wuji_glove.launch.py"
    )
    spec = importlib.util.spec_from_file_location("glove_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    config = tmp_path / "glove.yaml"
    parameters = (
        " {}"
        if configured is None
        else f"\n    min_joint_confidence: {configured}"
    )
    config.write_text(f"/**:\n  ros__parameters:{parameters}\n")
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "hand_side": "right",
            "hand_model": "wuji_hand",
            "config_file": str(config),
            "min_joint_confidence": override,
        }
    )
    action = module._create_retarget_node(context)[0]
    evaluated = evaluate_parameters(context, action._Node__parameters)
    inline = evaluated[1]
    args = ["--ros-args", "--params-file", str(evaluated[0])]
    if override:
        threshold = inline["min_joint_confidence"]
        assert type(threshold) is float
        args.extend(["-p", f"min_joint_confidence:={threshold}"])
    else:
        assert "min_joint_confidence" not in inline
    rclpy.init(args=args)
    node = WujiGloveRetargetNode(_FakeRetargeter())
    try:
        assert node._min_joint_confidence == expected
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("model", ["wuji_hand", "wuji_hand_2"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_retarget_node_publishes_complete_named_joint_command(model, side):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"hand_model:={model}",
            "-p",
            f"hand_side:={side}",
        ]
    )
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
        if model == "wuji_hand":
            expected_names = [
                f"{side}_finger{finger}_joint{joint}"
                for finger in range(1, 6)
                for joint in range(1, 5)
            ]
        else:
            # Official Hand 2 JointMapping dense-index golden table.
            bodies = [
                "thumb_cmc_flex",
                "thumb_cmc_abd",
                "thumb_mcp",
                "thumb_ip",
                "index_finger_mcp_flex",
                "index_finger_mcp_abd",
                "index_finger_pip",
                "index_finger_dip",
                "middle_finger_mcp_flex",
                "middle_finger_mcp_abd",
                "middle_finger_pip",
                "middle_finger_dip",
                "ring_finger_mcp_flex",
                "ring_finger_mcp_abd",
                "ring_finger_pip",
                "ring_finger_dip",
                "pinky_mcp_flex",
                "pinky_mcp_abd",
                "pinky_pip",
                "pinky_dip",
            ]
            expected_names = [f"{side[0]}_{body}" for body in bodies]
        assert command.name == expected_names
        assert command.position == pytest.approx(
            np.arange(20, dtype=np.float32) / 10.0
        )
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


@pytest.mark.parametrize("model", ["wuji_hand", "wuji_hand_2"])
def test_retarget_node_gates_low_confidence_and_recovers_after_reset(model):
    rclpy.init(args=["--ros-args", "-p", f"hand_model:={model}"])
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
        assert node._hand_model == "wuji_hand"
        assert node._hand_side == "right"
        assert node._min_joint_confidence == 0.5
        node._skeleton_callback(_skeleton(confidence=1.1))

        assert retargeter.step_count == 0
        assert not node._tracking_valid
        assert node._tracking_error == "invalid skeleton confidence values"
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("model", ["wuji_hand", "wuji_hand_2"])
def test_zero_threshold_accepts_zero_confidence_but_rejects_invalid_data(
    model,
):
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"hand_model:={model}",
            "-p",
            "min_joint_confidence:=0.0",
        ]
    )
    retargeter = _FakeRetargeter()
    node = WujiGloveRetargetNode(retargeter)
    try:
        node._skeleton_callback(_skeleton(confidence=0.0))
        assert retargeter.step_count == 1
        assert node._tracking_valid

        for invalid in (-0.1, 1.1, float("nan"), float("inf")):
            node._skeleton_callback(_skeleton(confidence=invalid))
            assert retargeter.step_count == 1
            assert not node._tracking_valid
            assert node._tracking_error == "invalid skeleton confidence values"

        skeleton = _skeleton(confidence=0.0)
        skeleton.joints[0].pose.position.x = float("nan")
        node._skeleton_callback(skeleton)
        assert retargeter.step_count == 1
        assert not node._tracking_valid
        assert node._tracking_error == "invalid skeleton keypoints"

        node._skeleton_callback(SimpleNamespace(joints=[]))
        assert retargeter.step_count == 1
        assert node._tracking_error == "invalid skeleton keypoints"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_retarget_node_rejects_unknown_model():
    rclpy.init(args=["--ros-args", "-p", "hand_model:=unknown"])
    try:
        with pytest.raises(
            ValueError, match="hand_model must be 'wuji_hand' or 'wuji_hand_2'"
        ):
            WujiGloveRetargetNode(_FakeRetargeter())
    finally:
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
