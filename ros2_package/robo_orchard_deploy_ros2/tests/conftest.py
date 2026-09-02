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

"""Stub the ROS runtime so this package can be tested without a ROS install.

pytest imports this file before any test module, so the stubs are in place
by the time a test imports the code under test. Keeping them here rather
than in each test module also keeps every module's view of the stubs
identical, which a per-module stub cannot guarantee once more than one
module stubs the same package.
"""

import sys
import types
from enum import IntEnum

import numpy as np


class FakeNode:
    """Placeholder base class. Tests pass their own node doubles."""


class FakeSubscriber:
    def __init__(self, node, msg_type_class, topic, qos_profile=None):
        self.node = node
        self.msg_type_class = msg_type_class
        self.topic = topic
        self.qos_profile = qos_profile


class FakeApproximateTimeSynchronizer:
    def __init__(self, subscribers, queue_size, slop):
        self.subscribers = subscribers
        self.queue_size = queue_size
        self.slop = slop
        self.callback = None

    def registerCallback(self, callback):  # noqa: N802
        self.callback = callback


class FakeCvBridge:
    def imgmsg_to_cv2(self, msg, desired_encoding):
        return np.array([[msg.data, desired_encoding]], dtype=object)


class FakeJointState:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []


class FakeReliabilityPolicy(IntEnum):
    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2


class FakeDurabilityPolicy(IntEnum):
    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2


class FakeHistoryPolicy(IntEnum):
    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2


class FakeQoSProfile:
    def __init__(self, depth, reliability, durability, history):
        self.depth = depth
        self.reliability = reliability
        self.durability = durability
        self.history = history

    def __eq__(self, other):
        if not isinstance(other, FakeQoSProfile):
            return NotImplemented
        return (
            self.depth == other.depth
            and self.reliability == other.reliability
            and self.durability == other.durability
            and self.history == other.history
        )


def _install_stub_modules():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.DurabilityPolicy = FakeDurabilityPolicy
    rclpy_qos.HistoryPolicy = FakeHistoryPolicy
    rclpy_qos.QoSProfile = FakeQoSProfile
    rclpy_qos.ReliabilityPolicy = FakeReliabilityPolicy
    rclpy_qos.qos_profile_default = FakeQoSProfile(
        depth=10,
        reliability=FakeReliabilityPolicy.RELIABLE,
        durability=FakeDurabilityPolicy.VOLATILE,
        history=FakeHistoryPolicy.KEEP_LAST,
    )
    rclpy_qos.qos_profile_sensor_data = FakeQoSProfile(
        depth=5,
        reliability=FakeReliabilityPolicy.BEST_EFFORT,
        durability=FakeDurabilityPolicy.VOLATILE,
        history=FakeHistoryPolicy.KEEP_LAST,
    )
    rclpy.node = rclpy_node
    rclpy.qos = rclpy_qos
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos

    message_filters = types.ModuleType("message_filters")
    message_filters.Subscriber = FakeSubscriber
    message_filters.ApproximateTimeSynchronizer = (
        FakeApproximateTimeSynchronizer
    )
    sys.modules["message_filters"] = message_filters

    cv_bridge = types.ModuleType("cv_bridge")
    cv_bridge.CvBridge = FakeCvBridge
    sys.modules["cv_bridge"] = cv_bridge

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = FakeJointState
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg


_install_stub_modules()
