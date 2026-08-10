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

import threading
import time
from types import SimpleNamespace as Namespace

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from robo_orchard_wuji_glove_msg_ros2.msg import HandSkeleton
from robo_orchard_wuji_glove_ros2.driver_node import (
    WujiGloveDriverNode,
    _require_positive_finite,
)
from robo_orchard_wuji_glove_ros2.sdk_client import GloveDeviceInfo


class _FakeSdkClient:
    def __init__(
        self,
        fail_during_connect=False,
        emit_during_connect=False,
        connect_error="",
    ):
        self.connected = False
        self.config = None
        self.callbacks = None
        self.fail_during_connect = fail_during_connect
        self.emit_during_connect = emit_during_connect
        self.connect_error = connect_error
        self.disconnect_calls = 0

    def connect(self, config, callbacks, on_error):
        self.config = config
        self.callbacks = callbacks
        self.on_error = on_error
        if self.connect_error:
            raise RuntimeError(self.connect_error)
        self.connected = True
        info = GloveDeviceInfo(
            serial_number="WG1KA001",
            device_name=config.device_name,
            firmware_version="0.11.0",
            sdk_version="2026.7.21",
            handedness="right",
            ip_address="192.168.1.120",
            data_port=50000,
            connected=True,
        )
        if self.fail_during_connect:
            self.connected = False
            on_error("tactile: closed")
        if self.emit_during_connect:
            callbacks["hand_skeleton"](_skeleton_frame())
        return info

    def disconnect(self):
        self.disconnect_calls += 1
        self.connected = False


class _BlockingReconnectSdkClient(_FakeSdkClient):
    def __init__(self):
        super().__init__()
        self.connect_calls = 0
        self.reconnect_started = threading.Event()
        self.release_reconnect = threading.Event()

    def connect(self, config, callbacks, on_error):
        self.connect_calls += 1
        if self.connect_calls > 1:
            self.reconnect_started.set()
            if not self.release_reconnect.wait(timeout=2.0):
                raise RuntimeError("test reconnect was not released")
        return super().connect(config, callbacks, on_error)


class _DropsAfterInitializationSdkClient(_FakeSdkClient):
    def __init__(self):
        self._connected = False
        self._connected_reads = 0
        super().__init__()

    @property
    def connected(self):
        if not self._connected:
            return False
        self._connected_reads += 1
        return self._connected_reads == 1

    @connected.setter
    def connected(self, value):
        self._connected = value
        if value:
            self._connected_reads = 0


def _pose(index):
    return Namespace(
        position=[index * 0.001, 0.02, 0.03],
        orientation=Namespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _skeleton_frame():
    return Namespace(
        header=Namespace(
            seq=9,
            timestamp_us=1_700_000_000_123_456,
            frame_id="r_wrist",
        ),
        joints=[
            Namespace(
                name=f"joint_{index}",
                pose=_pose(index),
                confidence=0.9,
            )
            for index in range(21)
        ],
    )


def test_driver_publishes_sdk_skeleton_and_expected_topics():
    rclpy.init()
    sdk = _FakeSdkClient()
    node = WujiGloveDriverNode(sdk)
    peer = Node("wuji_glove_driver_test_peer")
    received = []
    diagnostics = []
    subscription = peer.create_subscription(
        HandSkeleton,
        "hand_skeleton",
        received.append,
        qos_profile_sensor_data,
    )
    diagnostics_subscription = peer.create_subscription(
        DiagnosticArray, "diagnostics", diagnostics.append, 10
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    try:
        topics = dict(node.get_topic_names_and_types())
        for topic in (
            "/device_info",
            "/diagnostics",
            "/emf_poses",
            "/hand_joint_angles",
            "/hand_skeleton",
            "/imu/palm",
            "/tactile",
            "/tactile_point_cloud",
            "/tactile_zones",
            "/tf",
            "/tf_static",
            "/tip_poses",
        ):
            assert topic in topics
        assert sdk.config.hand_side == "right"
        assert sdk.config.emf_poses_rate_divider == 1
        assert sdk.config.time_sync_max_age_s == 5.0
        assert sdk.config.time_sync_max_rtt_ms == 100.0
        assert sdk.config.sdk_user == "default"
        assert sdk.config.hand_model_path == ""
        assert sdk.config.frame_prefix == "wuji_glove_right"
        assert node._device_info.source_namespace == "/"
        assert "hand_skeleton" in sdk.config.enabled_streams
        assert "tactile_binary" not in sdk.config.enabled_streams
        assert sdk.disconnect_calls == 0

        frame = _skeleton_frame()
        frame.header.timestamp_us = 0
        sdk.callbacks["hand_skeleton"](frame)
        executor.spin_once(timeout_sec=0.01)
        assert not received
        assert node._discarded_frames == 1

        diagnostics_deadline = time.monotonic() + 2.0
        discarded_count = ""
        while (
            discarded_count != "1" and time.monotonic() < diagnostics_deadline
        ):
            node._publish_diagnostics()
            executor.spin_once(timeout_sec=0.01)
            if diagnostics:
                diagnostic_values = {
                    value.key: value.value
                    for value in diagnostics[-1].status[0].values
                }
                discarded_count = diagnostic_values["discarded_frames"]
        assert discarded_count == "1"
        assert diagnostics[-1].status[0].level == DiagnosticStatus.WARN
        assert (
            diagnostics[-1]
            .status[0]
            .message.startswith("waiting for valid hand_skeleton")
        )

        frame.header.timestamp_us = 1_700_000_000_123_456
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            sdk.callbacks["hand_skeleton"](frame)
            executor.spin_once(timeout_sec=0.01)

        assert received
        assert received[0].header.frame_id == "wuji_glove_right/r_wrist"
        assert received[0].joints[8].name == "joint_8"

        diagnostics_count = len(diagnostics)
        node._publish_diagnostics()
        deadline = time.monotonic() + 2.0
        while (
            len(diagnostics) == diagnostics_count
            and time.monotonic() < deadline
        ):
            executor.spin_once(timeout_sec=0.01)
        assert diagnostics[-1].status[0].level == DiagnosticStatus.OK
        assert diagnostics[-1].status[0].message == "ready"
    finally:
        executor.remove_node(peer)
        executor.remove_node(node)
        peer.destroy_subscription(subscription)
        peer.destroy_subscription(diagnostics_subscription)
        peer.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_driver_drops_callbacks_until_connection_is_ready():
    rclpy.init()
    sdk = _FakeSdkClient(emit_during_connect=True)
    node = WujiGloveDriverNode(sdk)
    try:
        assert node._received_frames == 0
        assert node._connection_ready.is_set()
        assert not node._publishing_enabled.is_set()

        sdk.callbacks["hand_skeleton"](_skeleton_frame())
        assert node._received_frames == 1
        assert node._publishing_enabled.is_set()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_driver_does_not_become_ready_when_first_publish_fails():
    class _FailingPublisher:
        @staticmethod
        def publish(message):
            raise RuntimeError("publisher unavailable")

    rclpy.init()
    sdk = _FakeSdkClient()
    node = WujiGloveDriverNode(sdk)
    try:
        callback = node._message_callback(
            "hand_skeleton",
            _FailingPublisher(),
            lambda frame, prefix: frame,
        )
        callback(_skeleton_frame())

        assert node._discarded_frames == 1
        assert node._last_critical_frame_monotonic is None
        assert not node._publishing_enabled.is_set()
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("value", [0.0, -1.0, float("inf"), float("nan")])
def test_driver_rejects_non_positive_or_non_finite_timeouts(value):
    with pytest.raises(ValueError, match="positive and finite"):
        _require_positive_finite("timeout", value)


def test_driver_preserves_stream_error_raised_during_connect():
    rclpy.init()
    sdk = _FakeSdkClient(fail_during_connect=True)
    node = WujiGloveDriverNode(sdk)
    try:
        assert node._reconnect_requested.is_set()
        assert node._last_error == "tactile: closed"
        assert not node._device_info.connected
        assert sdk.disconnect_calls == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_driver_preserves_actionable_device_selection_error():
    rclpy.init()
    error = (
        "failed to select a right Wuji Glove by handedness; exactly one "
        "matching glove must be discoverable. If multiple matching gloves "
        "are present, set serial_number explicitly."
    )
    sdk = _FakeSdkClient(connect_error=error)
    node = WujiGloveDriverNode(sdk)
    try:
        assert node._reconnect_requested.is_set()
        assert node._last_error == error
        assert not node._device_info.connected
        assert not node._publishing_enabled.is_set()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_driver_reconnects_when_critical_stream_never_becomes_valid():
    rclpy.init()
    sdk = _FakeSdkClient()
    node = WujiGloveDriverNode(sdk)
    try:
        assert node._connection_ready.is_set()
        assert not node._publishing_enabled.is_set()

        node._critical_stream_timeout_s = 0.01
        node._connected_at_monotonic = time.monotonic() - 1.0
        node._check_critical_stream_watchdog()

        assert node._reconnect_requested.is_set()
        assert not node._connection_ready.is_set()
        assert not node._publishing_enabled.is_set()
        assert not node._device_info.connected
        assert "no valid hand_skeleton frame" in node._last_error
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_runtime_reconnect_does_not_block_ros_timers():
    rclpy.init(args=["--ros-args", "-p", "reconnect_interval_s:=0.01"])
    sdk = _BlockingReconnectSdkClient()
    node = WujiGloveDriverNode(sdk)
    timer_calls = []
    timer = node.create_timer(
        0.01, lambda: timer_calls.append(time.monotonic())
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        sdk.connected = False
        node._connection_ready.clear()
        node._reconnect_requested.set()

        reconnect_deadline = time.monotonic() + 1.0
        while (
            not sdk.reconnect_started.is_set()
            and time.monotonic() < reconnect_deadline
        ):
            executor.spin_once(timeout_sec=0.01)
        assert sdk.reconnect_started.is_set()
        assert not node._device_info.connected

        timer_calls_before_block = len(timer_calls)
        progress_deadline = time.monotonic() + 0.2
        while (
            len(timer_calls) < timer_calls_before_block + 3
            and time.monotonic() < progress_deadline
        ):
            executor.spin_once(timeout_sec=0.01)
        assert len(timer_calls) >= timer_calls_before_block + 3
        assert not sdk.release_reconnect.is_set()
    finally:
        sdk.release_reconnect.set()
        node.destroy_timer(timer)
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_destroy_waits_for_blocked_reconnect_cleanup():
    rclpy.init()
    sdk = _BlockingReconnectSdkClient()
    node = WujiGloveDriverNode(sdk)
    destroy_finished = threading.Event()
    destroy_errors = []

    def destroy():
        try:
            node.destroy_node()
        except Exception as error:
            destroy_errors.append(error)
        finally:
            destroy_finished.set()

    destroy_thread = threading.Thread(target=destroy)
    try:
        assert not node._connection_worker.daemon
        sdk.connected = False
        node._reconnect_requested.set()
        node._request_reconnect()
        assert sdk.reconnect_started.wait(timeout=1.0)

        destroy_thread.start()
        assert not destroy_finished.wait(timeout=0.05)
        assert sdk.disconnect_calls == 0

        sdk.release_reconnect.set()
        assert destroy_finished.wait(timeout=1.0)
        destroy_thread.join()
        assert not destroy_errors
        assert not node._connection_worker.is_alive()
        assert sdk.disconnect_calls == 1
    finally:
        sdk.release_reconnect.set()
        if destroy_thread.is_alive():
            destroy_thread.join(timeout=1.0)
        if not destroy_finished.is_set():
            node.destroy_node()
        rclpy.shutdown()


def test_driver_rolls_back_info_when_connection_drops_before_ready():
    rclpy.init()
    sdk = _DropsAfterInitializationSdkClient()
    node = WujiGloveDriverNode(sdk)
    try:
        assert node._reconnect_requested.is_set()
        assert not node._connection_ready.is_set()
        assert not node._publishing_enabled.is_set()
        assert not node._device_info.connected
        assert "became unhealthy after initialization" in node._last_error
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_driver_without_skeleton_does_not_wait_for_skeleton_readiness():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "publish_hand_skeleton:=false",
            "-p",
            "publish_transforms:=false",
        ]
    )
    sdk = _FakeSdkClient()
    node = WujiGloveDriverNode(sdk)
    try:
        assert node._critical_stream == ""
        assert node._connection_ready.is_set()
        assert node._publishing_enabled.is_set()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_teleop_minimal_profile_only_opens_skeleton_stream():
    rclpy.init(args=["--ros-args", "-p", "stream_profile:=teleop_minimal"])
    sdk = _FakeSdkClient()
    node = WujiGloveDriverNode(sdk)
    try:
        assert sdk.config.enabled_streams == frozenset({"hand_skeleton"})
        assert not sdk.config.publish_transforms
        assert set(sdk.callbacks) == {"hand_skeleton"}
    finally:
        node.destroy_node()
        rclpy.shutdown()
