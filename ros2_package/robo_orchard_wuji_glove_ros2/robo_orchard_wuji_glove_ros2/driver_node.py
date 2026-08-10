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

from __future__ import annotations
import dataclasses
import math
import threading
import time
from collections.abc import Callable
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from robo_orchard_wuji_glove_msg_ros2.msg import (
    GloveInfo,
    HandJointAngles,
    HandSkeleton,
    PoseArrayWithConfidence,
    TactileFrame,
    TactileZones,
)
from robo_orchard_wuji_glove_ros2.conversions import (
    glove_info_to_message,
    hand_joint_angles_to_message,
    hand_skeleton_to_message,
    imu_to_message,
    point_cloud_to_message,
    poses_to_message,
    tactile_to_message,
    tactile_zones_to_message,
    transforms_to_messages,
)
from robo_orchard_wuji_glove_ros2.sdk_client import (
    SDK_STREAM_SPECS,
    GloveDeviceInfo,
    SdkClientConfig,
    WujiSdkClient,
)

_ROS_STREAM_TYPES = {
    "tactile": (TactileFrame, tactile_to_message),
    "tactile_zones": (TactileZones, tactile_zones_to_message),
    "tactile_binary": (TactileFrame, tactile_to_message),
    "tactile_residual": (TactileFrame, tactile_to_message),
    "emf_poses": (PoseArrayWithConfidence, poses_to_message),
    "tip_poses": (PoseArrayWithConfidence, poses_to_message),
    "hand_joint_angles": (HandJointAngles, hand_joint_angles_to_message),
    "hand_skeleton": (HandSkeleton, hand_skeleton_to_message),
    "tactile_point_cloud": (PointCloud2, point_cloud_to_message),
    "imu_palm": (Imu, imu_to_message),
}


def _require_positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite")


class WujiGloveDriverNode(Node):
    """Connect one Wuji Glove and publish its documented SDK streams."""

    def __init__(self, sdk_client: WujiSdkClient | Any | None = None) -> None:
        super().__init__("wuji_glove_driver")
        self._status_lock = threading.Lock()
        self._shutdown_requested = threading.Event()
        self._connection_wakeup = threading.Event()
        self._reconnect_requested = threading.Event()
        self._connection_ready = threading.Event()
        self._publishing_enabled = threading.Event()
        self._received_frames = 0
        self._discarded_frames = 0
        self._last_error = ""
        self._last_frame_error_log: dict[str, float] = {}
        self._connected_at_monotonic: float | None = None
        self._last_critical_frame_monotonic: float | None = None
        self._config, reconnect_interval = self._load_config()
        self._critical_stream = (
            "hand_skeleton"
            if "hand_skeleton" in self._config.enabled_streams
            else ""
        )
        self._client = sdk_client or WujiSdkClient()
        self._source_namespace = self.get_namespace()
        self._device_info = GloveDeviceInfo(
            serial_number=self._config.serial_number,
            device_name=self._config.device_name,
            source_namespace=self._source_namespace,
            handedness=self._config.hand_side,
        )

        self._stream_publishers: dict[str, Any] = {}
        self._transform_broadcaster: TransformBroadcaster | None = None
        self._static_transform_broadcaster: (
            StaticTransformBroadcaster | None
        ) = None
        self._create_publishers()
        self._publish_device_info()

        # Initialization remains synchronous so no ROS work starts before the
        # first connection attempt and time validation have completed.
        self._try_connect()
        self._connection_worker = threading.Thread(
            target=self._connection_worker_loop,
            name=f"{self.get_name()}-connection",
            daemon=False,
        )
        self._connection_worker.start()
        self._reconnect_timer = self.create_timer(
            reconnect_interval, self._request_reconnect
        )
        self._diagnostics_timer = self.create_timer(
            1.0, self._publish_diagnostics
        )

    def destroy_node(self) -> bool:
        """Complete SDK cleanup before destroying ROS publishers."""
        self._shutdown_requested.set()
        self._connection_ready.clear()
        self._publishing_enabled.clear()
        self._connection_wakeup.set()
        self._connection_worker.join()
        return super().destroy_node()

    def _load_config(self) -> tuple[SdkClientConfig, float]:
        serial_number = str(self.declare_parameter("serial_number", "").value)
        hand_side = str(self.declare_parameter("hand_side", "right").value)
        device_name = str(
            self.declare_parameter("device_name", "wuji_glove_right").value
        )
        frame_prefix = str(
            self.declare_parameter("frame_prefix", "").value
        ).strip()
        sdk_user = str(self.declare_parameter("sdk_user", "default").value)
        hand_model_path = str(
            self.declare_parameter("hand_model_path", "").value
        )
        sdk_log_level = str(
            self.declare_parameter("sdk_log_level", "warn").value
        )
        divider = int(
            self.declare_parameter("emf_poses_rate_divider", 1).value
        )
        reconnect_interval = float(
            self.declare_parameter("reconnect_interval_s", 2.0).value
        )
        self._critical_stream_timeout_s = float(
            self.declare_parameter("critical_stream_timeout_s", 1.0).value
        )
        time_sync_max_age_s = float(
            self.declare_parameter("time_sync_max_age_s", 5.0).value
        )
        time_sync_max_rtt_ms = float(
            self.declare_parameter("time_sync_max_rtt_ms", 100.0).value
        )
        stream_profile = str(
            self.declare_parameter("stream_profile", "configured").value
        ).strip()
        if hand_side not in ("left", "right"):
            raise ValueError("hand_side must be 'left' or 'right'")
        if not device_name or "/" in device_name or "." in device_name:
            raise ValueError(
                "device_name must be non-empty and contain neither '/' nor '.'"
            )
        if not frame_prefix:
            frame_prefix = device_name
        if (
            frame_prefix.startswith("/")
            or frame_prefix.endswith("/")
            or "//" in frame_prefix
            or any(character.isspace() for character in frame_prefix)
        ):
            raise ValueError(
                "frame_prefix must be a relative frame path without empty "
                "segments or whitespace"
            )
        if divider < 1:
            raise ValueError("emf_poses_rate_divider must be at least 1")
        _require_positive_finite("reconnect_interval_s", reconnect_interval)
        _require_positive_finite(
            "critical_stream_timeout_s", self._critical_stream_timeout_s
        )
        _require_positive_finite("time_sync_max_age_s", time_sync_max_age_s)
        _require_positive_finite("time_sync_max_rtt_ms", time_sync_max_rtt_ms)
        if stream_profile not in ("configured", "teleop_minimal"):
            raise ValueError(
                "stream_profile must be 'configured' or 'teleop_minimal'"
            )
        if sdk_log_level not in (
            "off",
            "error",
            "warn",
            "warning",
            "info",
            "debug",
            "trace",
        ):
            raise ValueError("sdk_log_level is invalid")

        enabled_streams = frozenset(
            spec.name
            for spec in SDK_STREAM_SPECS
            if bool(
                self.declare_parameter(
                    spec.parameter_name, spec.enabled_by_default
                ).value
            )
        )
        publish_transforms = bool(
            self.declare_parameter("publish_transforms", True).value
        )
        if stream_profile == "teleop_minimal":
            enabled_streams = frozenset({"hand_skeleton"})
            publish_transforms = False
        config = SdkClientConfig(
            serial_number=serial_number,
            hand_side=hand_side,
            device_name=device_name,
            frame_prefix=frame_prefix,
            sdk_user=sdk_user,
            hand_model_path=hand_model_path,
            sdk_log_level=sdk_log_level,
            emf_poses_rate_divider=divider,
            time_sync_max_age_s=time_sync_max_age_s,
            time_sync_max_rtt_ms=time_sync_max_rtt_ms,
            enabled_streams=enabled_streams,
            publish_transforms=publish_transforms,
        )
        return config, reconnect_interval

    def _create_publishers(self) -> None:
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._info_publisher = self.create_publisher(
            GloveInfo, "device_info", info_qos
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "diagnostics", 10
        )
        for spec in SDK_STREAM_SPECS:
            if spec.name in self._config.enabled_streams:
                message_type, _ = _ROS_STREAM_TYPES[spec.name]
                self._stream_publishers[spec.name] = self.create_publisher(
                    message_type, spec.topic, sensor_qos
                )

        if self._config.publish_transforms:
            self._transform_broadcaster = TransformBroadcaster(self)
            self._static_transform_broadcaster = StaticTransformBroadcaster(
                self
            )

    def _make_callbacks(self) -> dict[str, Callable[[Any], None]]:
        callbacks: dict[str, Callable[[Any], None]] = {}
        for stream_name, publisher in self._stream_publishers.items():
            _, converter = _ROS_STREAM_TYPES[stream_name]
            callbacks[stream_name] = self._message_callback(
                stream_name, publisher, converter
            )
        if self._transform_broadcaster is not None:
            callbacks["transforms"] = self._transform_callback(False)
            callbacks["static_transforms"] = self._transform_callback(True)
        return callbacks

    def _message_callback(
        self,
        stream_name: str,
        publisher: Any,
        converter: Callable[[Any, str], Any],
    ) -> Callable[[Any], None]:
        def callback(frame: Any) -> None:
            if (
                self._shutdown_requested.is_set()
                or not self._connection_ready.is_set()
                or self._reconnect_requested.is_set()
            ):
                return
            is_critical = stream_name == self._critical_stream
            if not is_critical and not self._publishing_enabled.is_set():
                return
            try:
                message = converter(frame, self._config.frame_prefix)
                if is_critical:
                    if (
                        not self._connection_ready.is_set()
                        or self._reconnect_requested.is_set()
                    ):
                        return
                elif not self._publishing_enabled.is_set():
                    return
                publisher.publish(message)
            except Exception as error:
                self._log_frame_error(stream_name, error)
                return
            self._received_frames += 1
            if is_critical:
                self._record_critical_frame()

        return callback

    def _transform_callback(self, static: bool) -> Callable[[Any], None]:
        broadcaster = (
            self._static_transform_broadcaster
            if static
            else self._transform_broadcaster
        )
        stream_name = "static_transforms" if static else "transforms"

        def callback(frame: Any) -> None:
            if (
                self._shutdown_requested.is_set()
                or not self._connection_ready.is_set()
                or not self._publishing_enabled.is_set()
            ):
                return
            try:
                broadcaster.sendTransform(
                    transforms_to_messages(frame, self._config.frame_prefix)
                )
                self._received_frames += 1
            except Exception as error:
                self._log_frame_error(stream_name, error)

        return callback

    def _record_critical_frame(self) -> None:
        now = time.monotonic()
        became_ready = False
        with self._status_lock:
            if (
                not self._connection_ready.is_set()
                or self._reconnect_requested.is_set()
                or not self._device_info.connected
            ):
                return
            self._last_critical_frame_monotonic = now
            if not self._publishing_enabled.is_set():
                self._publishing_enabled.set()
                self._last_error = ""
                became_ready = True
        if became_ready:
            self.get_logger().info(
                f"received first valid {self._critical_stream} frame; ready"
            )

    def _request_reconnect(self) -> None:
        if not self._shutdown_requested.is_set():
            self._connection_wakeup.set()

    def _connection_worker_loop(self) -> None:
        try:
            while True:
                self._connection_wakeup.wait()
                self._connection_wakeup.clear()
                if self._shutdown_requested.is_set():
                    return
                self._try_connect()
        finally:
            self._connection_ready.clear()
            self._publishing_enabled.clear()
            self._client.disconnect()

    def _try_connect(self) -> None:
        if self._shutdown_requested.is_set():
            return
        client_connected = self._client.connected
        if self._shutdown_requested.is_set() or (
            client_connected and not self._reconnect_requested.is_set()
        ):
            return
        self._connection_ready.clear()
        self._publishing_enabled.clear()
        self._reconnect_requested.clear()
        publish_disconnected = False
        with self._status_lock:
            self._connected_at_monotonic = None
            self._last_critical_frame_monotonic = None
            if self._device_info.connected:
                self._device_info = dataclasses.replace(
                    self._device_info, connected=False
                )
                publish_disconnected = True
                if not client_connected and not self._last_error:
                    self._last_error = "Wuji Glove connection lost"
        if publish_disconnected:
            self._publish_device_info()
        try:
            info = self._client.connect(
                self._config,
                self._make_callbacks(),
                self._on_stream_error,
            )
        except Exception as error:
            if self._shutdown_requested.is_set():
                return
            self._reconnect_requested.set()
            with self._status_lock:
                self._last_error = str(error)
                self._device_info = dataclasses.replace(
                    self._device_info, connected=False
                )
            self._publish_device_info()
            self.get_logger().warning(f"failed to connect Wuji Glove: {error}")
            return

        if self._shutdown_requested.is_set():
            return
        info = dataclasses.replace(
            info, source_namespace=self._source_namespace
        )
        client_connected = self._client.connected
        if self._shutdown_requested.is_set():
            return
        with self._status_lock:
            connection_failed = (
                self._reconnect_requested.is_set() or not client_connected
            )
            if connection_failed:
                self._device_info = dataclasses.replace(info, connected=False)
                if not self._last_error:
                    self._last_error = (
                        "Wuji Glove became unhealthy while connecting"
                    )
            else:
                self._device_info = info
                self._last_error = ""
        if connection_failed:
            self._reconnect_requested.set()
            self._publish_device_info()
            return

        self._publish_device_info()
        client_connected = self._client.connected
        publish_disconnected = False
        with self._status_lock:
            connected = (
                not self._shutdown_requested.is_set()
                and not self._reconnect_requested.is_set()
                and client_connected
                and self._device_info.connected
            )
            if connected:
                self._connected_at_monotonic = time.monotonic()
                self._connection_ready.set()
                if not self._critical_stream:
                    self._publishing_enabled.set()
            elif self._device_info.connected:
                self._device_info = dataclasses.replace(
                    self._device_info, connected=False
                )
                if not self._last_error:
                    self._last_error = (
                        "Wuji Glove became unhealthy after initialization"
                    )
                publish_disconnected = True
        if not connected:
            if not self._shutdown_requested.is_set():
                self._reconnect_requested.set()
                if publish_disconnected:
                    self._publish_device_info()
            return
        if self._shutdown_requested.is_set():
            return
        state = "warming_up" if self._critical_stream else "ready"
        self.get_logger().info(
            "connected Wuji Glove: "
            f"side={info.handedness} serial={info.serial_number} "
            f"alias={info.device_name} sdk={info.sdk_version} "
            f"sdk_user={info.sdk_user} "
            f"hand_model={info.hand_model_path or 'sdk-managed'} "
            f"frame_prefix={self._config.frame_prefix} "
            f"state={state}"
        )

    def _on_stream_error(self, message: str) -> None:
        if self._shutdown_requested.is_set():
            return
        self._reconnect_requested.set()
        with self._status_lock:
            self._connection_ready.clear()
            self._publishing_enabled.clear()
            self._last_error = message
            self._device_info = dataclasses.replace(
                self._device_info, connected=False
            )
        self._publish_device_info()
        self.get_logger().warning(f"Wuji Glove stream ended: {message}")

    def _check_critical_stream_watchdog(self) -> None:
        if (
            not self._critical_stream
            or not self._connection_ready.is_set()
            or self._reconnect_requested.is_set()
        ):
            return
        now = time.monotonic()
        tripped = False
        watchdog_error = ""
        with self._status_lock:
            reference = self._last_critical_frame_monotonic
            if reference is None:
                reference = self._connected_at_monotonic
            if (
                reference is not None
                and now - reference > self._critical_stream_timeout_s
            ):
                self._connection_ready.clear()
                self._publishing_enabled.clear()
                self._reconnect_requested.set()
                watchdog_error = (
                    f"no valid {self._critical_stream} frame for "
                    f"{now - reference:.3f}s"
                )
                self._last_error = watchdog_error
                self._device_info = dataclasses.replace(
                    self._device_info, connected=False
                )
                tripped = True
        if tripped:
            self._publish_device_info()
            self.get_logger().warning(
                f"{watchdog_error}; requesting time sync and reconnect"
            )

    def _log_frame_error(self, stream_name: str, error: Exception) -> None:
        self._discarded_frames += 1
        now = time.monotonic()
        last_log = self._last_frame_error_log.get(stream_name, 0.0)
        if now - last_log >= 5.0:
            self._last_frame_error_log[stream_name] = now
            self.get_logger().error(
                f"failed to process {stream_name} frame: {error}"
            )

    def _publish_device_info(self) -> None:
        with self._status_lock:
            info = self._device_info
        self._info_publisher.publish(
            glove_info_to_message(info, self.get_clock().now().to_msg())
        )

    def _publish_diagnostics(self) -> None:
        self._check_critical_stream_watchdog()
        now = time.monotonic()
        with self._status_lock:
            info = self._device_info
            error = self._last_error
            last_critical_frame = self._last_critical_frame_monotonic
        connected = self._client.connected and info.connected
        ready = connected and self._publishing_enabled.is_set()
        status = DiagnosticStatus()
        status.name = f"{self.get_fully_qualified_name()}/connection"
        status.hardware_id = info.serial_number or self._config.serial_number
        if not connected:
            status.level = DiagnosticStatus.ERROR
            status.message = error or "disconnected"
        elif not ready:
            status.level = DiagnosticStatus.WARN
            status.message = f"waiting for valid {self._critical_stream} frame"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ready"
        critical_frame_age = (
            "never"
            if last_critical_frame is None
            else f"{now - last_critical_frame:.3f}"
        )
        status.values = [
            KeyValue(key="handedness", value=self._config.hand_side),
            KeyValue(key="source_namespace", value=info.source_namespace),
            KeyValue(key="sdk_version", value=info.sdk_version),
            KeyValue(key="sdk_user", value=info.sdk_user),
            KeyValue(key="frame_prefix", value=self._config.frame_prefix),
            KeyValue(key="critical_stream", value=self._critical_stream),
            KeyValue(key="critical_frame_age_s", value=critical_frame_age),
            KeyValue(
                key="hand_model_path",
                value=info.hand_model_path or "sdk-managed",
            ),
            KeyValue(key="received_frames", value=str(self._received_frames)),
            KeyValue(
                key="discarded_frames", value=str(self._discarded_frames)
            ),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the Wuji Glove driver node."""
    rclpy.init(args=args)
    node = None
    try:
        node = WujiGloveDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
