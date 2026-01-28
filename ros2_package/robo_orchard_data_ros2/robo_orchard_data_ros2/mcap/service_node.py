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

import functools
import os
from datetime import datetime

import rclpy
import rosbag2_py
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node, ParameterDescriptor
from rclpy.qos import QoSProfile
from rclpy.serialization import serialize_message
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from robo_orchard_data_msg_ros2.srv import StartRecording
from robo_orchard_data_ros2.mcap.config import RecordConfig, TopicSpec
from robo_orchard_data_ros2.mcap.utils import (
    FrameRateMonitor,
    TopicFilter,
)

__all__ = ["ServiceMcapRecorder"]


class ServiceMcapRecorder(Node):
    """A ROS 2 node for recording topics into MCAP format via Service control."""  # noqa: E501

    RECORDING_FILE = "__RECORDING__"

    def __init__(self):
        super().__init__("mcap_recorder_service")

        # --- State Management ---
        self._has_started_writing = False

        self.writer = None
        self.uri = None
        self.recording_flag = None

        # Session wait list
        self._session_wait_topics = set()

        # --- Discovery & Data ---
        self._active_topic_metadata = {}
        self._initialize_config()

        self._insepct_topics = set()
        self._subscribers = dict()
        self._cnt = 0
        self._hint_freq = 4096

        self._callback_group = ReentrantCallbackGroup()

        self._min_timestamp = None
        self._max_timestamp = None
        self._frame_rate_monitors = dict()
        self._msg_cnt = dict()

        # Static Latching
        self._latched_msgs = {}

        # --- Service Interfaces ---
        self._srv_start = self.create_service(
            StartRecording,
            "~/start_recording",
            self._handle_start_request,
            callback_group=self._callback_group,
        )
        self._srv_stop = self.create_service(
            Trigger,
            "~/stop_recording",
            self._handle_stop_request,
            callback_group=self._callback_group,
        )

        # --- Startup ---
        if self.config.no_discovery:
            self.scan_topics()
        else:
            self.create_timer(
                1.0, self.scan_topics, callback_group=self._callback_group
            )

        self.create_timer(
            1.0, self._monitor, callback_group=self._callback_group
        )
        self.get_logger().info("ServiceMcapRecorder Ready.")

    @property
    def is_recording(self) -> bool:
        return self.writer is not None

    def destroy_node(self):
        """Clean up resources on node shutdown."""
        self.get_logger().info("Shutting down recorder...")
        self._cleanup_writer()
        super().destroy_node()

    def _handle_start_request(self, request, response):
        """Handles the start recording request."""
        if self.is_recording:
            response.success = False
            response.message = "Already recording!"
            return response

        try:
            # 1. Resolve Path
            dest = request.destination
            if not dest:
                dest = datetime.now().strftime("rosbag2_%Y_%m_%d-%H_%M_%S")
            if not os.path.isabs(dest):
                dest = os.path.abspath(dest)
            self.uri = dest

            # 2. Initialize Writer
            self.recording_flag = os.path.join(self.uri, self.RECORDING_FILE)
            self.writer = rosbag2_py.SequentialWriter()
            self.writer.open(
                rosbag2_py.StorageOptions(
                    uri=self.uri,
                    storage_id="mcap",
                    max_cache_size=self.config.max_cache_size,
                ),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr",
                ),
            )

            # 3. Register Existing Metadata
            for metadata in self._active_topic_metadata.values():
                self.writer.create_topic(metadata)
                self._msg_cnt[metadata.name] = 0

            # 4. Reset Session Stats
            self._cnt = 0
            self._min_timestamp = None
            self._max_timestamp = None

            for _, data in self._frame_rate_monitors.items():
                data["monitor"] = FrameRateMonitor(
                    window_size=data["spec"].frame_rate_monitor.window_size
                )

            # 5. Initialize Logic Flags
            self._session_wait_topics = (
                set(self.config.wait_for_topics)
                if self.config.wait_for_topics
                else set()
            )
            self._has_started_writing = False

            self.get_logger().info(
                f"Session initialized. Waiting for topics: {self._session_wait_topics}"  # noqa: E501
            )

            response.success = True
            response.message = f"Session initialized at {self.uri}"

        except Exception as e:
            self.get_logger().error(f"Start failed: {e}")
            self._cleanup_writer()
            response.success = False
            response.message = str(e)

        return response

    def _handle_stop_request(self, request, response):
        """Handles the stop recording request."""
        if not self.is_recording:
            response.success = False
            response.message = "Not recording."
            return response

        self._cleanup_writer()
        response.success = True
        response.message = f"Stopped. Saved to {self.uri}"
        return response

    def _cleanup_writer(self):
        """Safely closes the writer and cleans up."""
        # Set flag first to stop data flow
        self._has_started_writing = False

        if self.writer:
            del self.writer
            self.writer = None

        if self.recording_flag and os.path.exists(self.recording_flag):
            try:
                os.remove(self.recording_flag)
            except OSError:
                pass

        self.get_logger().info("Recorded {} message".format(self._cnt))
        if self.duration == 0:
            msg = (
                "Empty MCAP file detected. Currently wait for topics: "
                f"{self._session_wait_topics}\n"
                "Possible causes:\n"
                "1. No matching topics subscribed (current filter: include = "
                f"{self.config.include_patterns} || "
                f"exclude = {self.config.exclude_patterns})\n"
                "2. Publisher nodes terminated before recording started\n"
                "3. QoS incompatibility preventing message reception\n"
                "Diagnostic actions:\n"
                "1. Verify topic availability: ros2 topic list\n"
                "2. Check topic spec compatibility: ros2 topic info\n"
            )
            self.get_logger().warn(msg)
        else:
            duration = self.duration * 1e-9
            for topic, msg_cnt in self._msg_cnt.items():
                self.get_logger().info(
                    "topic {}: count = {}, frame rate = {}".format(
                        topic,
                        msg_cnt,
                        msg_cnt / duration,
                    )
                )

    @functools.lru_cache(maxsize=512)  # noqa: B019
    def log_once(self, msg: str, level: str = "info"):
        if level == "info":
            self.get_logger().info(msg)
        elif level == "error":
            self.get_logger().error(msg)

    def _flush_latched_msgs(self):
        """Flushes all latched static messages."""
        if not self.writer or not self._latched_msgs:
            return
        for src_topic, (msgs, dst_topic, spec) in self._latched_msgs.items():
            for msg in msgs:
                self._write_message_internal(msg, src_topic, dst_topic, spec)

    def _write_message_internal(
        self, msg, src_topic: str, dst_topic: str, spec: TopicSpec
    ):
        """Low-level write function."""

        # Timestamp logic
        if (
            spec.stamp_type == "msg_header_stamp"
            and hasattr(msg, "header")
            and isinstance(msg.header, Header)
        ):
            timestamp = int(
                msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            )
        else:
            timestamp = self.get_clock().now().nanoseconds

        # Timestamp Validation
        if self.config.max_timestamp_difference_ns is not None:
            if self._min_timestamp is None:
                self._min_timestamp = timestamp
            else:
                if (
                    timestamp
                    < self._min_timestamp
                    - self.config.max_timestamp_difference_ns
                ):
                    return
                self._min_timestamp = min(timestamp, self._min_timestamp)

            if self._max_timestamp is None:
                self._max_timestamp = timestamp
            else:
                if (
                    self._max_timestamp
                    + self.config.max_timestamp_difference_ns
                    < timestamp
                ):
                    return
                self._max_timestamp = max(timestamp, self._max_timestamp)

        # Perform Write (Protected by try-except for race conditions on Stop)
        if self.is_recording:
            try:
                self.writer.write(dst_topic, serialize_message(msg), timestamp)
            except Exception:
                # This might happen if writer is deleted while writing
                self.get_logger().error("Get error while writing: {e}")

            self._cnt += 1

            if dst_topic in self._frame_rate_monitors:
                self._frame_rate_monitors[dst_topic]["monitor"].update(
                    timestamp
                )
            if dst_topic in self._msg_cnt:
                self._msg_cnt[dst_topic] += 1

            if self._cnt % self._hint_freq == 0:
                self.get_logger().info(f"Recording {self._cnt}-th message")

    def _message_callback(
        self, msg, src_topic: str, dst_topic: str, spec: TopicSpec
    ):
        # 1. Static Latching
        if src_topic in self.config.static_topics:
            if src_topic in self._latched_msgs:
                self._latched_msgs[src_topic][0].append(msg)
            else:
                self._latched_msgs[src_topic] = ([msg], dst_topic, spec)

        # 2. Recording Gate
        if not self.is_recording:
            return

        # 3. Wait-For-Topics Logic
        if self._session_wait_topics:
            if src_topic in self._session_wait_topics:
                self._session_wait_topics.remove(src_topic)
                self.get_logger().info(
                    f"Topic found: {src_topic}. Remaining: {self._session_wait_topics}"  # noqa: E501
                )

            if self._session_wait_topics:
                return

        # 4. Lazy Initialization (First Write Trigger)
        if not self._has_started_writing:
            self.get_logger().info("Beginning writing...")

            if self.recording_flag:
                with open(self.recording_flag, "w"):
                    pass

            self._flush_latched_msgs()
            self._has_started_writing = True

        # 5. Normal Write
        self._write_message_internal(msg, src_topic, dst_topic, spec)

    def _initialize_config(self):
        self.declare_parameter(
            "config_file",
            "",
            descriptor=ParameterDescriptor(
                description="Path to JSON config file"
            ),
        )
        config_file = (
            self.get_parameter("config_file")
            .get_parameter_value()
            .string_value
        )

        if not os.path.exists(config_file):
            raise FileNotFoundError(
                f"Config file {config_file} does not exist!"
            )

        with open(config_file, "r") as handle:
            self.config = RecordConfig.model_validate_json(handle.read())

        self.topic_filter = TopicFilter(
            include_patterns=self.config.include_patterns,
            exclude_patterns=self.config.exclude_patterns,
        )

    def get_topic_spec(self, topic: str) -> TopicSpec:
        return self.config.topic_spec.get(
            topic, self.config.default_topic_spec
        )

    def scan_topics(self):
        def _get_message_class(msg_type: str):
            try:
                module_name, class_name = msg_type.rsplit("/", 1)
                module_name = ".".join(module_name.split("/"))
                module = __import__(module_name, fromlist=[class_name])
                return getattr(module, class_name)
            except Exception as e:
                self.get_logger().error(f"Failed to import {msg_type}: {e}")
                return None

        for topic, msg_types in self.get_topic_names_and_types():
            if topic in self._insepct_topics:
                continue
            self._insepct_topics.add(topic)

            if not self.topic_filter(topic):
                continue

            msg_type = msg_types[0]
            msg_type_class = _get_message_class(msg_type)

            if not msg_type_class:
                self.get_logger().error(
                    f"Ignore topic {topic} due to import error"
                )
                continue

            spec = self.get_topic_spec(topic)
            dst_topic = (
                topic if spec.rename_topic is None else spec.rename_topic
            )

            qos = QoSProfile(
                depth=spec.qos_profile.depth,
                reliability=spec.qos_profile.reliability,
                durability=spec.qos_profile.durability,
                history=spec.qos_profile.history,
            )

            self._subscribers[topic] = self.create_subscription(
                msg_type_class,
                topic,
                functools.partial(
                    self._message_callback,
                    src_topic=topic,
                    dst_topic=dst_topic,
                    spec=spec,
                ),
                qos,
                callback_group=self._callback_group,
            )

            meta = rosbag2_py.TopicMetadata(
                name=dst_topic, type=msg_type, serialization_format="cdr"
            )
            self._active_topic_metadata[topic] = meta

            # If recording is active, register topic to the current writer
            if self.writer is not None:
                self.writer.create_topic(meta)

            if spec.frame_rate_monitor:
                self._frame_rate_monitors[dst_topic] = {
                    "monitor": FrameRateMonitor(
                        window_size=spec.frame_rate_monitor.window_size
                    ),
                    "spec": spec,
                }
            self._msg_cnt[dst_topic] = 0

            log_msg = f"Subscribed to {topic}"
            if spec.rename_topic:
                log_msg += f" as {spec.rename_topic}"
            self.get_logger().info(log_msg)

    def _monitor(self):
        if not self.is_recording:
            return

        if self._session_wait_topics:
            self.get_logger().warning(
                f"Waiting for topics: {self._session_wait_topics}"
            )

        current_ts = (
            self._max_timestamp
            if self._max_timestamp
            else self.get_clock().now().nanoseconds
        )

        for dst_topic, data in self._frame_rate_monitors.items():
            monitor = data["monitor"]

            if self.duration < 5 * 1e9:
                continue
            if len(monitor) < 2:
                continue

            spec = data["spec"]
            current_fps = monitor.get_fps(current_ts)
            min_fps = spec.frame_rate_monitor.min_hz
            max_fps = spec.frame_rate_monitor.max_hz

            if not (min_fps <= current_fps <= max_fps):
                self.get_logger().warning(
                    f"Topic [{dst_topic}] abnormal frame rate: {current_fps:.2f} Hz "  # noqa: E501
                    f"(expected range [{min_fps}, {max_fps}])"
                )

    @property
    def duration(self) -> int:
        if self._max_timestamp is None or self._min_timestamp is None:
            return 0
        return self._max_timestamp - self._min_timestamp


def main(args=None):
    rclpy.init(args=args)
    node = ServiceMcapRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
