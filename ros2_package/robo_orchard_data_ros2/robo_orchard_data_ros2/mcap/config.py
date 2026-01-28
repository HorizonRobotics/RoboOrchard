# Project RoboOrchard
#
# Copyright (c) 2024 Horizon Robotics. All Rights Reserved.
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

from typing import List, Literal, Mapping, Optional, Set

from pydantic import BaseModel, Field
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

__all__ = [
    "QosProfile",
    "FrameRateMonitor",
    "TopicSpec",
    "RecordConfig",
]


class QosProfile(BaseModel):
    """Quality of Service (QoS) profile configuration for ROS 2 topics.

    This class defines the QoS settings for a ROS 2 topic, including
    parameters such as message queue depth, reliability, and durability.

    Attributes:
        depth (int): The size of the message queue. Defaults to 10.
        reliability (int): The reliability policy for message delivery.
            Corresponds to `rclpy.qos.ReliabilityPolicy`.
            Defaults to rclpy.qos.ReliabilityPolicy.RELIABLE.
        durability (int): The durability policy defining whether old
            messages should be stored for late subscribers. Corresponds
            to `rclpy.qos.DurabilityPolicy`.
            Defaults to rclpy.qos.DurabilityPolicy.VOLATILE.
        history (int): The history policy determining how many past messages
            are kept in the queue. Corresponds to `rclpy.qos.HistoryPolicy`.
            Defaults to rclpy.qos.HistoryPolicy.KEEP_LAST, meaning only the
            last `depth` messages are retained.
    """

    depth: int = 10
    reliability: int = ReliabilityPolicy.RELIABLE
    durability: int = DurabilityPolicy.VOLATILE
    history: int = HistoryPolicy.KEEP_LAST


class FrameRateMonitor(BaseModel):
    """Frame rate analysis tool with configurable validation thresholds.

    Implements sliding window statistics to balance measurement accuracy with
    responsiveness, following real-time system monitoring best
    practices[1](@ref).

    Attributes:
        min_hz (float): Minimum acceptable frame rate (Hz). Values below
            this threshold trigger operational warnings.
            Default=0 (no minimum).
        max_hz (float): Maximum permitted frame rate (Hz). Values above
            indicate potential sensor/data overflow. Default=10⁹ (practically
            unlimited).
        window_size (int): Number of consecutive timestamps used for
            rate calculation. Larger values smooth outliers but increase
            latency in detecting rate changes. Default=10000 (the same sa
            ros2 topic hz tools).
    """

    min_hz: float = 0
    max_hz: float = 10e9
    window_size: int = 10000


class TopicSpec(BaseModel):
    """Specification of a ROS 2 topic for recording.

    This class defines how a specific topic should be recorded,
    including the timestamp source and QoS settings.

    Attributes:
        stamp_type (Literal["recorder_clock", "msg_header_stamp"]): The
            source of timestamps for recorded messages, `"recorder_clock"`
            means timestamps from the recorder process, `"msg_header_stamp"`
            means timestamps from the message header timestamp.
            Defaults to `"recorder_clock"`.
        qos_profile (QosProfile): The QoS profile associated with this
            topic. Defaults to `QosProfile()`.
        rename_topic (str | None): Rename to target topic. Defaults to `None`.
        frame_rate_monitor (FrameRateMonitor): The frame rate monitor.
            Defaults to `None`.
    """

    stamp_type: Literal["recorder_clock", "msg_header_stamp"] = (
        "recorder_clock"
    )
    qos_profile: QosProfile = QosProfile()
    rename_topic: str | None = None
    frame_rate_monitor: FrameRateMonitor | None = None


class RecordConfig(BaseModel):
    """Configuration for recording ROS 2 topics.

    This class defines which topics should be recorded, QoS settings,
    filtering options, and memory management constraints.

    Attributes:
        topic_spec (Mapping[str, TopicSpec]): A mapping of topic names
            to their specific recording settings.
        default_topic_spec (TopicSpec): The default topic recording
            settings if a topic is not explicitly listed in `topic_spec`.
        include_patterns (Optional[List[str]]): A list of regex patterns or topics
            specifying which topics should be included. If None, all
            topics are included by default.
        exclude_patterns (Optional[List[str]]): A list of regex patterns or topics
            specifying which topics should be excluded.
        no_discovery (bool): If `True`, disables automatic topic
            discovery. Defaults to `False`.
        max_cache_size (int): The maximum cache size for recorded
            messages, in bytes. Defaults to `256MB (256 * 1024 * 1024)`.
        wait_for_topics (Set[str]): A set of topic names to wait for before starting the recording process.
            This field allows you to specify a list of topics that the node should wait to be successfully subscribed
            to before it begins recording data. This is useful in scenarios where you want to ensure that all necessary
            data sources are available before starting the recording, preventing
            the capture of incomplete or missing information.

            If this set is empty (the default), the recording will start immediately
            without waiting for any specific topics.

            Example usage:
                wait_for_topics = {"/camera/image_raw", "/lidar/points"}
                # The recording will start only after both the camera and lidar topics
                # have been subscribed to.
        static_topics (List[str]): A list of topic names identified as static topics
                (e.g., topics with `Transient Local` durability like `/tf_static`).
                Defaults to an empty list. These topics are typically recorded with special
                handling to ensure their data is preserved even if received before recording starts.
        max_timestamp_difference_ns: (int | None): The maximum allowed difference between consecutive message timestamps, in nanoseconds. Defaults to None.
            This parameter defines the threshold for detecting and handling
            timestamp anomalies in incoming messages. If the absolute difference
            between a message's timestamp and the previously recorded timestamp
            exceeds this value, the message is considered to have a timestamp error
            and will be dropped.

            A value of `None` indicates that no timestamp gap check should be performed,
            effectively disabling this feature.  If a value is provided, it should
            be a non-negative integer.
    """  # noqa

    topic_spec: Mapping[str, TopicSpec] = dict()
    default_topic_spec: TopicSpec = TopicSpec()
    include_patterns: Optional[List[str]] = None
    exclude_patterns: Optional[List[str]] = None
    no_discovery: bool = False
    max_cache_size: int = 256 * 1024 * 1024  # 256mb
    wait_for_topics: Set[str] = Field(default_factory=lambda: set())
    static_topics: List[str] = Field(default_factory=lambda: [])
    max_timestamp_difference_ns: int | None = None
