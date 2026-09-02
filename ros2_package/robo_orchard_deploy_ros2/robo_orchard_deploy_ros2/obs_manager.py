# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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

from message_filters import ApproximateTimeSynchronizer
from rclpy.node import Node

from robo_orchard_deploy_ros2 import codec
from robo_orchard_deploy_ros2.config import DeployConfig
from robo_orchard_deploy_ros2.topic_manager import TopicManager


class ObservationManager:
    """Subscribe the configured observation channels and sync them.

    The manager is embodiment agnostic: which topics exist, what they carry
    and under which key the model server expects them all come from
    ``config.observation_config.channels``.
    """

    def __init__(self, node: Node, config: DeployConfig):
        self._node = node
        self._config = config
        self.current_observations = {}
        self.obs_lock = threading.Lock()
        self.topic_manager = TopicManager(self._node)
        self._channels = list(config.observation_config.channels)
        self._init_is_ready = False
        self._init_timer = self._node.create_timer(
            1.0, self._attempt_subscriptions
        )
        self._has_new_frame = False

    def _attempt_subscriptions(self):
        """Timer callback to attempt subscribing all observation topics."""
        if self._init_is_ready:
            self._init_timer.cancel()
            return

        available_topics = {
            name: types[0]
            for name, types in self._node.get_topic_names_and_types()
            if types
        }

        all_topics_available = True
        for channel in self._channels:
            if channel.topic not in available_topics:
                self._node.get_logger().warning(
                    f"Topic '{channel.topic}' for "
                    f"'{channel.server_input_key}' not available yet. "
                    f"Retrying..."
                )
                all_topics_available = False
                break

        if not all_topics_available:
            return

        _subscribers = []
        for channel in self._channels:
            published_type = available_topics[channel.topic]
            if published_type != channel.msg_type:
                self._node.get_logger().warning(
                    f"Topic '{channel.topic}' publishes '{published_type}' "
                    f"but the channel declares '{channel.msg_type}'. "
                    f"Subscribing with the declared type."
                )
            msg_type_class = self.topic_manager.get_message_class(
                channel.msg_type
            )

            if msg_type_class is None:
                self._node.get_logger().error(
                    f"Cannot import msg type for topic '{channel.topic}'"
                )
                return

            _subscribers.append(
                self.topic_manager.create_subscriber(
                    channel.topic,
                    msg_type_class,
                    qos_profile=channel.qos_profile,
                )
            )

        self._subscribers = _subscribers
        self.approx_sync = ApproximateTimeSynchronizer(
            self._subscribers,
            queue_size=self._config.observation_config.sync_queue_size,
            slop=self._config.observation_config.sync_slop,
        )
        self.approx_sync.registerCallback(self._observe_callback)

        self._init_is_ready = True
        self._init_timer.cancel()
        self._node.get_logger().info(
            "All observation topics are subscribed and synchronized."
        )

    def _observe_callback(self, *msgs):
        """Callback function for synchronized observations."""
        _cur_observations = {}

        for channel, msg in zip(self._channels, msgs, strict=True):
            try:
                _cur_observations[channel.server_input_key] = codec.decode(
                    channel, msg
                )
            except Exception as error:
                self._node.get_logger().error(
                    f"Dropping observation frame: channel "
                    f"'{channel.server_input_key}' on topic "
                    f"'{channel.topic}' failed to decode: {error}"
                )
                return

        with self.obs_lock:
            self.current_observations = _cur_observations
            self._has_new_frame = True

    def get_observations(self):
        """Return the newest frame, or None if it was already served.

        The caller polls at its own frequency, which is usually slower than
        the topics publish, so the same frame would otherwise be inferred
        more than once.
        """
        with self.obs_lock:
            if not self._has_new_frame:
                self._node.get_logger().debug(
                    "No new observations since last call,"
                    "check if the topics are publishing."
                )
                return None
            self._has_new_frame = False
            return self.current_observations.copy()
