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

import cv_bridge
import numpy as np
from message_filters import ApproximateTimeSynchronizer
from rclpy.node import Node

from robo_orchard_deploy_ros2.config import DeployConfig
from robo_orchard_deploy_ros2.topic_manager import TopicManager


class ObservationManager:
    def __init__(self, node: Node, config: DeployConfig):
        self._node = node
        self._config = config
        self.cv_bridge = cv_bridge.CvBridge()
        self.current_observations = {}
        self.obs_lock = threading.Lock()
        self.topic_manager = TopicManager(self._node)
        self._processing_map = {
            "color_topics": self._process_color_image,
            "depth_topics": self._process_depth_image,
            "intrinsic_topics": self._process_camera_intrinsic,
            "arm_state_topics": self._process_arm_state,
        }
        self._subscriber_info = []
        self._initialize()
        self._init_is_ready = False
        self._init_timer = self._node.create_timer(
            1.0, self._attempt_subscriptions
        )
        self.last_timestamp = -1.0

    def _initialize(self):
        obs_config = self._config.observation_config

        for obs_type, topics_dict in obs_config.model_dump().items():
            if obs_type not in self._processing_map:
                continue

            processing_callback = self._processing_map[obs_type]

            for obs_key, topic_name in topics_dict.items():
                self._subscriber_info.append(
                    {
                        "obs_key": obs_key,
                        "topic_name": topic_name,
                        "callback": processing_callback,
                        "msg_type": None,
                        "subscriber_obj": None,
                    }
                )
        self._subscriber_info.sort(key=lambda x: x["topic_name"])

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
        for info in self._subscriber_info:
            if info["topic_name"] not in available_topics:
                self._node.get_logger().warn(
                    f"Topic '{info['topic_name']}' for '{info['obs_key']}' "
                    f"not available yet. Retrying..."
                )
                all_topics_available = False
                break

        if not all_topics_available:
            return

        _subscribers = []
        for info in self._subscriber_info:
            topic_name = info["topic_name"]
            msg_type_str = available_topics[topic_name]
            msg_type_class = self.topic_manager.get_message_class(msg_type_str)

            if msg_type_class is None:
                self._node.get_logger().error(
                    f"Cannot import msg type for topic '{topic_name}'"
                )
                return

            subscriber = self.topic_manager.create_subscriber(
                topic_name, msg_type_class
            )
            _subscribers.append(subscriber)

        self._subscribers = _subscribers
        self.approx_sync = ApproximateTimeSynchronizer(
            self._subscribers, queue_size=1, slop=0.1
        )
        self.approx_sync.registerCallback(self._observe_callback)

        self._init_is_ready = True
        self._init_timer.cancel()
        self._node.get_logger().info(
            "All observation topics are subscribed and synchronized."
        )

    def _process_color_image(self, msg):
        return self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _process_depth_image(self, msg):
        return self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def _process_camera_intrinsic(self, msg):
        return np.array(msg.p).reshape(3, 4)

    def _process_arm_state(self, msg):
        return np.array(msg.position)

    def _observe_callback(self, *msgs):
        """Callback function for synchronized observations."""
        _cur_observations = {}
        _obs_timestamps = []

        for info, msg in zip(self._subscriber_info, msgs, strict=True):
            if hasattr(msg, "header"):
                timestamp = (
                    msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                )
                _obs_timestamps.append(timestamp)

            obs_key = info["obs_key"]
            processing_callback = info["callback"]
            _cur_observations[obs_key] = processing_callback(msg)

        obs_timestamp = np.mean(_obs_timestamps) if _obs_timestamps else 0.0
        with self.obs_lock:
            self.current_observations.update(_cur_observations)
            self.current_observations["timestamp"] = obs_timestamp

    def get_observations(self):
        with self.obs_lock:
            current_ts = self.current_observations.get("timestamp", -1.0)
            if self.last_timestamp == current_ts:
                self._node.get_logger().debug(
                    "No new observations since last call,"
                    "check if the topics are publishing."
                )
                return None
            self.last_timestamp = current_ts
            return self.current_observations.copy()
