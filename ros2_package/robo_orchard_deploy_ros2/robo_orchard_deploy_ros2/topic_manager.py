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

from message_filters import Subscriber
from rclpy.node import Node


class TopicManager:
    def __init__(self, node: Node):
        self._node = node

    def get_message_class(self, msg_type: str):
        """Dynamically imports the message type.

        Args:
            msg_type (str): The ROS 2 message type (e.g.,
                "std_msgs/msg/String").

        Returns:
            type: The message class, or None if the import fails.
        """
        try:
            module_name, class_name = msg_type.rsplit("/", 1)
            module_name = ".".join(module_name.split("/"))
            module = __import__(module_name, fromlist=[class_name])
            return getattr(module, class_name)
        except Exception as e:
            self._node.get_logger().error(f"Failed to import {msg_type}: {e}")
            return None

    def create_publisher(self, topic_name: str, msg_type_class):
        """Creates a publisher for the given topic and message type.

        Args:
            topic (str): The topic name.
            msg_type_class: The message class.

        Returns:
            Publisher: The created publisher.
        """

        return self._node.create_publisher(msg_type_class, topic_name, 1)

    def create_subscriber(self, topic_name: str, msg_type_class):
        """Creates a subscriber for the given topic and message type.

        Args:
            topic (str): The topic name.
            msg_type_class: The message class.

        Returns:
            Subscriber: The created subscriber.
        """
        return Subscriber(self._node, msg_type_class, topic_name)
