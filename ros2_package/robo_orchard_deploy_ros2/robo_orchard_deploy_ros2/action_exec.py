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

import numpy as np
from rclpy.node import Node

from robo_orchard_deploy_ros2.config import DeployConfig
from robo_orchard_deploy_ros2.topic_manager import TopicManager


class ActionExecutor:
    """Publish one step of a model action onto the configured channels.

    The executor is embodiment agnostic: how many arms or hands exist, what
    their joints are called and which topic drives them all come from
    ``config.control_config.channels``.
    """

    def __init__(self, node: Node, config: DeployConfig):
        self._node = node
        self._config = config
        self._topic_manager = TopicManager(self._node)
        self._outputs = [
            self._build_output(channel)
            for channel in config.control_config.channels
        ]

    def _build_output(self, channel):
        """Resolve the message class and publisher of one action channel."""
        msg_type_class = self._topic_manager.get_message_class(
            channel.msg_type
        )
        if msg_type_class is None:
            raise ImportError(
                f"Cannot import '{channel.msg_type}' for action channel "
                f"'{channel.server_output_key}' on topic '{channel.topic}'."
            )
        publisher = self._topic_manager.create_publisher(
            channel.topic, msg_type_class
        )
        return channel, publisher, msg_type_class

    def _send_joint_action(
        self, channel, publisher, msg_type_class, joint_position
    ):
        if joint_position is None:
            self._node.get_logger().warning(
                f"Channel '{channel.server_output_key}' got no joint "
                f"position, skipping action send."
            )
            return
        if len(joint_position) != len(channel.joint_names):
            self._node.get_logger().error(
                f"Channel '{channel.server_output_key}' expects "
                f"{len(channel.joint_names)} joints but the model returned "
                f"{len(joint_position)}, skipping action send."
            )
            return

        goal_msg = msg_type_class()
        goal_msg.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.name = channel.joint_names
        goal_msg.position = [float(value) for value in joint_position]
        if channel.velocities is not None:
            goal_msg.velocity = channel.velocities
        if channel.efforts is not None:
            goal_msg.effort = channel.efforts
        publisher.publish(goal_msg)

    def action_step_count(self, action):
        """Return how many steps of ``action`` the channels can publish.

        The server returns one row per control step, so a sequence length
        is the horizon. Channels of unequal length mean the response is
        malformed, and a step only some of them can execute must not be
        published at all.

        Args:
            action (dict): The model server response.
        """
        lengths = {
            channel.server_output_key: len(
                action.get(channel.server_output_key) or []
            )
            for channel, _, _ in self._outputs
        }
        if len(set(lengths.values())) > 1:
            self._node.get_logger().error(
                f"Action channels returned different step counts "
                f"{lengths}, refusing to execute the response.",
                throttle_duration_sec=1,
            )
            return 0
        return min(lengths.values(), default=0)

    def remaining_actions(self, action, after_index):
        """Return the steps of ``action`` no channel has published yet.

        Each channel contributes its own rows under its own key, leaving
        the model server to assemble them the way its checkpoint expects.
        Channels without ``server_remaining_key`` contribute nothing.

        Args:
            action (dict): The model server response being executed.
            after_index (int): Index of the step published last.
        """
        remaining = {}
        for channel, _, _ in self._outputs:
            if channel.server_remaining_key is None:
                continue
            sequence = action.get(channel.server_output_key) or []
            rows = sequence[after_index + 1 :]
            if not rows:
                continue
            remaining[channel.server_remaining_key] = np.asarray(
                rows, dtype=np.float32
            )
        return remaining

    def send_action(self, action, action_index):
        """Publish step ``action_index`` of every configured channel.

        Args:
            action (dict): The model server response.
            action_index (int): Step to publish within each action sequence.
        """
        for channel, publisher, msg_type_class in self._outputs:
            sequence = action.get(channel.server_output_key)
            if not sequence:
                self._node.get_logger().warning(
                    f"Model response has no '{channel.server_output_key}', "
                    f"skipping action send.",
                    throttle_duration_sec=1,
                )
                continue
            if action_index >= len(sequence):
                self._node.get_logger().error(
                    f"Channel '{channel.server_output_key}' has "
                    f"{len(sequence)} steps, cannot send step "
                    f"{action_index}.",
                    throttle_duration_sec=1,
                )
                continue
            self._send_joint_action(
                channel, publisher, msg_type_class, sequence[action_index]
            )
