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

from typing import TYPE_CHECKING, Any

import numpy as np
from geometry_msgs.msg import PoseStamped
from numpy.typing import NDArray
from rclpy.node import Node

from robo_orchard_deploy_ros2.command_limiter import CommandLimiter
from robo_orchard_deploy_ros2.config import (
    DeployConfig,
    JointCommandChannel,
    PoseCommandChannel,
)
from robo_orchard_deploy_ros2.topic_manager import TopicManager

if TYPE_CHECKING:
    from rclpy.publisher import Publisher


def _pose_values(row: Any) -> NDArray[np.float64]:
    """Read a finite xyz,wxyz row with a nonzero quaternion."""
    try:
        values = np.asarray(row, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("expected seven finite pose values") from exc
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError("expected seven finite pose values")
    if not values[3:].any():
        raise ValueError("zero quaternion")
    return values


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
        # Keyed per channel, so it does not assume how many there are.
        self._limiter = CommandLimiter(
            config.max_command_velocity,
            config.control_config.control_frequency,
            node.get_logger(),
        )

    def reset_limiter(self):
        """Forget the last command sent on every channel.

        Call this on the pause boundary, where no state from before
        describes the robot after: it may have been moved, driven by
        something else, or powered down.

        Not on an ordinary gap between chunks. There the robot is holding
        the last command, which still says where it is, and clamping
        against it is both correct and protective.
        """
        self._limiter.reset()

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
        # Clamped after validation, so a malformed step is rejected on its
        # own terms rather than smoothed into a plausible one.
        goal_msg.position = [
            float(value)
            for value in self._limiter.apply(
                joint_position, channel.server_output_key
            )
        ]
        if channel.velocities is not None:
            goal_msg.velocity = channel.velocities
        if channel.efforts is not None:
            goal_msg.effort = channel.efforts
        publisher.publish(goal_msg)

    def _send_pose_action(
        self,
        channel: PoseCommandChannel,
        publisher: "Publisher",
        msg_type_class: type,
        row: Any,
        action_index: int,
        observation_stamp: tuple[int, int] | None,
    ) -> None:
        if observation_stamp is None:
            self._node.get_logger().error(
                "Pose action requires its request's observation timestamp",
                throttle_duration_sec=1,
            )
            return

        try:
            values = _pose_values(row)
        except ValueError as exc:
            self._node.get_logger().error(
                f"Channel '{channel.server_output_key}' has invalid pose: "
                f"{exc}.",
                throttle_duration_sec=1,
            )
            return
        quaternion = values[3:]
        scale = np.max(np.abs(quaternion))
        # Scale before taking the norm to avoid overflow/underflow.
        quaternion = quaternion / scale
        quaternion /= np.linalg.norm(quaternion)

        message = msg_type_class()
        message.header.stamp.sec, message.header.stamp.nanosec = (
            observation_stamp
        )
        message.header.frame_id = channel.frame_id
        pose = PoseStamped()
        pose.header.frame_id = channel.frame_id
        # Use the original row index and keep ROS time as an integer.
        sample_ns = (
            observation_stamp[0] * 1_000_000_000
            + observation_stamp[1]
            + round(
                (action_index + 1)
                * 1_000_000_000
                / self._config.control_config.control_frequency
            )
        )
        pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(
            sample_ns, 1_000_000_000
        )
        for axis, value in zip("xyz", values[:3], strict=True):
            setattr(pose.pose.position, axis, float(value))
        for axis, value in zip("wxyz", quaternion, strict=True):
            setattr(pose.pose.orientation, axis, float(value))
        message.poses = [pose]
        publisher.publish(message)

    def action_step_count(self, action: dict[str, Any]) -> int:
        """Return how many steps of ``action`` the channels can publish.

        The server returns one row per control step, so a sequence length
        is the horizon. Channels of unequal length mean the response is
        malformed, and a step only some of them can execute must not be
        published at all.

        Args:
            action (dict): The model server response.
        """
        lengths = {}
        for channel, _, _ in self._outputs:
            key = channel.server_output_key
            sequence = action.get(key, [])
            if isinstance(channel, PoseCommandChannel) and not isinstance(
                sequence, list
            ):
                self._node.get_logger().error(
                    f"Channel '{key}' expects an array of pose rows.",
                    throttle_duration_sec=1,
                )
                return 0
            lengths[key] = len(sequence or [])
        if len(set(lengths.values())) > 1:
            self._node.get_logger().error(
                f"Action channels returned different step counts "
                f"{lengths}, refusing to execute the response.",
                throttle_duration_sec=1,
            )
            return 0
        return min(lengths.values(), default=0)

    def validated_step_count(self, action: dict[str, Any]) -> int:
        """Return the response horizon, or zero if any pose row is invalid."""
        step_count = self.action_step_count(action)
        if not step_count:
            return 0
        for channel, _, _ in self._outputs:
            if not isinstance(channel, PoseCommandChannel):
                continue
            key = channel.server_output_key
            for index, row in enumerate(action[key]):
                try:
                    _pose_values(row)
                except ValueError as exc:
                    self._node.get_logger().error(
                        f"Channel '{key}' has invalid pose at row {index}: "
                        f"{exc}; refusing to execute the response.",
                        throttle_duration_sec=1,
                    )
                    return 0
        return step_count

    def remaining_actions(
        self, action: dict[str, Any], after_index: int
    ) -> dict[str, NDArray[np.float32]]:
        """Return the steps of ``action`` no channel has published yet.

        Each channel contributes its own rows under its own key, leaving
        the model server to assemble them the way its checkpoint expects.
        Channels without ``server_remaining_key`` contribute nothing.
        Only joint channels contribute RTC tails. Pose rows belong to an
        earlier observation origin and cannot be returned without a frame
        transformation.

        Args:
            action (dict): The model server response being executed.
            after_index (int): Index of the step published last.
        """
        remaining = {}
        for channel, _, _ in self._outputs:
            if (
                not isinstance(channel, JointCommandChannel)
                or channel.server_remaining_key is None
            ):
                continue
            sequence = action.get(channel.server_output_key) or []
            rows = sequence[after_index + 1 :]
            if not rows:
                continue
            remaining[channel.server_remaining_key] = np.asarray(
                rows, dtype=np.float32
            )
        return remaining

    def send_action(
        self,
        action: dict[str, Any],
        action_index: int,
        observation_stamp: tuple[int, int] | None = None,
    ) -> None:
        """Publish step ``action_index`` of every configured channel.

        Args:
            action (dict): The model server response.
            action_index: Step to publish within each action sequence.
            observation_stamp: Request reference (sec, nanosec) for poses.
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
            row = sequence[action_index]
            if isinstance(channel, PoseCommandChannel):
                self._send_pose_action(
                    channel,
                    publisher,
                    msg_type_class,
                    row,
                    action_index,
                    observation_stamp,
                )
            else:
                self._send_joint_action(
                    channel, publisher, msg_type_class, row
                )
