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

"""Decode ROS 2 messages into the arrays sent to the model server.

This is the only place that knows how a message becomes an array, so adding
an observation kind means adding a channel class in
:mod:`robo_orchard_deploy_ros2.config` and a branch here.
"""

import cv_bridge
import numpy as np

from robo_orchard_deploy_ros2.config import (
    CameraInfoChannel,
    ImageChannel,
    JointStateChannel,
    ObsChannelBase,
)

__all__ = ["decode"]

_BRIDGE = cv_bridge.CvBridge()


def decode(channel: ObsChannelBase, msg) -> np.ndarray:
    """Convert one received message into its observation array.

    Args:
        channel (ObsChannelBase): Channel declaration the message arrived for.
        msg: The received ROS 2 message.

    Returns:
        np.ndarray: The array published under ``channel.server_input_key``.

    Raises:
        TypeError: If the channel kind has no decoder.
        KeyError: If a joint declared by the channel is missing in the message.
    """
    if isinstance(channel, ImageChannel):
        return _BRIDGE.imgmsg_to_cv2(msg, desired_encoding=channel.encoding)
    if isinstance(channel, CameraInfoChannel):
        return np.array(msg.p).reshape(3, 4)
    if isinstance(channel, JointStateChannel):
        return _decode_joint_state(channel, msg)
    raise TypeError(
        f"No decoder for observation channel {type(channel).__name__}."
    )


def _decode_joint_state(channel: JointStateChannel, msg) -> np.ndarray:
    """Read joint positions in the order the model expects.

    Args:
        channel (JointStateChannel): The joint state channel declaration.
        msg: A ``sensor_msgs/msg/JointState`` message.

    Returns:
        np.ndarray: Joint positions ordered by ``channel.joint_names``, or in
        the published order when the channel declares no names.

    Raises:
        KeyError: If a declared joint is absent from the message.
    """
    if channel.joint_names is None:
        return np.asarray(msg.position, dtype=np.float64)

    positions = dict(zip(msg.name, msg.position, strict=False))
    missing = [n for n in channel.joint_names if n not in positions]
    if missing:
        raise KeyError(
            f"Channel '{channel.server_input_key}' expects joints {missing}, "
            f"which topic '{channel.topic}' does not publish."
        )
    return np.asarray(
        [positions[name] for name in channel.joint_names], dtype=np.float64
    )
