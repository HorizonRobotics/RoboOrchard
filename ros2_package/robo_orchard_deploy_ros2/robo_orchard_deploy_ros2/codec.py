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

"""Decode ROS 2 messages into observations sent to the model server.

This module converts messages into transport observations. Adding an
observation kind means adding a channel class in
:mod:`robo_orchard_deploy_ros2.config` and a branch here.
"""

from typing import Any

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


def decode(
    channel: ObsChannelBase, msg: Any
) -> np.ndarray | dict[str, list[str] | list[float]]:
    """Convert a message into an array or named joint observation.

    Args:
        channel (ObsChannelBase): Channel declaration the message arrived for.
        msg: The received ROS 2 message.

    Returns:
        An array for images/camera info, or a dictionary containing paired
        ``name`` and ``position`` lists for joints. Positions are in the ROS
        message's units; selected names retain their identity and ordering.

    Raises:
        TypeError: If the channel kind has no decoder.
        KeyError: If a joint declared by the channel is missing in the message.
        ValueError: If joint names or positions are malformed.
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


def _decode_joint_state(
    channel: JointStateChannel, msg
) -> dict[str, list[str] | list[float]]:
    """Copy paired names and positions, optionally selecting ROS joints.

    Args:
        channel (JointStateChannel): The joint state channel declaration.
        msg: A ``sensor_msgs/msg/JointState`` message.

    Returns:
        A JSON-compatible observation in configured or published order.

    Raises:
        KeyError: If a declared joint is absent from the message.
    """
    names = list(msg.name)
    values = np.asarray(msg.position, dtype=np.float64)
    if (
        not names
        or len(names) != len(set(names))
        or any(not name.strip() for name in names)
        or values.ndim != 1
        or len(names) != values.size
        or not np.all(np.isfinite(values))
    ):
        raise ValueError(
            "Joint names and finite positions must match uniquely"
        )
    positions = dict(zip(names, values.tolist(), strict=True))
    selected_names = (
        names if channel.joint_names is None else list(channel.joint_names)
    )
    if not selected_names or len(selected_names) != len(set(selected_names)):
        raise ValueError("Selected joint names must be non-empty and unique")
    missing = [name for name in selected_names if name not in positions]
    if missing:
        raise KeyError(
            f"Channel '{channel.server_input_key}' expects joints {missing}, "
            f"which topic '{channel.topic}' does not publish."
        )
    return {
        "name": selected_names,
        "position": [positions[name] for name in selected_names],
    }
