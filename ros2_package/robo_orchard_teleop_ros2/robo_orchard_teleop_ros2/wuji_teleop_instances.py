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
import re
from collections import Counter
from collections.abc import Hashable, Sequence
from dataclasses import dataclass

_ROS_NAME_SEGMENT_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


@dataclass(frozen=True)
class WujiTeleopInstance:
    """Resolved launch settings for one hand and glove pair."""

    hand_side: str
    hand_name: str
    hand_serial_number: str
    glove_namespace: str
    glove_serial_number: str
    glove_device_name: str
    glove_frame_prefix: str
    glove_sdk_user: str
    algo_topic: str


def _split_sides(value: str) -> list[str]:
    sides = [item.strip() for item in value.split(",")]
    if not sides or any(side not in ("left", "right") for side in sides):
        raise ValueError(
            "hand_side must be a comma-separated list containing only "
            "'left' and 'right'"
        )
    return sides


def _split_optional(value: str, count: int, name: str) -> list[str]:
    if not value.strip():
        return [""] * count
    items = [item.strip() for item in value.split(",")]
    if len(items) != count:
        raise ValueError(
            f"{name} must contain exactly {count} comma-separated values"
        )
    return items


def _split_broadcast(value: str, count: int, name: str) -> list[str]:
    items = [item.strip() or "default" for item in value.split(",")]
    if len(items) == 1:
        return items * count
    if len(items) != count:
        raise ValueError(
            f"{name} must be one value or exactly {count} "
            "comma-separated values"
        )
    return items


def _require_unique(values: Sequence[Hashable], name: str) -> None:
    if len(set(values)) != len(values):
        raise ValueError(f"{name} values must be unique")


def _require_unique_nonempty(values: list[str], name: str) -> None:
    present = [value for value in values if value]
    if len(set(present)) != len(present):
        raise ValueError(f"non-empty {name} values must be unique")


def _instance_key(hand_name: str) -> str:
    key = hand_name.removeprefix("hand_")
    if not key:
        raise ValueError(
            "hand_name must contain text after the 'hand_' prefix"
        )
    return key


def _canonical_namespace(value: str) -> str:
    namespace = value if value.startswith("/") else f"/{value}"
    if len(namespace) > 1:
        namespace = namespace.rstrip("/")
    segments = namespace.removeprefix("/").split("/")
    if namespace != "/" and (
        any(not segment for segment in segments)
        or any(
            not _ROS_NAME_SEGMENT_PATTERN.fullmatch(segment)
            for segment in segments
        )
    ):
        raise ValueError("glove_namespace values must be valid ROS namespaces")
    return namespace


def _canonical_topic(value: str, namespace: str) -> str:
    topic = value if value.startswith("/") else f"{namespace}/{value}"
    segments = topic.removeprefix("/").split("/")
    if (
        not value
        or not topic.startswith("/")
        or any(not segment for segment in segments)
        or any(
            not _ROS_NAME_SEGMENT_PATTERN.fullmatch(segment)
            for segment in segments
        )
    ):
        raise ValueError("algo_topic values must be valid ROS topic names")
    return topic


def resolve_wuji_teleop_instances(
    *,
    hand_side: str,
    hand_name: str = "",
    hand_serial_number: str = "",
    glove_namespace: str = "",
    glove_serial_number: str = "",
    glove_device_name: str = "",
    glove_frame_prefix: str = "",
    glove_sdk_user: str = "default",
    algo_topic: str = "",
) -> list[WujiTeleopInstance]:
    """Expand comma-separated launch values into validated instances."""
    sides = _split_sides(hand_side)
    count = len(sides)
    side_counts = Counter(sides)
    hand_names = _split_optional(hand_name, count, "hand_name")
    hand_serials = _split_optional(
        hand_serial_number, count, "hand_serial_number"
    )
    glove_namespaces = _split_optional(
        glove_namespace, count, "glove_namespace"
    )
    glove_serials = _split_optional(
        glove_serial_number, count, "glove_serial_number"
    )
    glove_device_names = _split_optional(
        glove_device_name, count, "glove_device_name"
    )
    glove_frame_prefixes = _split_optional(
        glove_frame_prefix, count, "glove_frame_prefix"
    )
    glove_sdk_users = _split_broadcast(glove_sdk_user, count, "glove_sdk_user")
    algo_topics = _split_optional(algo_topic, count, "algo_topic")
    bimanual_pair = count == 2 and set(sides) == {"left", "right"}
    explicit_prefixes = {prefix for prefix in glove_frame_prefixes if prefix}
    shared_bimanual_prefix = (
        next(iter(explicit_prefixes), "wuji_glove")
        if bimanual_pair and len(explicit_prefixes) <= 1
        else ""
    )

    instances = []
    for index, side in enumerate(sides):
        repeated_side = side_counts[side] > 1
        if repeated_side and not hand_names[index]:
            raise ValueError(
                f"hand_name is required for repeated hand_side {side!r}"
            )
        if repeated_side and not hand_serials[index]:
            raise ValueError(
                "hand_serial_number is required for every repeated "
                f"hand_side {side!r}"
            )
        if repeated_side and not glove_serials[index]:
            raise ValueError(
                "glove_serial_number is required for every repeated "
                f"hand_side {side!r}"
            )

        resolved_hand_name = hand_names[index] or f"hand_{side}"
        if not _ROS_NAME_SEGMENT_PATTERN.fullmatch(resolved_hand_name):
            raise ValueError(
                "hand_name values may contain only letters, digits, and "
                "underscores, and cannot start with a digit"
            )
        key = _instance_key(resolved_hand_name)
        resolved_namespace = _canonical_namespace(
            glove_namespaces[index] or f"/wuji_glove/{key}"
        )
        resolved_device_name = glove_device_names[index] or f"wuji_glove_{key}"
        resolved_frame_prefix = (
            glove_frame_prefixes[index]
            or shared_bimanual_prefix
            or resolved_device_name
        )
        if "/" in resolved_device_name or "." in resolved_device_name:
            raise ValueError(
                "glove_device_name values cannot contain '/' or '.'"
            )
        resolved_algo_topic = _canonical_topic(
            algo_topics[index] or f"/{key}_hand_algo_cmd",
            f"/{resolved_hand_name}/takeover_muxer",
        )

        instances.append(
            WujiTeleopInstance(
                hand_side=side,
                hand_name=resolved_hand_name,
                hand_serial_number=hand_serials[index],
                glove_namespace=resolved_namespace,
                glove_serial_number=glove_serials[index],
                glove_device_name=resolved_device_name,
                glove_frame_prefix=resolved_frame_prefix,
                glove_sdk_user=glove_sdk_users[index],
                algo_topic=resolved_algo_topic,
            )
        )

    _require_unique([item.hand_name for item in instances], "hand_name")
    _require_unique(
        [item.glove_namespace for item in instances], "glove_namespace"
    )
    _require_unique(
        [item.glove_device_name for item in instances], "glove_device_name"
    )
    _require_unique(
        [(item.glove_frame_prefix, item.hand_side) for item in instances],
        "glove_frame_prefix per hand side",
    )
    _require_unique([item.algo_topic for item in instances], "algo_topic")
    reserved_mux_topics = {
        f"/{item.hand_name}/joint_commands" for item in instances
    } | {
        f"{item.glove_namespace.rstrip('/')}/retargeted_joint_commands"
        for item in instances
    }
    conflicting_topics = sorted(
        {item.algo_topic for item in instances} & reserved_mux_topics
    )
    if conflicting_topics:
        raise ValueError(
            "algo_topic values must not overlap DAgger mux output or override "
            f"topics: {', '.join(conflicting_topics)}"
        )
    _require_unique_nonempty(hand_serials, "hand_serial_number")
    _require_unique_nonempty(glove_serials, "glove_serial_number")
    return instances
