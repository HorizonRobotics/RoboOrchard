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

import pytest

from robo_orchard_teleop_ros2.wuji_teleop_instances import (
    resolve_wuji_teleop_instances,
)


def test_single_side_preserves_simple_defaults():
    [instance] = resolve_wuji_teleop_instances(hand_side="right")

    assert instance.hand_side == "right"
    assert instance.hand_name == "hand_right"
    assert instance.hand_serial_number == ""
    assert instance.glove_namespace == "/wuji_glove/right"
    assert instance.glove_serial_number == ""
    assert instance.glove_device_name == "wuji_glove_right"
    assert instance.glove_frame_prefix == "wuji_glove_right"
    assert instance.glove_sdk_user == "default"
    assert instance.algo_topic == "/right_hand_algo_cmd"


def test_left_and_right_expand_without_other_arguments():
    left, right = resolve_wuji_teleop_instances(hand_side="left,right")

    assert left.hand_name == "hand_left"
    assert left.glove_namespace == "/wuji_glove/left"
    assert left.glove_device_name == "wuji_glove_left"
    assert left.glove_frame_prefix == "wuji_glove"
    assert left.algo_topic == "/left_hand_algo_cmd"
    assert right.hand_name == "hand_right"
    assert right.glove_namespace == "/wuji_glove/right"
    assert right.glove_device_name == "wuji_glove_right"
    assert right.glove_frame_prefix == "wuji_glove"
    assert right.algo_topic == "/right_hand_algo_cmd"


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({}, "hand_name is required"),
        (
            {"hand_name": "hand_left_a,hand_left_b"},
            "hand_serial_number is required",
        ),
        (
            {
                "hand_name": "hand_left_a,hand_left_b",
                "hand_serial_number": "HAND_A,HAND_B",
            },
            "glove_serial_number is required",
        ),
    ],
)
def test_repeated_side_requires_names_and_physical_device_ids(
    overrides, message
):
    with pytest.raises(ValueError, match=message):
        resolve_wuji_teleop_instances(hand_side="left,left", **overrides)


def test_repeated_side_derives_remaining_names_from_hand_name():
    first, second = resolve_wuji_teleop_instances(
        hand_side="left,left",
        hand_name="hand_left_a,hand_left_b",
        hand_serial_number="HAND_A,HAND_B",
        glove_serial_number="GLOVE_A,GLOVE_B",
    )

    assert first.hand_name == "hand_left_a"
    assert first.hand_serial_number == "HAND_A"
    assert first.glove_namespace == "/wuji_glove/left_a"
    assert first.glove_serial_number == "GLOVE_A"
    assert first.glove_device_name == "wuji_glove_left_a"
    assert first.glove_frame_prefix == "wuji_glove_left_a"
    assert first.algo_topic == "/left_a_hand_algo_cmd"
    assert second.hand_name == "hand_left_b"
    assert second.glove_namespace == "/wuji_glove/left_b"
    assert second.glove_device_name == "wuji_glove_left_b"
    assert second.glove_frame_prefix == "wuji_glove_left_b"
    assert second.algo_topic == "/left_b_hand_algo_cmd"


def test_empty_list_positions_keep_per_instance_derivation():
    left, right = resolve_wuji_teleop_instances(
        hand_side="left,right",
        hand_name="custom_left,",
        glove_namespace=",/custom/right_glove",
        glove_frame_prefix="custom_left_frames,",
        algo_topic="/custom_left_algo,",
    )

    assert left.hand_name == "custom_left"
    assert left.glove_namespace == "/wuji_glove/custom_left"
    assert left.glove_frame_prefix == "custom_left_frames"
    assert left.algo_topic == "/custom_left_algo"
    assert right.hand_name == "hand_right"
    assert right.glove_namespace == "/custom/right_glove"
    assert right.glove_frame_prefix == "custom_left_frames"
    assert right.algo_topic == "/right_hand_algo_cmd"


def test_relative_algo_topics_resolve_in_each_mux_namespace():
    left, right = resolve_wuji_teleop_instances(
        hand_side="left,right", algo_topic="commands,commands"
    )

    assert left.algo_topic == "/hand_left/takeover_muxer/commands"
    assert right.algo_topic == "/hand_right/takeover_muxer/commands"


@pytest.mark.parametrize(
    ("arguments", "conflict"),
    [
        (
            {"hand_side": "right", "algo_topic": "/hand_right/joint_commands"},
            "/hand_right/joint_commands",
        ),
        (
            {
                "hand_side": "right",
                "algo_topic": ("/wuji_glove/right/retargeted_joint_commands"),
            },
            "/wuji_glove/right/retargeted_joint_commands",
        ),
        (
            {
                "hand_side": "left,right",
                "algo_topic": (
                    "/hand_right/joint_commands,/hand_left/joint_commands"
                ),
            },
            "/hand_left/joint_commands",
        ),
    ],
)
def test_algo_topics_cannot_overlap_mux_command_roles(arguments, conflict):
    with pytest.raises(ValueError, match=f"overlap.*{conflict}"):
        resolve_wuji_teleop_instances(**arguments)


def test_namespace_is_canonicalized_before_identity_checks():
    [instance] = resolve_wuji_teleop_instances(
        hand_side="right", glove_namespace="custom/glove/"
    )
    assert instance.glove_namespace == "/custom/glove"

    with pytest.raises(ValueError, match="glove_namespace.*unique"):
        resolve_wuji_teleop_instances(
            hand_side="left,right", glove_namespace="same,/same/"
        )


@pytest.mark.parametrize("namespace", ["foo//bar", "foo-name", "1foo"])
def test_invalid_glove_namespace_is_rejected(namespace):
    with pytest.raises(ValueError, match="valid ROS namespaces"):
        resolve_wuji_teleop_instances(
            hand_side="right", glove_namespace=namespace
        )


def test_sdk_user_supports_broadcast_and_per_instance_values():
    broadcast = resolve_wuji_teleop_instances(
        hand_side="left,right", glove_sdk_user="Alice"
    )
    assert [item.glove_sdk_user for item in broadcast] == ["Alice", "Alice"]

    per_instance = resolve_wuji_teleop_instances(
        hand_side="left,right", glove_sdk_user="Alice,Bob"
    )
    assert [item.glove_sdk_user for item in per_instance] == ["Alice", "Bob"]


def test_opposite_sides_may_share_a_frame_prefix():
    instances = resolve_wuji_teleop_instances(
        hand_side="left,right", glove_frame_prefix="operator,operator"
    )
    assert [item.glove_frame_prefix for item in instances] == [
        "operator",
        "operator",
    ]


def test_same_side_frame_prefixes_remain_unique():
    with pytest.raises(ValueError, match="glove_frame_prefix.*unique"):
        resolve_wuji_teleop_instances(
            hand_side="left,left",
            hand_name="hand_left_a,hand_left_b",
            hand_serial_number="HAND_A,HAND_B",
            glove_serial_number="GLOVE_A,GLOVE_B",
            glove_frame_prefix="operator,operator",
        )


def test_per_instance_lists_must_match_instance_count():
    with pytest.raises(ValueError, match="exactly 2 comma-separated values"):
        resolve_wuji_teleop_instances(
            hand_side="left,right",
            hand_name="only_one_name",
        )


@pytest.mark.parametrize(
    ("argument", "value"),
    [
        ("hand_name", "same,same"),
        ("hand_serial_number", "HAND_A,HAND_A"),
        ("glove_serial_number", "GLOVE_A,GLOVE_A"),
        ("glove_namespace", "/same,/same"),
        ("glove_device_name", "same,same"),
        ("algo_topic", "/same,/same"),
    ],
)
def test_instance_identity_values_must_be_unique(argument, value):
    arguments = {
        "hand_side": "left,right",
        "hand_name": "hand_left,hand_right",
        argument: value,
    }
    with pytest.raises(ValueError, match="must be unique"):
        resolve_wuji_teleop_instances(**arguments)
