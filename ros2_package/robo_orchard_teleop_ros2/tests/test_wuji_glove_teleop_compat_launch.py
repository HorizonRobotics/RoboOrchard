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
from pathlib import Path

from launch_stubs import (
    IncludeLaunchDescription as _IncludeLaunchDescription,
    OpaqueFunction as _OpaqueFunction,
    load_launch_module,
)

LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "wuji_glove_teleop_compat.launch.py"
)


def _load_module():
    return load_launch_module(LAUNCH_PATH, "wuji_glove_teleop_compat_launch")


def _context(**overrides):
    values = {
        "hand_side": "right",
        "hand_name": "",
        "hand_serial_number": "",
        "glove_namespace": "",
        "glove_serial_number": "",
        "glove_device_name": "",
        "glove_frame_prefix": "",
        "glove_sdk_user": "default",
    }
    values.update(overrides)
    return values


def test_launch_creates_one_hand_and_glove_pair_per_side():
    module = _load_module()
    description = module.generate_launch_description()
    resolver = next(
        entity
        for entity in description.entities
        if isinstance(entity, _OpaqueFunction)
    )
    assert resolver.function is module._launch_instances

    actions = resolver.function(_context(hand_side="left,right"))
    includes = [
        action
        for action in actions
        if isinstance(action, _IncludeLaunchDescription)
    ]
    assert len(includes) == 4

    hand_includes = [
        include
        for include in includes
        if include.source.location.substitutions[0].package
        == "wujihand_bringup"
    ]
    glove_includes = [
        include
        for include in includes
        if include.source.location.substitutions[0].package
        == "robo_orchard_wuji_glove_ros2"
    ]
    assert [
        dict(item.launch_arguments)["hand_name"] for item in hand_includes
    ] == [
        "hand_left",
        "hand_right",
    ]
    assert [
        dict(item.launch_arguments)["glove_namespace"]
        for item in glove_includes
    ] == ["/wuji_glove/left", "/wuji_glove/right"]
    assert [
        dict(item.launch_arguments)["frame_prefix"] for item in glove_includes
    ] == ["wuji_glove", "wuji_glove"]


def test_launch_routes_each_glove_directly_to_its_hand():
    actions = _load_module()._launch_instances(
        _context(hand_side="left,right")
    )
    gloves = [actions[1], actions[3]]
    assert [
        dict(item.launch_arguments)["command_topic"] for item in gloves
    ] == [
        "/hand_left/joint_commands",
        "/hand_right/joint_commands",
    ]
    for glove in gloves:
        arguments = dict(glove.launch_arguments)
        assert arguments["sdk_user"] == "default"
        assert arguments["hand_model_path"].name == "glove_hand_model_path"
        assert arguments["stream_profile"].name == "glove_stream_profile"


def test_launch_forwards_same_side_names_and_serial_numbers():
    actions = _load_module()._launch_instances(
        _context(
            hand_side="left,left",
            hand_name="hand_left_a,hand_left_b",
            hand_serial_number="HAND_A,HAND_B",
            glove_serial_number="GLOVE_A,GLOVE_B",
            glove_sdk_user="Alice,Bob",
        )
    )

    first_hand = dict(actions[0].launch_arguments)
    first_glove = dict(actions[1].launch_arguments)
    second_hand = dict(actions[2].launch_arguments)
    second_glove = dict(actions[3].launch_arguments)
    assert (first_hand["hand_name"], first_hand["serial_number"]) == (
        "hand_left_a",
        "HAND_A",
    )
    assert (
        first_glove["glove_namespace"],
        first_glove["glove_serial_number"],
        first_glove["frame_prefix"],
        first_glove["sdk_user"],
    ) == (
        "/wuji_glove/left_a",
        "GLOVE_A",
        "wuji_glove_left_a",
        "Alice",
    )
    assert (second_hand["hand_name"], second_hand["serial_number"]) == (
        "hand_left_b",
        "HAND_B",
    )
    assert (
        second_glove["glove_namespace"],
        second_glove["glove_serial_number"],
        second_glove["frame_prefix"],
        second_glove["sdk_user"],
    ) == (
        "/wuji_glove/left_b",
        "GLOVE_B",
        "wuji_glove_left_b",
        "Bob",
    )
