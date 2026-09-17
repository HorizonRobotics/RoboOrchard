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
    DeclareLaunchArgument as _DeclareLaunchArgument,
    IncludeLaunchDescription as _IncludeLaunchDescription,
    Node as _Node,
    OpaqueFunction as _OpaqueFunction,
    load_launch_module,
)

LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "wuji_glove_dagger_compat.launch.py"
)


def _load_module():
    return load_launch_module(LAUNCH_PATH, "wuji_glove_dagger_compat_launch")


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
        "algo_topic": "",
    }
    values.update(overrides)
    return values


def test_launch_creates_hand_pairs_and_one_manager():
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
    assert not [action for action in actions if isinstance(action, _Node)]
    managers = [
        entity
        for entity in description.entities
        if isinstance(entity, _Node)
        and entity.kwargs.get("executable") == "control_manager_node"
    ]
    assert len(managers) == 1


def test_launch_routes_each_glove_to_manager_override():
    actions = _load_module()._launch_instances(
        _context(hand_side="left,right")
    )
    gloves = [actions[1], actions[3]]
    expected = [
        "/hand_left/control/override",
        "/hand_right/control/override",
    ]

    for glove, override_topic in zip(gloves, expected, strict=True):
        arguments = dict(glove.launch_arguments)
        assert arguments["command_topic"] == override_topic
        assert arguments["sdk_user"] == "default"
        assert arguments["hand_model_path"].name == "glove_hand_model_path"
        assert arguments["stream_profile"].name == "glove_stream_profile"
        assert arguments["frame_prefix"] == "wuji_glove"


def test_launch_uses_hand_name_for_manager_override_topic():
    actions = _load_module()._launch_instances(
        _context(glove_namespace="custom/glove/")
    )
    glove = dict(actions[1].launch_arguments)
    assert glove["glove_namespace"] == "/custom/glove"
    assert glove["command_topic"] == "/hand_right/control/override"


def test_launch_defaults_instance_arguments_to_side_derivation():
    description = _load_module().generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    }

    for name in (
        "hand_name",
        "glove_namespace",
        "glove_device_name",
        "glove_frame_prefix",
        "algo_topic",
    ):
        assert arguments[name].default_value == ""
    assert arguments["glove_stream_profile"].default_value == "teleop_minimal"
    assert "replay_time_s" not in arguments
