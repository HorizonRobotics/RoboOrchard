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
    LaunchConfiguration as _LaunchConfiguration,
    OpaqueFunction as _OpaqueFunction,
    load_launch_module,
)

LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "marvin_wuji_keyboard_teleop.launch.py"
)


def _load_module():
    return load_launch_module(
        LAUNCH_PATH, "marvin_wuji_keyboard_teleop_launch"
    )


def _description():
    return _load_module().generate_launch_description()


def _includes(description):
    return [
        entity
        for entity in description.entities
        if isinstance(entity, _IncludeLaunchDescription)
    ]


def _include_by_file(description, launch_file):
    return next(
        include
        for include in _includes(description)
        if include.source.location.substitutions[2] == launch_file
    )


def _arguments(description):
    return {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    }


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


def test_launch_includes_marvin_and_keyboard():
    description = _description()
    included = [
        include.source.location.substitutions[2]
        for include in _includes(description)
    ]
    assert sorted(included) == [
        "marvin_pico_teleop.launch.py",
        "teleop_keyboard.launch.py",
    ]


def test_launch_adds_one_hand_and_glove_pair_per_side():
    module = _load_module()
    description = module.generate_launch_description()
    resolver = next(
        entity
        for entity in description.entities
        if isinstance(entity, _OpaqueFunction)
    )
    assert resolver.function is module._launch_instances

    actions = resolver.function(_context(hand_side="left,right"))
    assert len(actions) == 4
    assert [
        dict(action.launch_arguments).get("command_topic")
        for action in actions
        if "command_topic" in dict(action.launch_arguments)
    ] == ["/hand_left/joint_commands", "/hand_right/joint_commands"]


def test_activation_source_is_pinned_to_keyboard():
    marvin = _include_by_file(_description(), "marvin_pico_teleop.launch.py")
    # A literal, not a LaunchConfiguration: the operator wears gloves and
    # cannot reach the Pico grip button, so this must not be overridable.
    assert dict(marvin.launch_arguments)["operator_input_source"] == "keyboard"


def test_keyboard_config_comes_from_dedicated_argument():
    description = _description()
    keyboard = _include_by_file(description, "teleop_keyboard.launch.py")
    config_file = dict(keyboard.launch_arguments)["config_file"]
    assert isinstance(config_file, _LaunchConfiguration)
    assert config_file.name == "keyboard_config_file"
    # A bare config_file would leak into the glove includes as well.
    assert "config_file" not in _arguments(description)
    # Required, not defaulted: the only shipped config is the example
    # template, so any default would point at a file that does not exist
    # on a clean install.
    assert (
        _arguments(description)["keyboard_config_file"].default_value is None
    )


def test_driver_config_defaults_to_the_wuji_hand_profile():
    default = _arguments(_description())["driver_config_file"].default_value
    assert default.substitutions[0].package == "robo_orchard_marvin_ros2"
    assert default.substitutions[2] == "marvin_m6s_with_wujihand.yaml"


def test_both_arms_follow_the_keyboard_by_default():
    arguments = _arguments(_description())
    assert arguments["keyboard_control_side"].default_value == "both"
    assert arguments["auto_enable_side"].default_value == "both"
