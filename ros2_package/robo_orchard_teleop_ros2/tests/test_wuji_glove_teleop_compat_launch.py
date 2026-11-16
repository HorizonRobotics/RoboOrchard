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
from unittest.mock import patch

import pytest
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
    / "wuji_glove_teleop_compat.launch.py"
)


def _load_module():
    return load_launch_module(LAUNCH_PATH, "wuji_glove_teleop_compat_launch")


def _context(**overrides):
    values = {
        "hand_side": "right",
        "hand_model": "wuji_hand",
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


@pytest.mark.parametrize("dagger", [False, True])
def test_every_declared_hand2_setting_is_forwarded(dagger):
    filename = (
        "wuji_glove_dagger_compat.launch.py"
        if dagger
        else "wuji_glove_teleop_compat.launch.py"
    )
    module = load_launch_module(LAUNCH_PATH.parent / filename, filename)
    description = module.generate_launch_description()
    defaults = {
        entity.name: entity.default_value
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    }
    hand2_defaults = {
        name.removeprefix("hand2_"): value
        for name, value in defaults.items()
        if name.startswith("hand2_")
    }
    context = {
        **defaults,
        "hand_model": "wuji_hand_2",
        "hand_side": "left,right",
        "hand_serial_number": "000123,000456",
    }
    actions = module._launch_instances(context)
    hands = [
        action
        for action in actions
        if isinstance(action, _IncludeLaunchDescription)
        and action.source.location.substitutions[0].package
        == "robo_orchard_wuji2_ros2"
    ]
    assert len(hands) == 2
    for hand in hands:
        forwarded = dict(hand.launch_arguments)
        assert set(forwarded) - {
            "hand_name",
            "hand_side",
            "serial_number",
        } == (set(hand2_defaults))
        for name, default in hand2_defaults.items():
            assert forwarded[name].name == f"hand2_{name}"
            assert forwarded[name].perform(context) == default


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


def test_launch_routes_each_glove_to_manager_override():
    actions = _load_module()._launch_instances(
        _context(hand_side="left,right")
    )
    gloves = [actions[1], actions[3]]
    assert [
        dict(item.launch_arguments)["command_topic"] for item in gloves
    ] == [
        "/hand_left/control/override",
        "/hand_right/control/override",
    ]
    for glove in gloves:
        arguments = dict(glove.launch_arguments)
        assert arguments["sdk_user"] == "default"
        assert arguments["hand_model_path"].name == "glove_hand_model_path"
        assert arguments["stream_profile"].name == "glove_stream_profile"


@pytest.mark.parametrize("model", ["wuji_hand", "wuji_hand_2"])
@pytest.mark.parametrize(
    "filename",
    [
        "wuji_glove_teleop_compat.launch.py",
        "wuji_glove_dagger_compat.launch.py",
        "marvin_wuji_keyboard_teleop.launch.py",
    ],
)
def test_shared_confidence_override_is_optional_and_glove_only(
    model, filename
):
    module = load_launch_module(
        LAUNCH_PATH.with_name(filename), "wuji_confidence_launch"
    )
    description = module.generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    }
    assert arguments["glove_min_joint_confidence"].default_value == ""
    context = _context(
        hand_side="left,right",
        hand_model=model,
        hand_serial_number="000123,000456",
        algo_topic="",
        glove_min_joint_confidence="0.0",
    )
    actions = module._launch_instances(context)
    for hand, glove in (actions[:2], actions[2:4]):
        assert "min_joint_confidence" not in dict(hand.launch_arguments)
        forwarded = dict(glove.launch_arguments)["min_joint_confidence"]
        assert forwarded.name == "glove_min_joint_confidence"
        assert forwarded.perform(context) == "0.0"


def test_launch_starts_exactly_one_control_manager():
    description = _load_module().generate_launch_description()
    managers = [
        entity
        for entity in description.entities
        if isinstance(entity, _Node)
        and entity.kwargs.get("executable") == "control_manager_node"
    ]
    assert len(managers) == 1
    assert managers[0].kwargs["namespace"] == "/robot/control"


@pytest.mark.parametrize("model", ["wuji_hand", "wuji_hand_2"])
@pytest.mark.parametrize("launch_name", ["teleop", "dagger"])
def test_launch_forwards_same_side_names_and_serial_numbers(
    model, launch_name
):
    path = LAUNCH_PATH.with_name(f"wuji_glove_{launch_name}_compat.launch.py")
    module = load_launch_module(path, f"wuji_same_side_{launch_name}")
    actions = module._launch_instances(
        _context(
            hand_side="left,left",
            hand_model=model,
            hand_name="hand_left_a,hand_left_b",
            hand_serial_number="HAND_A,HAND_B",
            glove_serial_number="GLOVE_A,GLOVE_B",
            glove_sdk_user="Alice,Bob",
            algo_topic="",
        )
    )

    first_hand = dict(actions[0].launch_arguments)
    first_glove = dict(actions[1].launch_arguments)
    second_hand = dict(actions[2].launch_arguments)
    second_glove = dict(actions[3].launch_arguments)
    assert first_glove["hand_model"] == model
    assert second_glove["hand_model"] == model
    assert first_glove["command_topic"] == "/hand_left_a/control/override"
    assert second_glove["command_topic"] == "/hand_left_b/control/override"
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


@pytest.mark.parametrize(
    "filename",
    [
        "wuji_glove_teleop_compat.launch.py",
        "wuji_glove_dagger_compat.launch.py",
        "marvin_wuji_keyboard_teleop.launch.py",
    ],
)
@pytest.mark.parametrize("auto_enable", [None, "true", "false"])
@pytest.mark.parametrize(
    "models",
    [
        "wuji_hand",
        "wuji_hand_2",
        "wuji_hand,wuji_hand_2",
        "wuji_hand_2,wuji_hand",
    ],
)
def test_model_selection_wires_only_supported_driver_arguments(
    filename, auto_enable, models
):
    module = load_launch_module(
        LAUNCH_PATH.with_name(filename), "wuji_models_launch"
    )
    declarations = {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, _DeclareLaunchArgument)
    }
    assert declarations["hand2_auto_enable"].default_value == "true"
    context = _context(
        hand_side="left,right",
        hand_model=models,
        hand_serial_number="000123,000456" if models == "wuji_hand_2" else "",
        algo_topic="",
        hand2_auto_enable=(
            declarations["hand2_auto_enable"].default_value
            if auto_enable is None
            else auto_enable
        ),
    )
    actions = module._launch_instances(context)
    selected = models.split(",")
    if len(selected) == 1:
        selected *= 2
    for index, (side, model) in enumerate(
        zip(["left", "right"], selected, strict=True)
    ):
        hand, glove = actions[2 * index : 2 * index + 2]
        arguments = dict(hand.launch_arguments)
        package = hand.source.location.substitutions[0].package
        filename = hand.source.location.substitutions[-1]
        assert arguments["hand_name"] == f"hand_{side}"
        assert arguments["hand_side"] == side
        if model == "wuji_hand_2":
            assert package == "robo_orchard_wuji2_ros2"
            assert filename == "wuji_hand2.launch.py"
            assert {
                key: arguments[key]
                for key in (
                    "hand_name",
                    "hand_side",
                    "serial_number",
                )
            } == {
                "hand_name": f"hand_{side}",
                "hand_side": side,
                "serial_number": (
                    ["000123", "000456"][index]
                    if models == "wuji_hand_2"
                    else ""
                ),
            }
            assert arguments["auto_enable"].name == "hand2_auto_enable"
            assert arguments["auto_enable"].perform(context) == (
                "true" if auto_enable is None else auto_enable
            )
            for name in (
                "connect_timeout",
                "publish_joint_diagnostics",
                "publish_imu",
                "publish_fingertip_tactile",
                "reset_joint_position",
                "reset_duration_s",
                "reset_timeout_s",
            ):
                assert arguments[name].name == f"hand2_{name}"
            assert len(arguments) == 11
        else:
            assert package == "wujihand_bringup"
            assert filename == "wujihand.launch.py"
            assert arguments["publish_rate"].name == "hand_publish_rate"
            assert arguments["filter_cutoff_freq"].name == (
                "hand_filter_cutoff_freq"
            )
            assert arguments["diagnostics_rate"].name == (
                "hand_diagnostics_rate"
            )
            assert arguments["rviz"] == arguments["foxglove"] == "false"
            assert len(arguments) == 8
        target = dict(glove.launch_arguments)
        assert "auto_enable" not in target
        assert target["hand_model"] == model
        assert target["hand_side"] == side
        assert target["command_topic"] == f"/hand_{side}/control/override"


@pytest.mark.parametrize(
    "filename",
    [
        "wuji_glove_teleop_compat.launch.py",
        "wuji_glove_dagger_compat.launch.py",
        "marvin_wuji_keyboard_teleop.launch.py",
    ],
)
@pytest.mark.parametrize("serials", ["", "000123,", ",000456", " , "])
def test_all_multi_hand2_entries_reject_missing_serials_before_includes(
    filename: str, serials: str
) -> None:
    module = load_launch_module(LAUNCH_PATH.with_name(filename), filename)
    context = _context(
        hand_side="left,right",
        hand_model="wuji_hand_2",
        hand_serial_number=serials,
        algo_topic="",
    )
    with (
        patch.object(module, "build_wuji_pair_actions") as build_pair,
        pytest.raises(ValueError, match="required for every Hand2"),
    ):
        module._launch_instances(context)
    build_pair.assert_not_called()


@pytest.mark.parametrize(
    "filename",
    [
        "wuji_glove_teleop_compat.launch.py",
        "wuji_glove_dagger_compat.launch.py",
        "marvin_wuji_keyboard_teleop.launch.py",
    ],
)
def test_all_single_hand2_entries_keep_automatic_discovery(
    filename: str,
) -> None:
    module = load_launch_module(LAUNCH_PATH.with_name(filename), filename)
    actions = module._launch_instances(
        _context(hand_model="wuji_hand_2", algo_topic="")
    )
    assert len(actions) == 2
    assert dict(actions[0].launch_arguments)["serial_number"] == ""
