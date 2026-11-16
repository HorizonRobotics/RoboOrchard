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

from pathlib import Path

import pytest
from launch_stubs import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    Node,
    OpaqueFunction,
    load_launch_module,
)

LAUNCH_DIRECTORY = Path(__file__).resolve().parents[1] / "launch"


class _Context(dict):
    def perform_substitution(self, substitution):
        return substitution.perform(self)


def _launch(filename, **overrides):
    module = load_launch_module(LAUNCH_DIRECTORY / filename, filename)
    entities = module.generate_launch_description().entities
    defaults = {
        entity.name: entity.default_value
        for entity in entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    context = _Context(defaults, **overrides)
    if filename == "franka_pico_teleop.launch.py":
        factory = next(
            entity for entity in entities if isinstance(entity, OpaqueFunction)
        )
        entities = [*entities, *factory.function(context)]
    return entities, defaults, context


@pytest.mark.parametrize(
    "filename,executables",
    [
        ("piper_pico_teleop_compat.launch.py", ["piper_pico_vr_teleop"]),
        ("piper_pico_dagger_compat.launch.py", ["piper_pico_vr_teleop"]),
        ("marvin_pico_teleop.launch.py", ["marvin_pico_vr_teleop"]),
        (
            "hexfellow_pico_teleop.launch.py",
            [
                "hexfellow_chassis_pico_vr_teleop",
                "hexfellow_lift_pico_vr_teleop",
            ],
        ),
    ],
)
@pytest.mark.parametrize("frequency", [None, "60.0", "90.0"])
def test_direct_launch_frequency(filename, executables, frequency):
    overrides = {} if frequency is None else {"teleop_frequency_hz": frequency}
    entities, defaults, context = _launch(filename, **overrides)
    assert defaults["teleop_frequency_hz"] == "30.0"
    nodes = {
        entity.kwargs["executable"]: entity.kwargs
        for entity in entities
        if isinstance(entity, Node)
    }
    for executable in executables:
        parameter = nodes[executable]["parameters"][0]["teleop_frequency_hz"]
        assert parameter.value_type is float
        assert parameter.value.perform(context) == (frequency or "30.0")


@pytest.mark.parametrize("frequency", [None, "60.0", "90.0"])
def test_franka_and_robotiq_preserve_independent_defaults(frequency):
    overrides = {"enable_robotiq": "true"}
    if frequency is not None:
        overrides.update(
            teleop_frequency_hz=frequency,
            robotiq_teleop_frequency_hz=frequency,
        )
    entities, defaults, context = _launch(
        "franka_pico_teleop.launch.py", **overrides
    )
    assert defaults["teleop_frequency_hz"] == "90.0"
    assert defaults["robotiq_teleop_frequency_hz"] == "30.0"
    for executable, default in [
        ("franka_pico_vr_teleop", "90.0"),
        ("teleop_engagement", "90.0"),
        ("robotiq_pico_vr_teleop", "30.0"),
    ]:
        node = next(
            entity.kwargs
            for entity in entities
            if isinstance(entity, Node)
            and entity.kwargs["executable"] == executable
        )
        parameter = node["parameters"][0]["teleop_frequency_hz"]
        assert parameter.value_type is float
        assert parameter.value.perform(context) == (frequency or default)


@pytest.mark.parametrize(
    "filename,children",
    [
        (
            "hexfellow_piper_pico_teleop_compat.launch.py",
            {
                "piper_pico_teleop_compat.launch.py",
                "hexfellow_pico_teleop.launch.py",
            },
        ),
        (
            "marvin_wuji_keyboard_teleop.launch.py",
            {"marvin_pico_teleop.launch.py"},
        ),
    ],
)
@pytest.mark.parametrize("frequency", [None, "60.0", "90.0"])
def test_combined_launch_forwards_frequency(filename, children, frequency):
    overrides = {} if frequency is None else {"teleop_frequency_hz": frequency}
    entities, defaults, context = _launch(filename, **overrides)
    assert defaults["teleop_frequency_hz"] == "30.0"
    actions = []
    for entity in entities:
        actions.extend(
            entity.actions if isinstance(entity, GroupAction) else [entity]
        )
    checked = set()
    for action in actions:
        if not isinstance(action, IncludeLaunchDescription):
            continue
        child = action.source.location.substitutions[-1]
        if child not in children:
            continue
        arguments = dict(action.launch_arguments)
        forwarded = arguments["teleop_frequency_hz"].perform(context)
        assert forwarded == (frequency or "30.0")
        child_entities, _, child_context = _launch(
            child, teleop_frequency_hz=forwarded
        )
        for node in child_entities:
            if not isinstance(node, Node):
                continue
            for parameters in node.kwargs.get("parameters", []):
                if (
                    isinstance(parameters, dict)
                    and "teleop_frequency_hz" in parameters
                ):
                    assert (
                        parameters["teleop_frequency_hz"].value.perform(
                            child_context
                        )
                        == forwarded
                    )
        checked.add(child)
    assert checked == children
