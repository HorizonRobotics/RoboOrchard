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
import importlib.util
import sys
import types
from pathlib import Path

LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "marvin_pico_teleop.launch.py"
)


class _Entity:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.name = args[0] if args else kwargs.get("name")
        self.default_value = kwargs.get("default_value")


class _LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


class _LaunchConfiguration:
    def __init__(self, name):
        self.name = name


class _ParameterValue:
    def __init__(self, value, value_type=None):
        self.value = value
        self.value_type = value_type


def _load_module():
    modules = {
        "launch": types.ModuleType("launch"),
        "launch.actions": types.ModuleType("launch.actions"),
        "launch.launch_description_sources": types.ModuleType(
            "launch.launch_description_sources"
        ),
        "launch.substitutions": types.ModuleType("launch.substitutions"),
        "launch_ros": types.ModuleType("launch_ros"),
        "launch_ros.actions": types.ModuleType("launch_ros.actions"),
        "launch_ros.parameter_descriptions": types.ModuleType(
            "launch_ros.parameter_descriptions"
        ),
        "launch_ros.substitutions": types.ModuleType(
            "launch_ros.substitutions"
        ),
    }
    modules["launch"].LaunchDescription = _LaunchDescription
    modules["launch.actions"].DeclareLaunchArgument = _Entity
    modules["launch.actions"].IncludeLaunchDescription = _Entity
    modules[
        "launch.launch_description_sources"
    ].PythonLaunchDescriptionSource = _Entity
    modules["launch.substitutions"].LaunchConfiguration = _LaunchConfiguration
    modules["launch.substitutions"].PathJoinSubstitution = _Entity
    modules["launch_ros.actions"].Node = _Entity
    modules[
        "launch_ros.parameter_descriptions"
    ].ParameterValue = _ParameterValue
    modules["launch_ros.substitutions"].FindPackageShare = _Entity

    saved = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    try:
        spec = importlib.util.spec_from_file_location(
            "marvin_pico_teleop_launch", LAUNCH_PATH
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        for name, previous in saved.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


def test_marvin_launch_wires_keyboard_input_parameters():
    description = _load_module().generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if entity.name
        and "default_value" in entity.kwargs
        and "package" not in entity.kwargs
    }
    assert arguments["operator_input_source"].default_value == "pico"
    assert arguments["keyboard_control_side"].default_value == "both"

    teleop = next(
        entity
        for entity in description.entities
        if entity.kwargs.get("executable") == "marvin_pico_vr_teleop"
    )
    parameters = teleop.kwargs["parameters"][0]
    assert parameters["operator_input_source"].name == "operator_input_source"
    assert parameters["keyboard_control_side"].name == "keyboard_control_side"
    assert parameters["keyboard_activation_topic"].name == (
        "keyboard_activation_topic"
    )
    assert parameters["keyboard_reset_topic"].name == "keyboard_reset_topic"
    timeout = parameters["keyboard_activation_timeout_s"]
    assert timeout.value.name == "keyboard_activation_timeout_s"
    assert timeout.value_type is float
