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
    Path(__file__).resolve().parents[1] / "launch" / "wuji_glove.launch.py"
)


class _LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


class _DeclareLaunchArgument:
    def __init__(self, name, default_value=None, description=""):
        self.name = name
        self.default_value = default_value
        self.description = description


class _LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class _OpaqueFunction:
    def __init__(self, function):
        self.function = function


class _SetLaunchConfiguration:
    def __init__(self, name, value):
        self.name = name
        self.value = value


class _Node:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class _ParameterValue:
    def __init__(self, value, value_type=None):
        self.value = value
        self.value_type = value_type


def _load_module():
    modules = {
        "ament_index_python": types.ModuleType("ament_index_python"),
        "ament_index_python.packages": types.ModuleType(
            "ament_index_python.packages"
        ),
        "launch": types.ModuleType("launch"),
        "launch.actions": types.ModuleType("launch.actions"),
        "launch.substitutions": types.ModuleType("launch.substitutions"),
        "launch_ros": types.ModuleType("launch_ros"),
        "launch_ros.actions": types.ModuleType("launch_ros.actions"),
        "launch_ros.parameter_descriptions": types.ModuleType(
            "launch_ros.parameter_descriptions"
        ),
    }
    modules["ament_index_python.packages"].get_package_share_directory = (
        lambda package: f"/share/{package}"
    )
    modules["launch"].LaunchDescription = _LaunchDescription
    modules["launch.actions"].DeclareLaunchArgument = _DeclareLaunchArgument
    modules["launch.actions"].OpaqueFunction = _OpaqueFunction
    modules["launch.actions"].SetLaunchConfiguration = _SetLaunchConfiguration
    modules["launch.substitutions"].LaunchConfiguration = _LaunchConfiguration
    modules["launch_ros.actions"].Node = _Node
    modules[
        "launch_ros.parameter_descriptions"
    ].ParameterValue = _ParameterValue

    saved = {name: sys.modules.get(name) for name in modules}
    sys.modules.update(modules)
    try:
        spec = importlib.util.spec_from_file_location(
            "wuji_glove_launch", LAUNCH_PATH
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


def _resolve_side_defaults(description, values):
    resolved = dict(values)
    resolver = next(
        entity
        for entity in description.entities
        if isinstance(entity, _OpaqueFunction)
    )
    for action in resolver.function(resolved):
        resolved[action.name] = action.value
    return resolved


def test_launch_starts_driver_and_retarget_nodes():
    description = _load_module().generate_launch_description()
    nodes = [
        entity for entity in description.entities if isinstance(entity, _Node)
    ]

    assert [node.kwargs["executable"] for node in nodes] == [
        "wuji_glove_driver_node",
        "wuji_glove_retarget_node",
    ]
    assert all(
        node.kwargs["namespace"].name == "glove_namespace" for node in nodes
    )


def test_launch_forwards_device_selection_and_command_topic():
    description = _load_module().generate_launch_description()
    nodes = [
        entity for entity in description.entities if isinstance(entity, _Node)
    ]
    driver, retarget = nodes

    overrides = driver.kwargs["parameters"][1]
    assert overrides["serial_number"].value.name == "glove_serial_number"
    assert overrides["serial_number"].value_type is str
    assert overrides["hand_side"].value.name == "hand_side"
    assert overrides["frame_prefix"].value.name == "frame_prefix"
    assert overrides["frame_prefix"].value_type is str
    assert overrides["sdk_user"].value.name == "sdk_user"
    assert overrides["sdk_user"].value_type is str
    assert overrides["hand_model_path"].value.name == "hand_model_path"
    assert overrides["hand_model_path"].value_type is str
    assert overrides["stream_profile"].value.name == "stream_profile"
    assert overrides["stream_profile"].value_type is str
    source_topic, destination_topic = retarget.kwargs["remappings"][0]
    assert source_topic == "retargeted_joint_commands"
    assert destination_topic.name == "command_topic"

    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    }
    assert (
        arguments["command_topic"].default_value == "retargeted_joint_commands"
    )
    assert arguments["stream_profile"].default_value == "configured"


def test_launch_derives_empty_names_from_hand_side():
    description = _load_module().generate_launch_description()
    arguments = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    }
    assert arguments["glove_namespace"].default_value == ""
    assert arguments["device_name"].default_value == ""
    assert arguments["frame_prefix"].default_value == ""

    left = _resolve_side_defaults(
        description,
        {"hand_side": "left", "glove_namespace": "", "device_name": ""},
    )
    assert left["glove_namespace"] == "/wuji_glove/left"
    assert left["device_name"] == "wuji_glove_left"

    right = _resolve_side_defaults(
        description,
        {"hand_side": "right", "glove_namespace": "", "device_name": ""},
    )
    assert right["glove_namespace"] == "/wuji_glove/right"
    assert right["device_name"] == "wuji_glove_right"

    custom = _resolve_side_defaults(
        description,
        {
            "hand_side": "right",
            "glove_namespace": "/custom/glove",
            "device_name": "custom_device",
        },
    )
    assert custom["glove_namespace"] == "/custom/glove"
    assert custom["device_name"] == "custom_device"
