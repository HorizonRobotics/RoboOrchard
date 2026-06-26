# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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
from typing import List

LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "piper_pico_teleop_compat.launch.py"
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


class _ParameterValue:
    def __init__(self, value, value_type=None):
        self.value = value
        self.value_type = value_type


class _Node:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def _load_module():
    launch_module = types.ModuleType("launch")
    launch_module.LaunchDescription = _LaunchDescription
    launch_actions = types.ModuleType("launch.actions")
    launch_actions.DeclareLaunchArgument = _DeclareLaunchArgument
    launch_substitutions = types.ModuleType("launch.substitutions")
    launch_substitutions.LaunchConfiguration = _LaunchConfiguration
    launch_ros_module = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = _Node
    launch_ros_param_desc = types.ModuleType(
        "launch_ros.parameter_descriptions"
    )
    launch_ros_param_desc.ParameterValue = _ParameterValue

    saved_modules = {
        name: sys.modules.get(name)
        for name in (
            "launch",
            "launch.actions",
            "launch.substitutions",
            "launch_ros",
            "launch_ros.actions",
            "launch_ros.parameter_descriptions",
        )
    }
    sys.modules["launch"] = launch_module
    sys.modules["launch.actions"] = launch_actions
    sys.modules["launch.substitutions"] = launch_substitutions
    sys.modules["launch_ros"] = launch_ros_module
    sys.modules["launch_ros.actions"] = launch_ros_actions
    sys.modules["launch_ros.parameter_descriptions"] = launch_ros_param_desc

    spec = importlib.util.spec_from_file_location(
        "piper_pico_teleop_compat_launch", LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    for name, saved in saved_modules.items():
        if saved is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = saved

    return module


def test_launch_starts_bridge_teleop_and_both_arm_controllers():
    module = _load_module()

    description = module.generate_launch_description()

    nodes = [
        entity for entity in description.entities if isinstance(entity, _Node)
    ]
    names = [node.kwargs["name"] for node in nodes]

    assert "pico_bridge" in names
    assert "piper_pico_vr_teleop" in names
    assert "robot_right_single_controller" in names
    assert "robot_left_single_controller" in names


def test_launch_wires_reset_joint_position_to_controllers():
    module = _load_module()

    description = module.generate_launch_description()
    nodes = [
        entity for entity in description.entities if isinstance(entity, _Node)
    ]
    left = next(
        node
        for node in nodes
        if node.kwargs["name"] == "robot_left_single_controller"
    )
    right = next(
        node
        for node in nodes
        if node.kwargs["name"] == "robot_right_single_controller"
    )

    left_reset = left.kwargs["parameters"][0]["reset_joint_position"]
    right_reset = right.kwargs["parameters"][0]["reset_joint_position"]

    assert left_reset.value.name == "left_reset_joint_position"
    assert left_reset.value_type == List[float]
    assert right_reset.value.name == "right_reset_joint_position"
    assert right_reset.value_type == List[float]


def test_launch_routes_pico_joint_outputs_to_algo_topics():
    module = _load_module()

    description = module.generate_launch_description()
    nodes = [
        entity for entity in description.entities if isinstance(entity, _Node)
    ]
    teleop_node = next(
        node for node in nodes if node.kwargs["name"] == "piper_pico_vr_teleop"
    )
    right_controller = next(
        node
        for node in nodes
        if node.kwargs["name"] == "robot_right_single_controller"
    )
    left_controller = next(
        node
        for node in nodes
        if node.kwargs["name"] == "robot_left_single_controller"
    )

    assert (
        "/robot/left/joint_cmd",
        "/left_algo_cmd",
    ) in teleop_node.kwargs["remappings"]
    assert (
        "/robot/right/joint_cmd",
        "/right_algo_cmd",
    ) in teleop_node.kwargs["remappings"]
    assert (
        "/robot/right/joint_cmd",
        "/right_algo_cmd",
    ) in right_controller.kwargs["remappings"]
    assert (
        "/robot/left/joint_cmd",
        "/left_algo_cmd",
    ) in left_controller.kwargs["remappings"]
    parameters = teleop_node.kwargs["parameters"]
    assert parameters[0]["ee_link_name"] == "link6"
    assert parameters[0]["left_reset_service"] == "/robot/left/reset_ctrl"
    assert parameters[0]["right_reset_service"] == "/robot/right/reset_ctrl"


def test_launch_routes_arm_feedback_to_puppet_topics():
    module = _load_module()

    description = module.generate_launch_description()
    nodes = [
        entity for entity in description.entities if isinstance(entity, _Node)
    ]
    teleop_node = next(
        node for node in nodes if node.kwargs["name"] == "piper_pico_vr_teleop"
    )
    right_controller = next(
        node
        for node in nodes
        if node.kwargs["name"] == "robot_right_single_controller"
    )
    left_controller = next(
        node
        for node in nodes
        if node.kwargs["name"] == "robot_left_single_controller"
    )

    left_remappings = left_controller.kwargs["remappings"]
    right_remappings = right_controller.kwargs["remappings"]
    teleop_remappings = teleop_node.kwargs["remappings"]

    assert (
        "/robot/left/status",
        "/puppet/status_left",
    ) in left_remappings
    assert (
        "/robot/left/ee_pose",
        "/puppet/end_pose_left",
    ) in left_remappings
    assert (
        "/robot/left/joint_state",
        "/puppet/joint_left",
    ) in left_remappings
    assert (
        "/robot/right/status",
        "/puppet/status_right",
    ) in right_remappings
    assert (
        "/robot/right/ee_pose",
        "/puppet/end_pose_right",
    ) in right_remappings
    assert (
        "/robot/right/joint_state",
        "/puppet/joint_right",
    ) in right_remappings

    assert (
        "/robot/left/ee_pose",
        "/puppet/end_pose_left",
    ) in teleop_remappings
    assert (
        "/robot/left/joint_state",
        "/puppet/joint_left",
    ) in teleop_remappings
    assert (
        "/robot/right/ee_pose",
        "/puppet/end_pose_right",
    ) in teleop_remappings
    assert (
        "/robot/right/joint_state",
        "/puppet/joint_right",
    ) in teleop_remappings
