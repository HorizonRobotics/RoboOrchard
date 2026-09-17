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
from typing import List

PACKAGE_PATH = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_PATH / "launch" / "piper_pico_dagger_compat.launch.py"
SETUP_PATH = PACKAGE_PATH / "setup.py"


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


class _PathJoinSubstitution:
    def __init__(self, substitutions):
        self.substitutions = list(substitutions)


class _FindPackageShare:
    def __init__(self, package):
        self.package = package


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
    launch_substitutions.PathJoinSubstitution = _PathJoinSubstitution
    launch_ros_module = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = _Node
    launch_ros_substitutions = types.ModuleType("launch_ros.substitutions")
    launch_ros_substitutions.FindPackageShare = _FindPackageShare
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
            "launch_ros.substitutions",
            "launch_ros.parameter_descriptions",
        )
    }
    sys.modules["launch"] = launch_module
    sys.modules["launch.actions"] = launch_actions
    sys.modules["launch.substitutions"] = launch_substitutions
    sys.modules["launch_ros"] = launch_ros_module
    sys.modules["launch_ros.actions"] = launch_ros_actions
    sys.modules["launch_ros.substitutions"] = launch_ros_substitutions
    sys.modules["launch_ros.parameter_descriptions"] = launch_ros_param_desc

    spec = importlib.util.spec_from_file_location(
        "piper_pico_dagger_compat_launch", LAUNCH_PATH
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


def _nodes(description):
    return [
        entity for entity in description.entities if isinstance(entity, _Node)
    ]


def _node_by_name(description, name):
    return next(
        node for node in _nodes(description) if node.kwargs["name"] == name
    )


def _launch_arguments(description):
    return [
        entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    ]


def _launch_argument_by_name(description, name):
    return next(
        argument
        for argument in _launch_arguments(description)
        if argument.name == name
    )


def test_setup_does_not_register_legacy_control_owners():
    setup_source = SETUP_PATH.read_text()

    assert "take_over.node:main" not in setup_source
    assert "take_over.orchestrator" not in setup_source


def test_launch_declares_arguments_and_one_manager() -> None:
    module = _load_module()

    description = module.generate_launch_description()

    assert [argument.name for argument in _launch_arguments(description)] == [
        "left_joint_names",
        "right_joint_names",
        "left_algo_topic",
        "right_algo_topic",
        "left_slave_can_port",
        "right_slave_can_port",
        "left_base_frame_id",
        "left_ee_frame_id",
        "right_base_frame_id",
        "right_ee_frame_id",
        "publish_ee_tf",
        "enable_mit_control_mode",
        "control_manager_config_file",
        "urdf_path",
        "match_tolerance",
        "operator_input_source",
        "keyboard_control_side",
        "keyboard_activation_topic",
        "keyboard_reset_topic",
        "keyboard_activation_timeout_s",
        "left_reset_joint_position",
        "right_reset_joint_position",
    ]
    assert (
        _launch_argument_by_name(description, "match_tolerance").default_value
        == "0.1"
    )
    assert (
        _launch_argument_by_name(
            description, "operator_input_source"
        ).default_value
        == "pico"
    )
    assert (
        _launch_argument_by_name(
            description, "keyboard_control_side"
        ).default_value
        == "both"
    )
    assert (
        _launch_argument_by_name(
            description, "left_reset_joint_position"
        ).default_value
        == "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    )
    assert (
        _launch_argument_by_name(
            description, "right_reset_joint_position"
        ).default_value
        == "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    )
    assert [node.kwargs["name"] for node in _nodes(description)] == [
        "control_manager",
        "robot_left_controller",
        "robot_right_controller",
        "pico_bridge",
        "piper_pico_vr_teleop",
    ]


def test_launch_wires_one_manager_between_inputs_and_drivers():
    module = _load_module()
    description = module.generate_launch_description()

    manager = _node_by_name(description, "control_manager")
    assert manager.kwargs["package"] == "robo_orchard_control_manager_ros2"
    assert manager.kwargs["executable"] == "control_manager_node"
    assert manager.kwargs["remappings"][0][0] == "/left_algo_cmd"
    assert manager.kwargs["remappings"][0][1].name == "left_algo_topic"
    assert manager.kwargs["remappings"][1][0] == "/right_algo_cmd"
    assert manager.kwargs["remappings"][1][1].name == "right_algo_topic"
    assert not any(
        node.kwargs["executable"] in {"take_over", "vr_orchestrator"}
        for node in _nodes(description)
    )


def test_launch_wires_reset_joint_position_to_controllers():
    module = _load_module()
    description = module.generate_launch_description()

    left = _node_by_name(description, "robot_left_controller")
    right = _node_by_name(description, "robot_right_controller")

    left_reset = left.kwargs["parameters"][0]["reset_joint_position"]
    right_reset = right.kwargs["parameters"][0]["reset_joint_position"]

    # Each side must use ParameterValue with an explicit float-list type so
    # the string launch argument is coerced to a double[] parameter rather
    # than silently dropped against the node's typed declaration.
    assert left_reset.value.name == "left_reset_joint_position"
    assert left_reset.value_type == List[float]
    assert right_reset.value.name == "right_reset_joint_position"
    assert right_reset.value_type == List[float]


def test_launch_uses_manager_config_instead_of_per_side_orchestrators():
    module = _load_module()
    description = module.generate_launch_description()

    manager = _node_by_name(description, "control_manager")
    config_value = manager.kwargs["parameters"][0]["config_file"]
    assert config_value.name == "control_manager_config_file"


def test_launch_routes_vr_teleop_outputs_and_feedback_topics():
    module = _load_module()
    description = module.generate_launch_description()

    teleop = _node_by_name(description, "piper_pico_vr_teleop")

    assert teleop.kwargs["executable"] == "piper_pico_vr_teleop"
    assert teleop.kwargs["namespace"] == "/pico_bridge"
    assert teleop.kwargs["parameters"][0]["urdf_path"].name == "urdf_path"
    assert teleop.kwargs["parameters"][0]["ee_link_name"] == "link6"
    assert teleop.kwargs["parameters"][0]["base_link_name"] == "base_link"
    # All takeover/release mode switching now targets the global Manager.
    for service_param in (
        "left_takeover_service",
        "left_auto_service",
        "right_takeover_service",
        "right_auto_service",
    ):
        assert service_param not in teleop.kwargs["parameters"][0]
    assert teleop.kwargs["parameters"][0]["reset_service"] == (
        "/robot/control/reset"
    )
    assert (
        teleop.kwargs["parameters"][0]["match_tolerance"].name
        == "match_tolerance"
    )
    assert (
        teleop.kwargs["parameters"][0]["operator_input_source"].name
        == "operator_input_source"
    )
    assert (
        teleop.kwargs["parameters"][0]["keyboard_control_side"].name
        == "keyboard_control_side"
    )
    assert (
        teleop.kwargs["parameters"][0]["keyboard_activation_topic"].name
        == "keyboard_activation_topic"
    )
    assert (
        teleop.kwargs["parameters"][0]["keyboard_reset_topic"].name
        == "keyboard_reset_topic"
    )
    keyboard_timeout = teleop.kwargs["parameters"][0][
        "keyboard_activation_timeout_s"
    ]
    assert keyboard_timeout.value.name == "keyboard_activation_timeout_s"
    assert keyboard_timeout.value_type is float
    assert (
        "/robot/left/joint_cmd",
        "/pico_teleop/joint_left",
    ) in teleop.kwargs["remappings"]
    assert (
        "/robot/right/joint_cmd",
        "/pico_teleop/joint_right",
    ) in teleop.kwargs["remappings"]
    assert (
        "/robot/left/ee_pose",
        "/puppet/end_pose_left",
    ) in teleop.kwargs["remappings"]
    assert (
        "/robot/right/joint_state",
        "/puppet/joint_right",
    ) in teleop.kwargs["remappings"]
