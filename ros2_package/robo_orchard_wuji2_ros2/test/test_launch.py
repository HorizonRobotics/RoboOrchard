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

import importlib.util
from pathlib import Path
from typing import Any
from unittest.mock import patch

import pytest
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node

PACKAGE = Path(__file__).resolve().parents[1]


def _description() -> tuple[LaunchDescription, dict[str, Any]]:
    path = PACKAGE / "launch" / "wuji_hand2.launch.py"
    spec = importlib.util.spec_from_file_location("wuji_hand2_launch", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with (
        patch.object(
            module, "get_package_share_directory", return_value=str(PACKAGE)
        ),
        patch.object(module, "Node", wraps=Node) as node,
    ):
        description = module.generate_launch_description()
    return description, node.call_args.kwargs


def test_defaults_are_disabled_and_namespaced() -> None:
    """Resolve real Humble substitutions without starting any process."""
    description, node_arguments = _description()
    context = LaunchContext()
    arguments = {
        action.name: action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    for argument in arguments.values():
        argument.execute(context)
    node = next(
        action for action in description.entities if isinstance(action, Node)
    )
    assert node.node_package == "robo_orchard_wuji2_ros2"
    assert context.launch_configurations["hand_name"] == "hand_right"
    assert node_arguments["namespace"].perform(context) == "hand_right"
    overrides = node_arguments["parameters"][1]
    values = {key: value.evaluate(context) for key, value in overrides.items()}
    assert values == {
        "endpoint": "",
        "serial_number": "",
        "hand_side": "right",
        "connect_timeout": 10.0,
        "auto_enable": False,
        "publish_joint_diagnostics": True,
        "publish_imu": True,
        "publish_fingertip_tactile": False,
        "reset_joint_position": [0.0] * 20,
        "reset_duration_s": 2.0,
        "reset_timeout_s": 5.0,
        "reset_feedback_timeout_s": 0.2,
        "reset_position_tolerance": 0.2,
    }


def test_explicit_left_instance_and_parameter_types() -> None:
    """Keep serial strings typed and namespaces independent."""
    description, node_arguments = _description()
    context = LaunchContext()
    context.launch_configurations.update(
        hand_name="hand_left_second",
        hand_side="left",
        serial_number="000123",
        endpoint="usb://example",
        auto_enable="false",
        publish_imu="false",
        connect_timeout="2.5",
        reset_joint_position=str([0.1] * 20),
        reset_duration_s="1.5",
    )
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert node_arguments["namespace"].perform(context) == "hand_left_second"
    values = {
        key: value.evaluate(context)
        for key, value in node_arguments["parameters"][1].items()
    }
    assert values["serial_number"] == "000123"
    assert values["hand_side"] == "left"
    assert values["endpoint"] == "usb://example"
    assert values["connect_timeout"] == 2.5
    assert values["auto_enable"] is False
    assert values["publish_imu"] is False
    assert values["reset_joint_position"] == [0.1] * 20
    assert values["reset_duration_s"] == 1.5
    assert context.launch_configurations["hand_name"] == "hand_left_second"


def _dual_instances(
    overrides: dict[str, str],
) -> tuple[list[dict[str, Any]], dict[str, str]]:
    """Execute real include scopes, capturing nodes without executing them."""
    path = PACKAGE / "launch" / "wuji_hand2_dual.launch.py"
    spec = importlib.util.spec_from_file_location(
        "wuji_hand2_dual_launch", path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    node_arguments: dict[Node, dict[str, Any]] = {}
    instances: list[dict[str, Any]] = []

    def capture_node(**kwargs: Any) -> Node:
        node = Node(**kwargs)
        node_arguments[node] = kwargs
        return node

    def visit(entity: Any) -> None:
        if isinstance(entity, Node):
            kwargs = node_arguments[entity]
            instances.append(
                {
                    "namespace": kwargs["namespace"].perform(context),
                    **{
                        name: value.evaluate(context)
                        for name, value in kwargs["parameters"][1].items()
                    },
                }
            )
            return
        if isinstance(entity, LaunchDescription):
            for child in entity.entities:
                visit(child)
            return
        before = dict(context.launch_configurations)
        for child in entity.execute(context) or []:
            visit(child)
        if isinstance(entity, GroupAction):
            assert context.launch_configurations == before

    with (
        patch(
            "ament_index_python.packages.get_package_share_directory",
            return_value=str(PACKAGE),
        ),
        patch("launch_ros.actions.Node", side_effect=capture_node),
    ):
        spec.loader.exec_module(module)
        visit(module.generate_launch_description())
    return instances, dict(context.launch_configurations)


def test_dual_defaults_match_single_and_keep_scopes_isolated() -> None:
    """Compose disabled left/right nodes without leaking child selectors."""
    single, node_arguments = _description()
    context = LaunchContext()
    for action in single.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    expected = {
        name: value.evaluate(context)
        for name, value in node_arguments["parameters"][1].items()
    }
    instances, configurations = _dual_instances(
        {"left_serial_number": "000123", "right_endpoint": "usb://right"}
    )
    assert instances == [
        {
            **expected,
            "namespace": "hand_left",
            "hand_side": "left",
            "serial_number": "000123",
        },
        {
            **expected,
            "namespace": "hand_right",
            "hand_side": "right",
            "endpoint": "usb://right",
        },
    ]
    selectors = {"hand_name", "hand_side", "endpoint", "serial_number"}
    assert selectors.isdisjoint(configurations)
    shared = set(context.launch_configurations) - selectors
    assert set(configurations) == shared | {
        "left_endpoint",
        "right_endpoint",
        "left_serial_number",
        "right_serial_number",
    }
    for name in shared:
        assert configurations[name] == context.launch_configurations[name]


@pytest.mark.parametrize(
    ("overrides", "missing_side"),
    [
        ({}, "left"),
        ({"right_serial_number": "000456"}, "left"),
        ({"left_endpoint": "usb://left"}, "right"),
        (
            {
                "left_endpoint": " ",
                "left_serial_number": "\t",
                "right_endpoint": "usb://right",
            },
            "left",
        ),
    ],
)
def test_dual_requires_both_selectors_before_creating_nodes(
    overrides: dict[str, str], missing_side: str
) -> None:
    """Reject incomplete selectors before either hand can connect."""
    with (
        patch.object(
            Node, "__init__", side_effect=AssertionError("Node created")
        ) as node,
        pytest.raises(
            RuntimeError,
            match=f"requires {missing_side}_endpoint or "
            f"{missing_side}_serial_number",
        ),
    ):
        _dual_instances(overrides)
    node.assert_not_called()


def test_dual_selectors_and_all_shared_options_are_forwarded() -> None:
    """Keep leading-zero serials and side selectors independent."""
    options = {
        "connect_timeout": "3.5",
        "auto_enable": "true",
        "publish_joint_diagnostics": "false",
        "publish_imu": "false",
        "publish_fingertip_tactile": "true",
        "reset_duration_s": "1.5",
        "reset_timeout_s": "4.0",
        "reset_feedback_timeout_s": "0.1",
        "reset_position_tolerance": "0.03",
        "reset_joint_position": str([0.1] * 20),
    }
    overrides = {
        **options,
        "left_endpoint": "usb://left",
        "right_endpoint": "usb://right",
        "left_serial_number": "000123",
        "right_serial_number": "000456",
        "hand_name": "outer_hand",
        "hand_side": "outer_side",
        "endpoint": "outer_endpoint",
        "serial_number": "outer_serial",
    }
    instances, configurations = _dual_instances(overrides)
    expected_options = {
        "connect_timeout": 3.5,
        "auto_enable": True,
        "publish_joint_diagnostics": False,
        "publish_imu": False,
        "publish_fingertip_tactile": True,
        "reset_duration_s": 1.5,
        "reset_timeout_s": 4.0,
        "reset_feedback_timeout_s": 0.1,
        "reset_position_tolerance": 0.03,
        "reset_joint_position": [0.1] * 20,
    }
    assert instances == [
        {
            **expected_options,
            "namespace": "hand_left",
            "hand_side": "left",
            "endpoint": "usb://left",
            "serial_number": "000123",
        },
        {
            **expected_options,
            "namespace": "hand_right",
            "hand_side": "right",
            "endpoint": "usb://right",
            "serial_number": "000456",
        },
    ]
    assert configurations == overrides
