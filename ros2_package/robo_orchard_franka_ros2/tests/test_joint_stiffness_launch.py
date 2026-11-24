# Project RoboOrchard
#
# Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
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
import os
import sys
import threading
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PACKAGE_PATH = Path(__file__).parents[1]


class _Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


def _load_launch_module():
    path = PACKAGE_PATH / "launch/joint_position_control.launch.py"
    spec = importlib.util.spec_from_file_location("stiffness_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def launch_description(monkeypatch):
    modules = {
        "launch": {
            "LaunchContext": object,
            "LaunchDescription": _Action,
        },
        "launch.actions": {
            name: type(name, (_Action,), {})
            for name in (
                "DeclareLaunchArgument",
                "OpaqueFunction",
                "RegisterEventHandler",
                "Shutdown",
            )
        },
        "launch.conditions": {"IfCondition": _Action},
        "launch.event_handlers": {"OnProcessExit": _Action},
        "launch.events": {},
        "launch.events.process": {"ProcessExited": object},
        "launch.substitutions": {"LaunchConfiguration": _Action},
        "launch_ros": {},
        "launch_ros.actions": {"Node": type("Node", (_Action,), {})},
        "launch_ros.parameter_descriptions": {"ParameterValue": _Action},
        "xacro": {},
        "ament_index_python": {},
        "ament_index_python.packages": {
            "get_package_share_directory": lambda name: f"/share/{name}"
        },
    }
    for name, members in modules.items():
        stub = types.ModuleType(name)
        stub.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, stub)
    module = _load_launch_module()
    return module, module.generate_launch_description().args[0]


def _registration_action(module, actions):
    return next(
        action
        for action in actions
        if isinstance(action, module.OpaqueFunction)
        and action.kwargs["function"].__name__ == "register_controller_start"
    )


def test_setup_uses_launch_gains_and_is_registered_before_start(
    launch_description,
):
    module, actions = launch_description
    argument = next(
        action
        for action in actions
        if isinstance(action, module.DeclareLaunchArgument)
        and action.args[0] == "joint_stiffness"
    )
    assert argument.kwargs["default_value"] == (
        "[3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0]"
    )
    setup = next(
        action
        for action in actions
        if isinstance(action, module.Node)
        and action.kwargs["executable"] == "configure_franka.py"
    )
    assert setup.kwargs["package"] == "robo_orchard_franka_ros2"
    stiffness = setup.kwargs["parameters"][0]["joint_stiffness"]
    assert stiffness.args[0].args == ("joint_stiffness",)
    assert stiffness.kwargs["value_type"] is str
    config = setup.kwargs["parameters"][0]["robot_config_file"]
    assert config.args[0].args == ("robot_config_file",)
    assert config.kwargs["value_type"] is str
    config_argument = next(
        action
        for action in actions
        if isinstance(action, module.DeclareLaunchArgument)
        and action.args[0] == "robot_config_file"
    )
    assert config_argument.kwargs["default_value"] == ""
    registration = _registration_action(module, actions)
    [handler] = registration.kwargs["function"](
        types.SimpleNamespace(launch_configurations={})
    )
    assert isinstance(handler, module.RegisterEventHandler)
    assert handler.args[0].kwargs["target_action"] is setup
    assert actions.index(registration) < actions.index(setup)
    assert [
        action.kwargs["arguments"][0]
        for action in actions
        if isinstance(action, module.Node)
        and action.kwargs["executable"] == "spawner"
    ] == ["joint_state_broadcaster"]


@pytest.mark.parametrize("returncode", [0, 1, -15])
@pytest.mark.parametrize("is_shutdown", [False, True])
@pytest.mark.parametrize(
    ("launch_configurations", "manager"),
    [
        ({}, "/controller_manager"),
        (
            {"ros_namespace": "/robots/arm_a"},
            "/robots/arm_a/controller_manager",
        ),
    ],
)
def test_only_success_starts_robot_state_and_position_controllers(
    launch_description, returncode, is_shutdown, launch_configurations, manager
):
    module, actions = launch_description
    registration = _registration_action(module, actions)
    [handler] = registration.kwargs["function"](
        types.SimpleNamespace(launch_configurations=launch_configurations)
    )
    callback = handler.args[0].kwargs["on_exit"]
    # The namespace of a scoped include is no longer set on process exit.
    event = types.SimpleNamespace(returncode=returncode)
    context = types.SimpleNamespace(
        launch_configurations={}, is_shutdown=is_shutdown
    )
    if is_shutdown:
        assert callback(event, context) == []
        return
    if returncode != 0:
        with pytest.raises(RuntimeError, match=f"exit code {returncode}"):
            callback(event, context)
        return

    result = callback(event, context)
    assert [
        action.kwargs["arguments"]
        for action in result
        if isinstance(action, module.Node)
        and action.kwargs["package"] == "controller_manager"
        and action.kwargs["executable"] == "spawner"
    ] == [
        ["franka_robot_state_broadcaster", "--controller-manager", manager],
        ["joint_position_command_controller", "--controller-manager", manager],
    ]
    assert len(result) == 2


@pytest.mark.parametrize(
    "scenario", ["integer_gains", "rejected", "yaml_wins"]
)
def test_configuration_round_trip_with_ros_launch(
    monkeypatch, tmp_path, scenario
):
    rclpy = pytest.importorskip("rclpy")
    franka_srv = pytest.importorskip("franka_msgs.srv")
    from launch import LaunchDescription, LaunchService
    from launch.actions import (
        GroupAction,
        OpaqueFunction,
        SetLaunchConfiguration,
    )
    from launch_ros.actions import Node, PushRosNamespace
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    module = _load_launch_module()
    namespace = f"/franka_setup_test_{os.getpid()}"
    accepted = scenario != "rejected"
    stiffness = "[1000, 1000, 1000, 1000, 1000, 1000, 1000]"
    config = ""
    expected_gains = [1000.0] * 7
    if scenario == "yaml_wins":
        stiffness = "[invalid"
        config_path = tmp_path / "robot.yaml"
        config_path.write_text(
            "joint_stiffness:\n"
            "  joint_stiffness: [500, 500.0, 500, 500, 500, 500, 500]\n",
            encoding="utf-8",
        )
        config = str(config_path)
        expected_gains = [500.0] * 7

    spawned = []

    def launch_node(*, package, executable, **kwargs):
        if executable == "configure_franka.py":
            return Node(
                executable=str(PACKAGE_PATH / "scripts" / executable), **kwargs
            )

        def record_spawner(_context):
            if package == "controller_manager" and executable == "spawner":
                spawned.append(kwargs["arguments"][0])

        return OpaqueFunction(function=record_spawner)

    monkeypatch.setattr(
        module, "get_package_share_directory", lambda _: "/unused"
    )
    monkeypatch.setattr(module, "_create_robot_nodes", lambda *_args: [])
    monkeypatch.setattr(module, "Node", launch_node)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")

    received = []

    def set_stiffness(request, response):
        received.append(list(request.joint_stiffness))
        response.success = accepted
        response.error = "" if accepted else "Rejected by test service"
        return response

    context = Context()
    rclpy.init(
        args=[],
        context=context,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    server = rclpy.create_node(
        "fake_franka", namespace=namespace, context=context
    )
    server.create_service(
        franka_srv.SetJointStiffness,
        "service_server/set_joint_stiffness",
        set_stiffness,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(server)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        launch_service = LaunchService()
        launch_service.include_launch_description(
            LaunchDescription(
                [
                    SetLaunchConfiguration("robot_ip", "127.0.0.1"),
                    SetLaunchConfiguration("joint_stiffness", stiffness),
                    SetLaunchConfiguration("robot_config_file", config),
                    GroupAction(
                        [
                            PushRosNamespace(namespace),
                            module.generate_launch_description(),
                        ]
                    ),
                ]
            )
        )
        returncode = launch_service.run()
    finally:
        executor.shutdown()
        thread.join()
        server.destroy_node()
        rclpy.shutdown(context=context)

    assert received == [expected_gains]
    if accepted:
        assert returncode == 0
        assert spawned == [
            "joint_state_broadcaster",
            "franka_robot_state_broadcaster",
            "joint_position_command_controller",
        ]
    else:
        assert returncode != 0
        assert spawned == ["joint_state_broadcaster"]


@pytest.mark.parametrize("namespace", ["/", "/robots/arm_a"])
def test_controller_config_applies_in_driver_namespace(namespace):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context

    context = Context()
    config = PACKAGE_PATH / "config/fr3_controllers.yaml"
    rclpy.init(
        args=["--ros-args", "--params-file", str(config)], context=context
    )
    try:
        for name, parameter in (
            ("controller_manager", "joint_position_command_controller.type"),
            ("franka_robot_state_broadcaster", "publish_rate"),
            ("joint_position_command_controller", "reset_joint_position"),
        ):
            node = rclpy.create_node(
                name,
                namespace=namespace,
                context=context,
                automatically_declare_parameters_from_overrides=True,
            )
            try:
                assert node.has_parameter(parameter)
            finally:
                node.destroy_node()
    finally:
        rclpy.shutdown(context=context)


def test_package_installs_setup_and_registers_python_tests():
    # ament --symlink-install executes the source, preserving its permissions.
    assert os.access(PACKAGE_PATH / "scripts/configure_franka.py", os.X_OK)
    manifest = ET.parse(PACKAGE_PATH / "package.xml").getroot()
    assert "rclpy" in {
        dependency.text for dependency in manifest.findall("exec_depend")
    }
    assert "python3-yaml" in {
        dependency.text for dependency in manifest.findall("exec_depend")
    }
    assert "ament_cmake_pytest" in {
        dependency.text for dependency in manifest.findall("test_depend")
    }
    cmake = (PACKAGE_PATH / "CMakeLists.txt").read_text()
    assert "install(PROGRAMS scripts/configure_franka.py" in cmake
    assert "tests/test_joint_stiffness_setup.py" in cmake
    assert "tests/test_joint_stiffness_launch.py" in cmake
