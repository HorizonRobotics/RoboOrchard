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

import ast
from pathlib import Path
from xml.etree import ElementTree

import pytest
from launch_stubs import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Node,
    load_launch_module,
)


@pytest.fixture
def launch():
    path = (
        Path(__file__).resolve().parents[1]
        / "launch/hexfellow_pico_teleop.launch.py"
    )
    module = load_launch_module(path, "hexfellow_pico_launch")
    return module.generate_launch_description()


def nodes_by_executable(launch):
    return {
        entity.kwargs["executable"]: entity.kwargs
        for entity in launch.entities
        if isinstance(entity, Node)
    }


def arguments(launch):
    return {
        entity.name: entity.default_value
        for entity in launch.entities
        if isinstance(entity, DeclareLaunchArgument)
    }


@pytest.fixture
def driver_launch(launch):
    includes = [
        entity
        for entity in launch.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert len(includes) == 1
    return includes[0]


def test_includes_vendor_multi_bringup_from_installed_package(driver_launch):
    location = driver_launch.source.location.substitutions
    assert location[0].package == "hex_device_ros_wrapper"
    assert location[1:] == ["launch", "ros2", "multi_bringup.launch.py"]
    assert dict(driver_launch.launch_arguments)["simple_mode"] == "true"


def test_device_addresses_are_required_and_forwarded(launch, driver_launch):
    defaults = arguments(launch)
    forwarded = dict(driver_launch.launch_arguments)
    addresses = {
        "chassis_url": "192.0.2.10:8439",
        "lift_url": "192.0.2.11:8439",
    }
    for name, value in addresses.items():
        assert name in defaults
        assert defaults[name] is None
        assert forwarded[name].perform(addresses) == value


@pytest.mark.parametrize(
    "name,default,override",
    [
        ("enable_chassis_bridge", "true", "false"),
        ("enable_lift_bridge", "true", "false"),
        ("chassis_is_kcp", "true", "false"),
        ("lift_is_kcp", "true", "false"),
        ("chassis_read_only", "false", "true"),
        ("lift_read_only", "false", "true"),
    ],
)
def test_connection_options_forward_defaults_and_overrides(
    launch, driver_launch, name, default, override
):
    defaults = arguments(launch)
    parameter = dict(driver_launch.launch_arguments)[name]
    assert parameter.perform(defaults) == default
    assert parameter.perform({name: override}) == override


def test_vendor_runtime_dependencies_are_declared():
    manifest = Path(__file__).resolve().parents[1] / "package.xml"
    dependencies = {
        dependency.text
        for dependency in ElementTree.parse(manifest).findall("exec_depend")
    }
    assert {
        "hex_bridge",
        "hex_device_msgs",
        "hex_device_ros_wrapper",
    } <= dependencies


def test_launch_starts_only_one_bridge_manager_and_both_adapters(launch):
    nodes = nodes_by_executable(launch)
    assert set(nodes) == {
        "pico_bridge",
        "hexfellow_chassis_pico_vr_teleop",
        "hexfellow_lift_pico_vr_teleop",
        "control_manager_node",
    }
    assert (
        len([item for item in launch.entities if isinstance(item, Node)]) == 4
    )
    assert nodes["control_manager_node"]["namespace"] == "/robot/control"
    for executable in nodes.keys() - {"control_manager_node"}:
        assert nodes[executable]["namespace"] == "/pico_bridge"


def test_manager_requires_caller_configuration_without_default(launch):
    defaults = arguments(launch)
    assert "control_manager_config_file" in defaults
    assert defaults["control_manager_config_file"] is None
    manager = nodes_by_executable(launch)["control_manager_node"]
    config = manager["parameters"][0]["config_file"]
    assert config.name == "control_manager_config_file"


@pytest.mark.parametrize(
    "executable,argument",
    [
        ("pico_bridge", "start_pico_bridge"),
        ("control_manager_node", "start_control_manager"),
    ],
)
def test_shared_nodes_can_be_disabled(launch, executable, argument):
    defaults = arguments(launch)
    condition = nodes_by_executable(launch)[executable]["condition"]
    assert defaults[argument] == "true"
    assert condition.evaluate(defaults)
    assert not condition.evaluate({argument: "false"})
    for adapter in (
        "hexfellow_chassis_pico_vr_teleop",
        "hexfellow_lift_pico_vr_teleop",
    ):
        assert "condition" not in nodes_by_executable(launch)[adapter]


def test_vr_topic_override_rewires_both_adapters_and_bridge(launch):
    nodes = nodes_by_executable(launch)
    defaults = arguments(launch)
    assert defaults["vr_topic"] == "/pico_bridge/vr_state"
    bridge_source, bridge_target = nodes["pico_bridge"]["remappings"][0]
    assert bridge_source == "vr_state"
    context = {"vr_topic": "/shared/vr_state"}
    assert bridge_target.perform(context) == "/shared/vr_state"
    for executable in (
        "hexfellow_chassis_pico_vr_teleop",
        "hexfellow_lift_pico_vr_teleop",
    ):
        parameter = nodes[executable]["parameters"][0]["vr_topic"]
        assert parameter.perform(context) == "/shared/vr_state"


def test_candidate_topics_and_feedback_are_configurable(launch):
    defaults = arguments(launch)
    nodes = nodes_by_executable(launch)
    chassis = nodes["hexfellow_chassis_pico_vr_teleop"]["parameters"][0]
    lift = nodes["hexfellow_lift_pico_vr_teleop"]["parameters"][0]
    assert chassis["cmd_vel_topic"].perform(defaults) == (
        "/hexfellow_teleop/cmd_vel"
    )
    assert lift["joint_cmd_topic"].perform(defaults) == (
        "/hexfellow_teleop/lift_joint_cmd"
    )
    assert lift["state_topic"].perform(defaults) == "/lift/motor_states"
    custom = {
        "chassis_override_topic": "/override/chassis",
        "lift_override_topic": "/override/lift",
        "lift_state_topic": "/measured/lift",
    }
    assert chassis["cmd_vel_topic"].perform(custom) == "/override/chassis"
    assert lift["joint_cmd_topic"].perform(custom) == "/override/lift"
    assert lift["state_topic"].perform(custom) == "/measured/lift"


def test_publish_frequency_is_explicitly_float(launch):
    nodes = nodes_by_executable(launch)
    for executable in (
        "hexfellow_chassis_pico_vr_teleop",
        "hexfellow_lift_pico_vr_teleop",
    ):
        frequency = nodes[executable]["parameters"][0]["publish_hz"]
        assert frequency.value_type is float
        assert frequency.value.name == "publish_hz"


@pytest.mark.parametrize(
    "component,name,default,override",
    [
        ("chassis", "max_linear_x", "0.2", "0.1"),
        ("chassis", "max_angular_z", "0.2", "0.15"),
        ("lift", "position_rate", "0.08", "0.04"),
    ],
)
def test_speed_parameters_forward_defaults_and_overrides_as_float(
    launch, component, name, default, override
):
    defaults = arguments(launch)
    assert defaults[name] == default
    node = nodes_by_executable(launch)[f"hexfellow_{component}_pico_vr_teleop"]
    parameter = node["parameters"][0][name]
    assert parameter.value_type is float
    assert parameter.value.name == name
    assert parameter.value.perform(defaults) == default
    assert parameter.value.perform({name: override}) == override


def test_both_adapters_have_packaged_entrypoints():
    package_root = Path(__file__).resolve().parents[1]
    tree = ast.parse((package_root / "setup.py").read_text())
    setup_call = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "setup"
    )
    entrypoints = next(
        ast.literal_eval(keyword.value)
        for keyword in setup_call.keywords
        if keyword.arg == "entry_points"
    )["console_scripts"]
    for component in ("chassis", "lift"):
        module = (
            f"robo_orchard_teleop_ros2.robot.hexfellow.{component}.pico_vr"
        )
        assert (
            f"hexfellow_{component}_pico_vr_teleop = {module}:main"
            in entrypoints
        )
        assert (package_root / (module.replace(".", "/") + ".py")).is_file()
