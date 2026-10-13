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

from collections import Counter
from pathlib import Path
from xml.etree import ElementTree

import pytest
from launch_stubs import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    Node,
    load_launch_module,
)


@pytest.fixture
def components():
    directory = Path(__file__).resolve().parents[1] / "launch"
    module = load_launch_module(
        directory / "hexfellow_piper_pico_teleop_compat.launch.py",
        "hexfellow_piper_compat_launch",
    )
    entities = module.generate_launch_description().entities
    required = {
        entity.name: entity.default_value
        for entity in entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.default_value is None
    }
    assert required == {
        "chassis_url": None,
        "lift_url": None,
        "control_manager_config_file": None,
    }
    groups = [entity for entity in entities if isinstance(entity, GroupAction)]
    assert len(groups) == 2
    result = {}
    for group in groups:
        assert isinstance(group, GroupAction)
        assert group.scoped
        assert len(group.actions) == 1
        include = group.actions[0]
        assert isinstance(include, IncludeLaunchDescription)
        location = include.source.location.substitutions
        package, folder = location[0].substitutions
        assert package.package == "robo_orchard_teleop_ros2"
        assert folder == "launch"
        child = load_launch_module(directory / location[1], location[1])
        description = child.generate_launch_description()
        defaults = {
            entity.name: entity.default_value
            for entity in description.entities
            if isinstance(entity, DeclareLaunchArgument)
        }
        result[location[1]] = (
            description.entities,
            defaults,
            dict(include.launch_arguments),
        )
    assert set(result) == {
        "piper_pico_teleop_compat.launch.py",
        "hexfellow_pico_teleop.launch.py",
    }
    return result


@pytest.mark.parametrize("vr_topic", ["/pico_bridge/vr_state", "/shared/vr"])
def test_composition_has_one_bridge_manager_and_all_adapters(
    components, vr_topic
):
    counts = Counter()
    caller = {
        "vr_topic": vr_topic,
        "chassis_url": "192.0.2.10:8439",
        "lift_url": "192.0.2.11:8439",
        "control_manager_config_file": "/example/control_manager.json",
    }
    for entities, defaults, overrides in components.values():
        forwarded = {
            name: value.perform({**defaults, **caller})
            if hasattr(value, "perform")
            else value
            for name, value in overrides.items()
        }
        context = {**defaults, **caller, **forwarded}
        for entity in entities:
            if not isinstance(entity, Node):
                continue
            node = entity.kwargs
            if "condition" in node and not node["condition"].evaluate(context):
                continue
            executable = node["executable"]
            counts[executable] += 1
            if executable in {"pico_bridge", "piper_pico_vr_teleop"}:
                assert node["namespace"] == "/pico_bridge"
                topic = dict(node["remappings"])["vr_state"]
                assert topic.perform(context) == vr_topic
            elif executable.startswith("hexfellow_"):
                topic = node["parameters"][0]["vr_topic"]
                assert topic.perform(context) == vr_topic
    assert counts == {
        "single_ctrl": 2,
        "pico_bridge": 1,
        "control_manager_node": 1,
        "piper_pico_vr_teleop": 1,
        "hexfellow_chassis_pico_vr_teleop": 1,
        "hexfellow_lift_pico_vr_teleop": 1,
    }


def test_combination_disables_only_hexfellow_shared_nodes(components):
    piper = components["piper_pico_teleop_compat.launch.py"][2]
    assert set(piper) == {"control_manager_config_file"}
    hexfellow = components["hexfellow_pico_teleop.launch.py"][2]
    assert {
        name: value
        for name, value in hexfellow.items()
        if name.startswith("start_")
    } == {
        "start_pico_bridge": "false",
        "start_control_manager": "false",
    }


def test_required_child_arguments_are_explicitly_forwarded(components):
    for _, defaults, overrides in components.values():
        required = {name for name, value in defaults.items() if value is None}
        assert required <= overrides.keys()
        for name in required:
            assert overrides[name].name == name


def test_driver_and_manager_arguments_reach_existing_nodes(components):
    caller = {
        "left_can_port": "can_test_left",
        "right_can_port": "can_test_right",
        "chassis_url": "192.0.2.10:8439",
        "lift_url": "192.0.2.11:8439",
        "control_manager_config_file": "/example/control_manager.json",
        "enable_chassis_bridge": "false",
        "enable_lift_bridge": "false",
        "chassis_is_kcp": "false",
        "lift_is_kcp": "false",
        "chassis_read_only": "true",
        "lift_read_only": "true",
    }
    drivers = {}
    managers = []
    vendor_launches = []
    for entities, defaults, overrides in components.values():
        assert defaults["control_manager_config_file"] is None
        forwarded = {
            name: value.perform({**defaults, **caller})
            if hasattr(value, "perform")
            else value
            for name, value in overrides.items()
        }
        context = {**defaults, **caller, **forwarded}
        for entity in entities:
            if isinstance(entity, IncludeLaunchDescription):
                location = entity.source.location.substitutions
                assert location[0].package == "hex_device_ros_wrapper"
                arguments = dict(entity.launch_arguments)
                assert arguments["simple_mode"] == "true"
                for name in caller.keys() - {
                    "left_can_port",
                    "right_can_port",
                    "control_manager_config_file",
                }:
                    assert arguments[name].perform(context) == caller[name]
                assert defaults["chassis_url"] is None
                assert defaults["lift_url"] is None
                vendor_launches.append(entity)
            elif isinstance(entity, Node):
                node = entity.kwargs
                if "condition" in node and not node["condition"].evaluate(
                    context
                ):
                    continue
                parameters = node.get("parameters", [{}])[0]
                if node["executable"] == "single_ctrl":
                    drivers[node["namespace"]] = parameters[
                        "can_port"
                    ].perform(context)
                elif node["executable"] == "control_manager_node":
                    assert node["namespace"] == "/robot/control"
                    managers.append(parameters["config_file"].perform(context))
    assert drivers == {
        "/robot/left": caller["left_can_port"],
        "/robot/right": caller["right_can_port"],
    }
    assert managers == [caller["control_manager_config_file"]]
    assert len(vendor_launches) == 1


@pytest.mark.parametrize(
    "component,name,default,override",
    [
        ("chassis", "max_linear_x", "0.2", "0.1"),
        ("chassis", "max_angular_z", "0.2", "0.15"),
        ("lift", "position_rate", "0.08", "0.04"),
    ],
)
def test_parent_speed_arguments_reach_hexfellow_nodes(
    components, component, name, default, override
):
    path = (
        Path(__file__).resolve().parents[1]
        / "launch/hexfellow_piper_pico_teleop_compat.launch.py"
    )
    module = load_launch_module(path, "combined_speed_launch")
    parent_defaults = {
        entity.name: entity.default_value
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    entities, child_defaults, forwarded = components[
        "hexfellow_pico_teleop.launch.py"
    ]
    assert parent_defaults[name] == child_defaults[name] == default
    assert forwarded[name].name == name
    node = next(
        entity.kwargs
        for entity in entities
        if isinstance(entity, Node)
        and entity.kwargs["executable"]
        == f"hexfellow_{component}_pico_vr_teleop"
    )
    parameter = node["parameters"][0][name]
    assert parameter.value_type is float
    for context, expected in (
        (parent_defaults, default),
        ({**parent_defaults, name: override}, override),
    ):
        value = forwarded[name].perform(context)
        assert (
            parameter.value.perform({**child_defaults, name: value})
            == expected
        )


def test_piper_driver_runtime_dependency_is_declared():
    manifest = Path(__file__).resolve().parents[1] / "package.xml"
    dependencies = {
        dependency.text
        for dependency in ElementTree.parse(manifest).findall("exec_depend")
    }
    assert "robo_orchard_piper_ros2" in dependencies
