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
import runpy
from collections import Counter
from pathlib import Path
from xml.etree import ElementTree

import pytest
import setuptools
import yaml

PACKAGES = Path(__file__).resolve().parents[2]
TRACKING = PACKAGES / "robo_orchard_hexfellow_ros2"
TELEOP = PACKAGES / "robo_orchard_teleop_ros2"
CALLER = {"chassis_url": "192.0.2.10:8439", "lift_url": "192.0.2.11:8439"}

# Reuse the launch doubles for the Teleop entry points this package composes.
_stub_spec = importlib.util.spec_from_file_location(
    "teleop_launch_stubs", TELEOP / "tests/launch_stubs.py"
)
assert _stub_spec is not None and _stub_spec.loader is not None
launch_stubs = importlib.util.module_from_spec(_stub_spec)
_stub_spec.loader.exec_module(launch_stubs)


def value(item, context):
    if isinstance(item, launch_stubs.PathJoinSubstitution):
        return str(
            Path(*(value(part, context) for part in item.substitutions))
        )
    if isinstance(item, launch_stubs.FindPackageShare):
        return str(PACKAGES / item.package)
    if hasattr(item, "perform"):
        return item.perform(context)
    return item


def expand(path, caller):
    """Expand scoped repository includes, leaving vendor opaque."""
    module = launch_stubs.load_launch_module(path, path.stem)
    entities = module.generate_launch_description().entities
    context = dict(caller)
    nodes, vendors = [], []

    def visit(actions, inherited):
        current = dict(inherited)
        for action in actions:
            if isinstance(action, launch_stubs.DeclareLaunchArgument):
                if action.name not in current:
                    current[action.name] = value(action.default_value, current)
        for action in actions:
            if isinstance(action, launch_stubs.GroupAction):
                assert action.scoped
                visit(action.actions, current)
            elif isinstance(action, launch_stubs.IncludeLaunchDescription):
                if action.condition and not action.condition.evaluate(current):
                    continue
                location = Path(value(action.source.location, current))
                forwarded = {
                    name: value(argument, current)
                    for name, argument in action.launch_arguments
                }
                if "hex_device_ros_wrapper" in location.parts:
                    vendors.append((location, forwarded))
                else:
                    child_nodes, child_vendors = expand(
                        location, {**current, **forwarded}
                    )
                    nodes.extend(child_nodes)
                    vendors.extend(child_vendors)
            elif isinstance(action, launch_stubs.Node):
                options = action.kwargs
                if "condition" in options and not options[
                    "condition"
                ].evaluate(current):
                    continue
                nodes.append((options, dict(current)))

    visit(entities, context)
    return nodes, vendors


def test_standalone_wraps_official_driver_with_shared_frame_and_ros_clock():
    nodes, vendors = expand(
        TRACKING / "launch/hexfellow_tracking.launch.py",
        {**CALLER, "body_frame": "chassis_link", "chassis_is_kcp": "false"},
    )
    assert len(vendors) == 1
    path, arguments = vendors[0]
    assert path.parts[-3:] == ("launch", "ros2", "multi_bringup.launch.py")
    assert arguments == {
        **CALLER,
        "enable_chassis_bridge": "true",
        "enable_lift_bridge": "true",
        "chassis_is_kcp": "false",
        "lift_is_kcp": "true",
        "chassis_read_only": "false",
        "lift_read_only": "false",
        "frame_id": "chassis_link",
        "simple_mode": "true",
        "enable_ros_clock": "true",
    }
    assert len(nodes) == 1
    node, context = nodes[0]
    assert node["executable"] == "hexfellow_tracker"
    assert value(node["parameters"][0], context) == str(
        TRACKING / "config/tracker.yaml"
    )
    assert (
        value(node["parameters"][1]["body_frame"], context) == "chassis_link"
    )


@pytest.mark.parametrize("start_drivers", ["true", "false"])
def test_combination_starts_each_runtime_once_and_forwards_configuration(
    start_drivers,
):
    nodes, vendors = expand(
        TRACKING / "launch/hexfellow_piper_pico_tracking.launch.py",
        {
            **CALLER,
            "start_drivers": start_drivers,
            "left_can_port": "can_test_left",
            "right_can_port": "can_test_right",
            "tracker_config_file": "/example/tracker.yaml",
        },
    )
    assert len(vendors) == (1 if start_drivers == "true" else 0)
    assert Counter(node["executable"] for node, _ in nodes) == {
        "single_ctrl": 2,
        "pico_bridge": 1,
        "control_manager_node": 1,
        "piper_pico_vr_teleop": 1,
        "hexfellow_chassis_pico_vr_teleop": 1,
        "hexfellow_lift_pico_vr_teleop": 1,
        "hexfellow_tracker": 1,
    }
    for node, context in nodes:
        if node["executable"] == "hexfellow_tracker":
            assert (
                value(node["parameters"][0], context)
                == "/example/tracker.yaml"
            )
        elif node["executable"] == "control_manager_node":
            assert value(node["parameters"][0]["config_file"], context) == str(
                TRACKING / "config/control_manager.example.yaml"
            )
        elif node["executable"] == "single_ctrl":
            side = "left" if node["namespace"] == "/robot/left" else "right"
            assert (
                value(node["parameters"][0]["can_port"], context)
                == f"can_test_{side}"
            )


def test_existing_legacy_composition_remains_direct_driver_without_tracker():
    nodes, vendors = expand(
        TELEOP / "launch/hexfellow_piper_pico_teleop_compat.launch.py",
        {**CALLER, "control_manager_config_file": "/example/legacy.yaml"},
    )
    assert len(vendors) == 1
    assert "hexfellow_tracker" not in {node["executable"] for node, _ in nodes}
    assert Counter(node["executable"] for node, _ in nodes)["pico_bridge"] == 1


@pytest.mark.parametrize(
    "overrides,expected",
    [
        ({}, {"chassis": 0.5, "lift": 0.5}),
        (
            {
                "chassis_stale_timeout_s": "0.25",
                "lift_stale_timeout_s": "0.75",
            },
            {"chassis": 0.25, "lift": 0.75},
        ),
    ],
)
def test_tracking_composition_inherits_independent_stale_timeouts(
    overrides, expected
):
    nodes, vendors = expand(
        TRACKING / "launch/hexfellow_piper_pico_tracking.launch.py",
        {**CALLER, **overrides},
    )
    assert len(vendors) == 1
    actual = {}
    for node, context in nodes:
        for component in expected:
            if node["executable"] == f"hexfellow_{component}_pico_vr_teleop":
                parameter = node["parameters"][0]["stale_timeout_s"]
                assert parameter.value_type is float
                actual[component] = float(value(parameter.value, context))
    assert actual == expected


def test_combination_forwards_custom_manager_and_shared_vr():
    nodes, vendors = expand(
        TRACKING / "launch/hexfellow_piper_pico_tracking.launch.py",
        {
            **CALLER,
            "control_manager_config_file": "/example/custom.yaml",
            "vr_topic": "/shared/vr",
            "body_frame": "chassis_link",
        },
    )
    assert vendors[0][1]["frame_id"] == "chassis_link"
    for node, context in nodes:
        if node["executable"] == "control_manager_node":
            assert (
                value(node["parameters"][0]["config_file"], context)
                == "/example/custom.yaml"
            )
        elif node["executable"] in {"pico_bridge", "piper_pico_vr_teleop"}:
            assert (
                value(dict(node["remappings"])["vr_state"], context)
                == "/shared/vr"
            )
        elif (
            node["executable"].startswith("hexfellow_")
            and node["executable"] != "hexfellow_tracker"
        ):
            assert (
                value(node["parameters"][0]["vr_topic"], context)
                == "/shared/vr"
            )


def test_example_schema_and_separate_typed_chassis_routes():
    source = (
        PACKAGES
        / "robo_orchard_control_manager_ros2"
        / "robo_orchard_control_manager_ros2/config.py"
    )
    spec = importlib.util.spec_from_file_location(
        "tracking_manager_config", source
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = module.load_control_manager_config(
        TRACKING / "config/control_manager.example.yaml"
    )
    assert len(config.channels) == 5
    channels = {channel.name: channel for channel in config.channels}
    assert channels["chassis_pose"].msg_type == "nav_msgs/msg/Path"
    assert channels["chassis_pose"].override_topic is None
    assert channels["chassis_teleop"].msg_type == "geometry_msgs/msg/Twist"
    assert channels["chassis_teleop"].autonomous_topic is None
    tracker = yaml.safe_load((TRACKING / "config/tracker.yaml").read_text())[
        "hexfellow_tracker"
    ]["ros__parameters"]
    assert channels["chassis_pose"].output_topic == tracker["target_topic"]
    assert channels["chassis_teleop"].output_topic == tracker["teleop_topic"]
    assert tracker["output_topic"] == "/chassis/cmd_vel"
    assert tracker["odom_topic"] == "/chassis/odom"
    assert not any(
        channel.output_topic == tracker["output_topic"]
        for channel in config.channels
    )
    assert channels["lift"].msg_type == "sensor_msgs/msg/JointState"
    assert config.enable_services == [
        "/robot/left/enable_ctrl",
        "/robot/right/enable_ctrl",
    ]
    assert config.reset_services == [
        "/robot/left/reset_ctrl",
        "/robot/right/reset_ctrl",
    ]


def test_installation_includes_launches_configs_and_one_way_dependencies(
    monkeypatch,
):
    captured = {}
    monkeypatch.chdir(TRACKING)
    monkeypatch.setattr(
        setuptools, "setup", lambda **kwargs: captured.update(kwargs)
    )
    runpy.run_path(str(TRACKING / "setup.py"))
    data = dict(captured["data_files"])
    prefix = "share/robo_orchard_hexfellow_ros2"
    assert set(data[f"{prefix}/launch"]) == {
        "launch/hexfellow_tracking.launch.py",
        "launch/hexfellow_piper_pico_tracking.launch.py",
    }
    assert set(data[f"{prefix}/config"]) == {
        "config/tracker.yaml",
        "config/control_manager.example.yaml",
    }
    dependencies = {
        child.text
        for child in ElementTree.parse(TRACKING / "package.xml").findall(
            "exec_depend"
        )
    }
    assert {
        "launch",
        "launch_ros",
        "hex_bridge",
        "hex_device_msgs",
        "hex_device_ros_wrapper",
        "robo_orchard_teleop_ros2",
    } <= dependencies
    legacy = {
        child.text
        for child in ElementTree.parse(TELEOP / "package.xml").findall(
            "exec_depend"
        )
    }
    assert "robo_orchard_hexfellow_ros2" not in legacy
