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
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "franka_pico_teleop.launch.py"
)


class _LaunchDescription:
    def __init__(self, entities):
        self.entities = list(entities)


class _DeclareLaunchArgument:
    def __init__(self, name, default_value=None, description=""):
        self.name = name
        self.default_value = default_value
        self.description = description


class _IncludeLaunchDescription:
    def __init__(self, launch_description_source, launch_arguments=None):
        self.launch_description_source = launch_description_source
        self.launch_arguments = dict(launch_arguments or [])


class _PythonLaunchDescriptionSource:
    def __init__(self, launch_file_path):
        self.launch_file_path = launch_file_path


class _LaunchConfiguration:
    def __init__(self, name):
        self.name = name


class _PathJoinSubstitution:
    def __init__(self, substitutions):
        self.substitutions = list(substitutions)


class _ParameterValue:
    def __init__(self, value, value_type=None):
        self.value = value
        self.value_type = value_type


class _FindPackageShare:
    def __init__(self, package):
        self.package = package


class _Node:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


class _OpaqueFunction:
    def __init__(self, function, args=()):
        self.function = function
        self.args = args


class _IfCondition:
    def __init__(self, predicate):
        self.predicate = predicate

    def evaluate(self, context):
        return context.perform_substitution(self.predicate) == "true"


def _load_module(path=LAUNCH_PATH):
    launch_module = types.ModuleType("launch")
    launch_module.LaunchDescription = _LaunchDescription
    launch_module.LaunchContext = object
    launch_actions = types.ModuleType("launch.actions")
    launch_actions.DeclareLaunchArgument = _DeclareLaunchArgument
    launch_actions.IncludeLaunchDescription = _IncludeLaunchDescription
    launch_actions.OpaqueFunction = _OpaqueFunction
    launch_actions.Shutdown = object
    launch_conditions = types.ModuleType("launch.conditions")
    launch_conditions.IfCondition = _IfCondition
    launch_description_sources = types.ModuleType(
        "launch.launch_description_sources"
    )
    launch_description_sources.PythonLaunchDescriptionSource = (
        _PythonLaunchDescriptionSource
    )
    launch_substitutions = types.ModuleType("launch.substitutions")
    launch_substitutions.LaunchConfiguration = _LaunchConfiguration
    launch_substitutions.PathJoinSubstitution = _PathJoinSubstitution
    launch_ros_module = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = _Node
    launch_ros_param_desc = types.ModuleType(
        "launch_ros.parameter_descriptions"
    )
    launch_ros_param_desc.ParameterValue = _ParameterValue
    launch_ros_substitutions = types.ModuleType("launch_ros.substitutions")
    launch_ros_substitutions.FindPackageShare = _FindPackageShare
    xacro_module = types.ModuleType("xacro")
    ament_index = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = lambda name: f"/share/{name}"

    module_names = (
        "launch",
        "launch.actions",
        "launch.conditions",
        "launch.launch_description_sources",
        "launch.substitutions",
        "launch_ros",
        "launch_ros.actions",
        "launch_ros.parameter_descriptions",
        "launch_ros.substitutions",
        "xacro",
        "ament_index_python",
        "ament_index_python.packages",
    )
    saved_modules = {name: sys.modules.get(name) for name in module_names}
    sys.modules["launch"] = launch_module
    sys.modules["launch.actions"] = launch_actions
    sys.modules["launch.conditions"] = launch_conditions
    sys.modules["launch.launch_description_sources"] = (
        launch_description_sources
    )
    sys.modules["launch.substitutions"] = launch_substitutions
    sys.modules["launch_ros"] = launch_ros_module
    sys.modules["launch_ros.actions"] = launch_ros_actions
    sys.modules["launch_ros.parameter_descriptions"] = launch_ros_param_desc
    sys.modules["launch_ros.substitutions"] = launch_ros_substitutions
    sys.modules["xacro"] = xacro_module
    sys.modules["ament_index_python"] = ament_index
    sys.modules["ament_index_python.packages"] = ament_packages

    try:
        spec = importlib.util.spec_from_file_location(
            "franka_pico_teleop_launch",
            path,
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module
    finally:
        for name, saved_module in saved_modules.items():
            if saved_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = saved_module


def _description(module=None, **overrides):
    module = module or _load_module()
    description = module.generate_launch_description()
    values = {
        entity.name: entity.default_value
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    }
    values.update(overrides)
    context = types.SimpleNamespace(
        perform_substitution=lambda substitution: values[substitution.name]
    )
    factory = description.entities.pop()
    assert isinstance(factory, _OpaqueFunction)
    description.entities.extend(factory.function(context, *factory.args))
    return description


def test_launch_starts_driver_bridge_teleop_and_one_manager():
    description = _description()

    nodes = [
        entity for entity in description.entities if isinstance(entity, _Node)
    ]
    assert [node.kwargs["name"] for node in nodes] == [
        "pico_bridge",
        "franka_pico_vr_teleop",
        "control_manager",
    ]

    driver = next(
        entity
        for entity in description.entities
        if isinstance(entity, _IncludeLaunchDescription)
    )
    driver_path = driver.launch_description_source.launch_file_path
    assert isinstance(driver_path, _PathJoinSubstitution)
    assert driver_path.substitutions[0].package == "robo_orchard_franka_ros2"
    assert driver_path.substitutions[-1] == "joint_position_control.launch.py"
    assert driver.launch_arguments["robot_ip"].name == "robot_ip"
    assert driver.launch_arguments["arm_id"] == "fr3"
    assert driver.launch_arguments["use_rviz"].name == "use_rviz"
    assert driver.launch_arguments["load_gripper"].name == "load_gripper"
    gripper_argument = next(
        entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
        and entity.name == "load_gripper"
    )
    assert gripper_argument.default_value == "false"


def test_launch_wires_arm_topics_and_reset_service():
    module = _load_module()

    description = _description(module)
    teleop_node = next(
        entity
        for entity in description.entities
        if isinstance(entity, _Node)
        and entity.kwargs["name"] == "franka_pico_vr_teleop"
    )
    parameters = teleop_node.kwargs["parameters"][0]

    assert parameters["joint_state_topic"] == "/franka/joint_states"
    assert parameters["joint_command_topic"] == "/robot/franka/joint_cmd"
    assert parameters["reset_service"] == "/robot/control/reset"
    assert teleop_node.kwargs["remappings"] == [
        ("/robot/franka/joint_cmd", "/franka_teleop/joint_franka")
    ]
    assert parameters["ik_regularization_parameter"].value.name == (
        "ik_regularization_parameter"
    )
    assert parameters["stale_timeout_s"].value.name == "stale_timeout_s"
    assert parameters["stale_timeout_s"].value_type is float
    stale_argument = next(
        entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
        and entity.name == "stale_timeout_s"
    )
    assert stale_argument.default_value == "0.3"
    assert "enable_gripper" not in parameters
    assert parameters["publish_activation"] is False
    assert parameters["recapture_service"] == "~/recapture_baseline"


def test_launch_requires_project_manager_config():
    module = _load_module()

    description = _description(module)
    arguments = [
        entity.name
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    ]

    assert arguments == [
        "robot_ip",
        "use_rviz",
        "load_gripper",
        "enable_robotiq",
        "robotiq_use_fake",
        "auto_enable",
        "robotiq_config_file",
        "engagement_config_file",
        "urdf_path",
        "source_type",
        "control_frequency_hz",
        "stale_timeout_s",
        "ik_regularization_parameter",
        "control_manager_config_file",
    ]
    manager_argument = next(
        entity
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
        and entity.name == "control_manager_config_file"
    )
    assert manager_argument.default_value is None
    manager = next(
        entity
        for entity in description.entities
        if isinstance(entity, _Node)
        and entity.kwargs["name"] == "control_manager"
    )
    assert manager.kwargs["package"] == "robo_orchard_control_manager_ros2"
    assert manager.kwargs["executable"] == "control_manager_node"
    assert manager.kwargs["namespace"] == "/robot/control"
    assert manager.kwargs["parameters"][0]["config_file"].name == (
        "control_manager_config_file"
    )


def test_documented_manager_config_matches_launch(monkeypatch):
    ros_packages = LAUNCH_PATH.parents[2]
    monkeypatch.syspath_prepend(
        str(ros_packages / "robo_orchard_control_manager_ros2")
    )
    from robo_orchard_control_manager_ros2.config import ControlManagerConfig

    guide = (LAUNCH_PATH.parents[1] / "README.md").read_text()
    _, heading, section = guide.partition(
        "### Franka managed teleoperation and DAgger\n"
    )
    assert heading, "Franka teleoperation section is missing from README"
    section = section.split("\n### ", 1)[0].split("\n## ", 1)[0]
    _, opening, example = section.partition("```yaml\n")
    assert opening, "Franka Control Manager YAML example is missing"
    example, closing, _ = example.partition("```")
    assert closing, "Franka Control Manager YAML code block is not closed"
    config = ControlManagerConfig.model_validate(yaml.safe_load(example))
    description = _description()
    teleop = next(
        entity
        for entity in description.entities
        if isinstance(entity, _Node)
        and entity.kwargs["name"] == "franka_pico_vr_teleop"
    )
    channel = config.channels[0]
    assert len(config.channels) == 1
    assert teleop.kwargs["remappings"] == [
        (channel.output_topic, channel.override_topic)
    ]
    assert channel.msg_type == "sensor_msgs/msg/JointState"
    assert channel.autonomous_topic == "/inference/joint_franka"
    assert config.reset_services == ["/robot/franka/reset_ctrl"]
    assert config.enable_services == []
    assert config.inference_disable_services == []
    assert config.replay_time_s == 0.0


def test_manifest_has_no_redundant_runtime_dependencies():
    manifest = ET.parse(LAUNCH_PATH.parents[1] / "package.xml").getroot()
    generic = {element.text for element in manifest.findall("depend")}
    runtime = {element.text for element in manifest.findall("exec_depend")}
    assert not generic.intersection(runtime)
    assert "robo_orchard_teleop_msg_ros2" in generic
    assert "robo_orchard_control_manager_ros2" in runtime
    assert "robo_orchard_robotiq_ros2" in runtime


def test_disabled_robotiq_is_not_resolved(monkeypatch):
    module = _load_module()

    def package_share(package):
        assert "robotiq" not in package
        return _FindPackageShare(package)

    monkeypatch.setattr(module, "FindPackageShare", package_share)
    description = _description(module)
    assert not any(
        isinstance(entity, _Node)
        and entity.kwargs["executable"]
        in ("robotiq_pico_vr_teleop", "teleop_engagement")
        for entity in description.entities
    )


def test_conflicting_grippers_fail_before_constructing_nodes(monkeypatch):
    module = _load_module()

    def unexpected_node(**kwargs):
        pytest.fail("A node was constructed before validating gripper choice")

    monkeypatch.setattr(module, "Node", unexpected_node)
    with pytest.raises(ValueError, match="mutually exclusive"):
        _description(module, enable_robotiq="true", load_gripper="true")


@pytest.mark.parametrize("driver_config", ["", "/project/robotiq.yaml"])
def test_combined_launch_and_shipped_configs(monkeypatch, driver_config):
    monkeypatch.syspath_prepend(
        str(LAUNCH_PATH.parents[2] / "robo_orchard_control_manager_ros2")
    )
    from robo_orchard_control_manager_ros2.config import ControlManagerConfig

    description = _description(
        enable_robotiq="true", robotiq_config_file=driver_config
    )
    nodes = {
        entity.kwargs["executable"]: entity.kwargs
        for entity in description.entities
        if isinstance(entity, _Node)
    }
    assert list(nodes) == [
        "pico_bridge",
        "franka_pico_vr_teleop",
        "control_manager_node",
        "robotiq_pico_vr_teleop",
        "teleop_engagement",
    ]
    arm = nodes["franka_pico_vr_teleop"]
    hand = nodes["robotiq_pico_vr_teleop"]
    assert arm["parameters"][0]["publish_activation"] is True
    assert hand["parameters"][0]["source_type"].name == (
        arm["parameters"][0]["source_type"].name
    )
    includes = [
        entity
        for entity in description.entities
        if isinstance(entity, _IncludeLaunchDescription)
    ]
    assert len(includes) == 2
    driver = includes[1]
    assert (
        driver.launch_description_source.launch_file_path.substitutions[
            0
        ].package
        == "robo_orchard_robotiq_ros2"
    )
    assert driver.launch_arguments["namespace"] == "robotiq"
    assert driver.launch_arguments["use_fake"].name == "robotiq_use_fake"
    assert driver.launch_arguments["auto_enable"].name == "auto_enable"
    if driver_config:
        assert driver.launch_arguments["config_file"] == driver_config
    else:
        assert "config_file" not in driver.launch_arguments

    config_dir = LAUNCH_PATH.parents[1] / "config"
    gate = yaml.safe_load(
        (config_dir / "franka_robotiq_engagement.yaml").read_text()
    )
    manager = ControlManagerConfig.model_validate(
        yaml.safe_load(
            (config_dir / "franka_robotiq_manager.example.yaml").read_text()
        )
    )
    assert (
        arm["remappings"][0][1] == (gate["command_routes"][0]["input_topic"])
    )
    assert (
        arm["parameters"][0]["recapture_service"]
        == (gate["recapture_service"])
    )
    assert gate["command_routes"][1]["input_topic"] == (
        "/robotiq_teleop/candidate"
    )
    assert gate["match_state_topic"] == "/robotiq_teleop/match_state"
    assert [r["output_topic"] for r in gate["command_routes"]] == [
        channel.override_topic for channel in manager.channels
    ]
    assert [channel.output_topic for channel in manager.channels] == [
        "/robot/franka/joint_cmd",
        "/robotiq/joint_commands",
    ]
    assert manager.enable_services == []
    assert manager.reset_services == ["/robot/franka/reset_ctrl"]
    assert manager.replay_time_s == 0.0


def test_robotiq_auto_enable_defaults_off_and_is_typed_at_driver():
    for path in (
        LAUNCH_PATH,
        LAUNCH_PATH.parents[2]
        / "robo_orchard_robotiq_ros2/launch/robotiq_driver.launch.py",
    ):
        description = _load_module(path).generate_launch_description()
        argument = next(
            entity
            for entity in description.entities
            if isinstance(entity, _DeclareLaunchArgument)
            and entity.name == "auto_enable"
        )
        assert argument.default_value == "false"
        if path != LAUNCH_PATH:
            driver = next(
                entity
                for entity in description.entities
                if isinstance(entity, _Node)
            )
            value = driver.kwargs["parameters"][1]["auto_enable"]
            assert value.value.name == "auto_enable"
            assert value.value_type is bool


@pytest.mark.parametrize("load_gripper", ["false", "true"])
def test_driver_optional_gripper_model_launch_and_feedback(load_gripper):
    driver_package = LAUNCH_PATH.parents[2] / "robo_orchard_franka_ros2"
    module = _load_module(
        driver_package / "launch/joint_position_control.launch.py"
    )
    description = module.generate_launch_description()
    defaults = {
        entity.name: entity.default_value
        for entity in description.entities
        if isinstance(entity, _DeclareLaunchArgument)
    }
    assert defaults["load_gripper"] == "false"
    values = {
        **defaults,
        "robot_ip": "192.0.2.1",
        "load_gripper": load_gripper,
    }
    context = types.SimpleNamespace(
        perform_substitution=lambda substitution: values[substitution.name]
    )
    mappings = {}

    def process_file(path, **kwargs):
        assert path == "/share/franka_description/robots/fr3/fr3.urdf.xacro"
        mappings.update(kwargs["mappings"])
        return types.SimpleNamespace(toprettyxml=lambda indent: str(mappings))

    module.xacro.process_file = process_file
    looked_up = []

    def package_share(name):
        looked_up.append(name)
        if load_gripper == "false":
            assert name != "franka_gripper"
        return f"/share/{name}"

    module.get_package_share_directory = package_share
    factory = next(
        entity
        for entity in description.entities
        if isinstance(entity, _OpaqueFunction)
    )
    entities = factory.function(context, *factory.args)
    assert mappings["hand"] == load_gripper
    assert mappings["robot_ip"] == values["robot_ip"]
    nodes = [entity for entity in entities if isinstance(entity, _Node)]
    arm = next(
        node
        for node in nodes
        if node.kwargs["executable"] == "ros2_control_node"
    )
    assert arm.kwargs["remappings"] == [
        ("joint_states", "franka/joint_states")
    ]
    joint_publisher = next(
        node
        for node in nodes
        if node.kwargs["executable"] == "joint_state_publisher"
    )
    sources = ["franka/joint_states"]
    assert not any(
        isinstance(entity, _IncludeLaunchDescription) for entity in entities
    )
    grippers = [
        entity
        for entity in nodes
        if entity.kwargs["package"] == "franka_gripper"
    ]
    if load_gripper == "true":
        sources.append("fr3_gripper/joint_states")
        assert "franka_gripper" in looked_up
        assert len(grippers) == 1
        gripper = grippers[0].kwargs
        assert gripper["executable"] == "franka_gripper_node"
        assert gripper["name"] == "fr3_gripper"
        assert gripper["parameters"] == [
            "/share/franka_gripper/config/franka_gripper_node.yaml",
            {
                "robot_ip": values["robot_ip"],
                "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"],
            },
        ]
    else:
        assert not grippers
    assert joint_publisher.kwargs["parameters"] == [
        {"source_list": sources, "rate": 30}
    ]
    manifest = ET.parse(driver_package / "package.xml").getroot()
    assert "franka_gripper" in {
        element.text for element in manifest.findall("exec_depend")
    }
