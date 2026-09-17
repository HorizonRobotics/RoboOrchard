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

from pathlib import Path

import pytest
from launch_stubs import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    Node,
    ParameterValue,
    load_launch_module,
)

PACKAGE_PATH = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_PATH / "launch"
REPOSITORY_PATH = PACKAGE_PATH.parents[1]


def _nodes(filename):
    module = load_launch_module(LAUNCH_PATH / filename, filename)
    description = module.generate_launch_description()
    return [
        entity for entity in description.entities if isinstance(entity, Node)
    ]


@pytest.mark.parametrize(
    "filename",
    [
        "piper_control_compat.launch.py",
        "piper_dagger_compat.launch.py",
        "piper_pico_dagger_compat.launch.py",
        "piper_pico_teleop_compat.launch.py",
    ],
)
def test_piper_runtime_has_one_manager_and_no_legacy_router(filename):
    nodes = _nodes(filename)

    assert (
        sum(
            node.kwargs["package"] == "robo_orchard_control_manager_ros2"
            for node in nodes
        )
        == 1
    )
    assert not any(
        node.kwargs["executable"]
        in {
            "take_over",
            "aloha_orchestrator",
            "vr_orchestrator",
        }
        for node in nodes
    )


@pytest.mark.parametrize(
    "filename",
    [
        "piper_control_compat.launch.py",
        "piper_dagger_compat.launch.py",
        "piper_pico_dagger_compat.launch.py",
        "piper_pico_teleop_compat.launch.py",
    ],
)
def test_piper_driver_joint_topics_are_manager_outputs(filename):
    nodes = _nodes(filename)
    drivers = [
        node
        for node in nodes
        if node.kwargs["package"] == "robo_orchard_piper_ros2"
        and node.kwargs["executable"] == "single_ctrl"
    ]

    assert drivers
    for driver in drivers:
        for source, target in driver.kwargs.get("remappings", []):
            if source.endswith("/joint_cmd"):
                assert target in {
                    "/robot/left/joint_cmd",
                    "/robot/right/joint_cmd",
                }


@pytest.mark.parametrize(
    "filename",
    [
        "piper_control_compat.launch.py",
        "piper_dagger_compat.launch.py",
        "piper_pico_dagger_compat.launch.py",
        "piper_pico_teleop_compat.launch.py",
    ],
)
def test_piper_drivers_start_ready_behind_manager_gate(filename):
    nodes = _nodes(filename)
    drivers = [
        node
        for node in nodes
        if node.kwargs["package"] == "robo_orchard_piper_ros2"
        and node.kwargs["executable"] == "single_ctrl"
    ]

    assert drivers
    assert all(
        driver.kwargs["parameters"][0]["auto_enable_arm_ctrl"] is True
        for driver in drivers
    )


def test_aloha_compat_delegates_to_managed_runtime() -> None:
    module = load_launch_module(
        LAUNCH_PATH / "piper_aloha_compat.launch.py",
        "piper_aloha_compat",
    )
    description = module.generate_launch_description()
    includes = [
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert len(includes) == 1
    launch_arguments = dict(includes[0].launch_arguments)
    assert launch_arguments["control_manager_config_file"].name == (
        "control_manager_config_file"
    )
    arguments = {
        entity.name: entity.default_value
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    dagger_module = load_launch_module(
        LAUNCH_PATH / "piper_dagger_compat.launch.py", "piper_dagger_compat"
    )
    dagger_arguments = {
        entity.name: entity.default_value
        for entity in dagger_module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    frame_arguments = [
        f"{role}_{parameter}"
        for role in ("left", "right", "left_master", "right_master")
        for parameter in ("base_frame_id", "ee_frame_id")
    ]
    for name in [*frame_arguments, "publish_ee_tf", "publish_master_ee_tf"]:
        assert arguments[name] == dagger_arguments[name]
        assert launch_arguments[name].name == name
        assert launch_arguments[name].perform(arguments) == arguments[name]
        arguments[name] = (
            "false"
            if name == "publish_ee_tf"
            else "true"
            if name == "publish_master_ee_tf"
            else f"custom_{name}"
        )
        assert launch_arguments[name].perform(arguments) == arguments[name]


@pytest.mark.parametrize(
    "filename",
    [
        "piper_control_compat.launch.py",
        "piper_dagger_compat.launch.py",
        "piper_pico_dagger_compat.launch.py",
        "piper_pico_teleop_compat.launch.py",
    ],
)
@pytest.mark.parametrize("override_parameters", [False, True])
def test_piper_ee_frames_and_tf_switches_reach_each_driver(
    filename: str, override_parameters: bool
) -> None:
    module = load_launch_module(LAUNCH_PATH / filename, filename)
    description = module.generate_launch_description()
    arguments = {
        entity.name: entity.default_value
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    drivers = {
        entity.kwargs["namespace"].rsplit("/", 1)[1]: entity.kwargs[
            "parameters"
        ][0]
        for entity in description.entities
        if isinstance(entity, Node)
        and entity.kwargs["package"] == "robo_orchard_piper_ros2"
        and entity.kwargs["executable"] == "single_ctrl"
    }
    roles = ["left", "right"]
    assert arguments["publish_ee_tf"] == "true"
    if filename == "piper_dagger_compat.launch.py":
        roles.extend(["left_master", "right_master"])
        assert arguments["publish_master_ee_tf"] == "false"
    assert set(drivers) == set(roles)
    if override_parameters:
        arguments["publish_ee_tf"] = "false"
        arguments["publish_master_ee_tf"] = "true"

    for role in roles:
        for parameter_name, suffix in (
            ("base_frame_id", "base_link"),
            ("ee_frame_id", "end_effector"),
        ):
            argument_name = f"{role}_{parameter_name}"
            assert arguments[argument_name] == f"{role}_{suffix}"
            if override_parameters:
                arguments[argument_name] = f"custom_{role}_{suffix}"
            parameter = drivers[role][parameter_name]
            assert isinstance(parameter, ParameterValue)
            assert parameter.value_type is str
            assert parameter.value.name == argument_name
            assert (
                parameter.value.perform(arguments) == arguments[argument_name]
            )

        tf_argument_name = (
            "publish_master_ee_tf"
            if role.endswith("_master")
            else "publish_ee_tf"
        )
        tf_parameter = drivers[role]["publish_ee_tf"]
        assert isinstance(tf_parameter, ParameterValue)
        assert tf_parameter.value_type is bool
        assert tf_parameter.value.name == tf_argument_name
        assert (
            tf_parameter.value.perform(arguments)
            == arguments[tf_argument_name]
        )


@pytest.mark.parametrize(
    "filename",
    [
        "piper_pico_dagger_compat.launch.py",
        "piper_pico_teleop_compat.launch.py",
    ],
)
def test_pico_runtimes_do_not_switch_manager_from_controller_inputs(filename):
    nodes = _nodes(filename)
    names = {node.kwargs["name"] for node in nodes}
    assert "pico_takeover_intent" not in names
    assert "pico_auto_intent" not in names


@pytest.mark.parametrize("filename", ["aloha_dagger.sh", "pico_dagger.sh"])
def test_holobrain_scripts_warn_that_replay_env_is_ignored(filename):
    script = (
        REPOSITORY_PATH / "projects" / "HoloBrain" / "teleop" / filename
    ).read_text(encoding="utf-8")

    assert "REPLAY_TIME_S is ignored" in script
    assert "replay_time_s in the Control Manager config" in script


@pytest.mark.parametrize(
    "filename",
    [
        "piper_control_compat.launch.py",
        "piper_dagger_compat.launch.py",
        "piper_aloha_compat.launch.py",
        "piper_pico_dagger_compat.launch.py",
        "piper_pico_teleop_compat.launch.py",
        "marvin_pico_teleop.launch.py",
        "marvin_wuji_keyboard_teleop.launch.py",
        "wuji_glove_teleop_compat.launch.py",
        "wuji_glove_dagger_compat.launch.py",
    ],
)
def test_manager_config_is_supplied_by_the_project(filename):
    module = load_launch_module(LAUNCH_PATH / filename, filename)
    description = module.generate_launch_description()
    argument = next(
        action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
        and action.name == "control_manager_config_file"
    )
    assert argument.default_value is None
