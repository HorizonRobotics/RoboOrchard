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
    Node,
    ParameterValue,
    load_launch_module,
)

LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch"


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
