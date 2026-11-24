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

import copy
import importlib.util
import sys
import types
from array import array
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml

# Synthetic validation fixtures, not recommended or official robot settings.
SETTINGS = {
    "joint_stiffness": {"joint_stiffness": [500] * 7},
    "cartesian_stiffness": {"cartesian_stiffness": [1000] * 3 + [100] * 3},
    "load": {
        "mass": 1,
        "center_of_mass": [0, 0, 0.1],
        "load_inertia": [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
    },
    "tcp_frame": {
        "transformation": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0.1, 1]
    },
    "stiffness_frame": {
        "transformation": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    },
    "force_torque_collision_behavior": {
        "lower_torque_thresholds_nominal": [10] * 7,
        "upper_torque_thresholds_nominal": [20] * 7,
        "lower_force_thresholds_nominal": [10] * 6,
        "upper_force_thresholds_nominal": [20] * 6,
    },
    "full_collision_behavior": {
        "lower_torque_thresholds_nominal": [10] * 7,
        "upper_torque_thresholds_nominal": [20] * 7,
        "lower_force_thresholds_nominal": [10] * 6,
        "upper_force_thresholds_nominal": [20] * 6,
        "lower_torque_thresholds_acceleration": [15] * 7,
        "upper_torque_thresholds_acceleration": [25] * 7,
        "lower_force_thresholds_acceleration": [15] * 6,
        "upper_force_thresholds_acceleration": [25] * 6,
    },
}
SERVICE_TYPES = {
    "joint_stiffness": "SetJointStiffness",
    "cartesian_stiffness": "SetCartesianStiffness",
    "load": "SetLoad",
    "tcp_frame": "SetTCPFrame",
    "stiffness_frame": "SetStiffnessFrame",
    "force_torque_collision_behavior": "SetForceTorqueCollisionBehavior",
    "full_collision_behavior": "SetFullCollisionBehavior",
}


def _request_type(fields):
    """Emulate generated float64 setters: fixed shape and float elements."""

    def set_field(self, name, value):
        expected = fields[name]
        if isinstance(expected, list):
            assert len(value) == len(expected)
            assert all(type(item) is float for item in value)
        else:
            assert type(value) is float
        object.__setattr__(self, name, value)

    return type(
        "Request", (), {"__slots__": tuple(fields), "__setattr__": set_field}
    )


@pytest.fixture
def setup_client(monkeypatch):
    node = Mock()
    node.declare_parameter.return_value.value = [500.0] * 7
    node.robot_config_file = ""
    node.declare_parameter.side_effect = lambda name, _, **_kwargs: (
        types.SimpleNamespace(value=node.robot_config_file)
        if name == "robot_config_file"
        else node.declare_parameter.return_value
    )
    client = node.create_client.return_value
    client.wait_for_service.return_value = True
    future = client.call_async.return_value
    future.done.return_value = True
    future.result.return_value = types.SimpleNamespace(success=True, error="")
    modules = {
        "rclpy": {
            "init": Mock(),
            "shutdown": Mock(),
            "spin_until_future_complete": Mock(),
        },
        "rclpy.node": {"Node": Mock(return_value=node)},
        "rcl_interfaces": {},
        "rcl_interfaces.msg": {"ParameterDescriptor": types.SimpleNamespace},
        "franka_msgs": {},
        "franka_msgs.srv": {
            SERVICE_TYPES[section]: types.SimpleNamespace(
                Request=_request_type(fields)
            )
            for section, fields in SETTINGS.items()
        },
    }
    for name, members in modules.items():
        stub = types.ModuleType(name)
        stub.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, stub)
    path = Path(__file__).parents[1] / "scripts/configure_franka.py"
    spec = importlib.util.spec_from_file_location("configure_franka", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, node, client, future


def test_success_sends_exact_gains_and_cleans_up(setup_client):
    module, node, client, _ = setup_client
    gains = [0.0, 250.0, 500.0, 750.0, 1000.0, 2000.0, 14250.0]
    node.declare_parameter.return_value.value = gains

    assert module.main(["--ros-args"]) == 0

    assert node.declare_parameter.call_args.args == ("joint_stiffness", "")
    assert node.declare_parameter.call_args.kwargs["descriptor"].dynamic_typing
    node.declare_parameter.assert_any_call("robot_config_file", "")
    node.create_client.assert_called_once_with(
        module.SetJointStiffness, "service_server/set_joint_stiffness"
    )
    assert client.call_async.call_args.args[0].joint_stiffness == gains
    assert "Nm/rad" in node.get_logger().info.call_args.args[0]
    assert str(gains) in node.get_logger().info.call_args.args[0]
    client.wait_for_service.assert_called_once_with(timeout_sec=30.0)
    assert (
        module.rclpy.spin_until_future_complete.call_args.kwargs["timeout_sec"]
        == 30.0
    )
    module.rclpy.init.assert_called_once_with(args=["--ros-args"])
    node.destroy_node.assert_called_once()
    module.rclpy.shutdown.assert_called_once()


@pytest.mark.parametrize(
    "gains",
    [
        None,
        [],
        [500.0] * 6,
        [500.0] * 8,
        [-1.0] * 7,
        [14250.1] * 7,
        [float("nan")] * 7,
        [float("inf")] * 7,
    ],
)
def test_invalid_gains_fail_before_service_request(setup_client, gains):
    module, node, client, _ = setup_client
    node.declare_parameter.return_value.value = gains

    assert module.main() == 1

    node.create_client.assert_not_called()
    client.call_async.assert_not_called()
    node.get_logger().error.assert_called_once()
    node.destroy_node.assert_called_once()
    module.rclpy.shutdown.assert_called_once()


@pytest.mark.parametrize(
    "failure",
    ["unavailable", "timeout", "rejected", "exception", "no_response"],
)
def test_communication_failures_exit_nonzero(setup_client, failure):
    module, node, client, future = setup_client
    if failure == "unavailable":
        client.wait_for_service.return_value = False
    elif failure == "timeout":
        future.done.return_value = False
    elif failure == "rejected":
        future.result.return_value = types.SimpleNamespace(
            success=False, error="Robot rejected stiffness"
        )
    elif failure == "exception":
        future.result.side_effect = RuntimeError("Transport failure")
    else:
        future.result.return_value = None

    assert module.main() == 1

    node.get_logger().error.assert_called_once()
    node.get_logger().info.assert_not_called()
    node.destroy_node.assert_called_once()
    module.rclpy.shutdown.assert_called_once()
    if failure == "unavailable":
        client.call_async.assert_not_called()
    elif failure == "timeout":
        future.result.assert_not_called()
    elif failure == "rejected":
        assert (
            "Robot rejected stiffness"
            in (node.get_logger().error.call_args.args[0])
        )


def test_wrong_parameter_type_fails_closed(setup_client):
    module, node, client, _ = setup_client
    node.declare_parameter.return_value.value = 500

    assert module.main() == 1

    client.call_async.assert_not_called()


def _config_file(tmp_path, node, config):
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    node.robot_config_file = str(path)


@pytest.mark.parametrize(
    "gains",
    [
        array("d", [500.0] * 7),
        array("q", [500] * 7),
        "[500, 500.0, 500, 500, 500, 500, 500]",
    ],
)
def test_stiffness_parameter_normalizes_numeric_gains(setup_client, gains):
    module, node, client, _ = setup_client
    node.declare_parameter.return_value.value = gains

    assert module.main() == 0
    module.Node.assert_called_once_with("configure_franka")
    assert client.call_async.call_args.args[0].joint_stiffness == [500.0] * 7


@pytest.mark.parametrize("section", SETTINGS)
def test_each_official_section_builds_exact_float_request(
    setup_client, tmp_path, section
):
    module, node, client, _ = setup_client
    _config_file(tmp_path, node, {section: SETTINGS[section]})

    assert module.main() == 0

    service_type, service_name = node.create_client.call_args.args
    assert service_type is getattr(module, SERVICE_TYPES[section])
    assert service_name == f"service_server/set_{section}"
    request = client.call_async.call_args.args[0]
    for field, value in SETTINGS[section].items():
        expected = (
            [float(item) for item in value]
            if isinstance(value, list)
            else float(value)
        )
        assert getattr(request, field) == expected


def test_yaml_joint_stiffness_overrides_launch_gains(setup_client, tmp_path):
    module, node, client, _ = setup_client
    node.declare_parameter.return_value.value = "[invalid"
    _config_file(
        tmp_path, node, {"joint_stiffness": SETTINGS["joint_stiffness"]}
    )

    assert module.main() == 0

    node.declare_parameter.assert_called_once_with("robot_config_file", "")
    assert client.call_async.call_args.args[0].joint_stiffness == [500.0] * 7
    assert str([500.0] * 7) in node.get_logger().info.call_args.args[0]


def test_all_optional_sections_apply_in_file_order_after_joint_stiffness(
    setup_client, tmp_path
):
    module, node, client, _ = setup_client
    config = {
        section: fields
        for section, fields in reversed(list(SETTINGS.items()))
        if section
        not in ("joint_stiffness", "force_torque_collision_behavior")
    }
    _config_file(tmp_path, node, config)

    assert module.main() == 0

    sections = ["joint_stiffness", *config]
    assert [call.args[1] for call in node.create_client.call_args_list] == [
        f"service_server/set_{section}" for section in sections
    ]
    assert client.call_async.call_count == len(sections)


@pytest.mark.parametrize("config", [{}, {"load": SETTINGS["load"]}])
def test_omitted_sections_are_not_written(setup_client, tmp_path, config):
    module, node, _, _ = setup_client
    _config_file(tmp_path, node, config)

    assert module.main() == 0

    assert [call.args[1] for call in node.create_client.call_args_list] == [
        "service_server/set_joint_stiffness",
        *[f"service_server/set_{section}" for section in config],
    ]


@pytest.mark.parametrize(
    "config",
    [
        None,
        [],
        "not a mapping",
        {"unknown": {}},
        {"load": None},
        {"load": {"mass": 1}},
        {"load": {**SETTINGS["load"], "extra": 1}},
        {
            "force_torque_collision_behavior": SETTINGS[
                "force_torque_collision_behavior"
            ],
            "full_collision_behavior": SETTINGS["full_collision_behavior"],
        },
    ],
)
def test_invalid_structure_fails_before_any_writes(
    setup_client, tmp_path, config
):
    module, node, client, _ = setup_client
    _config_file(tmp_path, node, config)

    assert module.main() == 1

    node.create_client.assert_not_called()
    client.call_async.assert_not_called()


@pytest.mark.parametrize(
    "contents", ["[broken", "", "!!python/object:unknown {}"]
)
def test_invalid_yaml_file_fails_before_any_writes(
    setup_client, tmp_path, contents
):
    module, node, client, _ = setup_client
    path = tmp_path / "robot.yaml"
    path.write_text(contents, encoding="utf-8")
    node.robot_config_file = str(path)

    assert module.main() == 1

    client.call_async.assert_not_called()


def test_missing_file_fails_before_any_writes(setup_client, tmp_path):
    module, node, client, _ = setup_client
    node.robot_config_file = str(tmp_path / "missing.yaml")

    assert module.main() == 1

    client.call_async.assert_not_called()


@pytest.mark.parametrize(
    "section,field",
    [
        (section, field)
        for section, fields in SETTINGS.items()
        for field in fields
    ],
)
@pytest.mark.parametrize("invalid", [True, "1.0", float("nan"), float("inf")])
def test_every_numeric_field_rejects_invalid_elements_before_writes(
    setup_client, tmp_path, section, field, invalid
):
    module, node, client, _ = setup_client
    fields = copy.deepcopy(SETTINGS[section])
    if isinstance(fields[field], list):
        fields[field][0] = invalid
    else:
        fields[field] = invalid
    _config_file(tmp_path, node, {section: fields})

    assert module.main() == 1

    node.create_client.assert_not_called()
    client.call_async.assert_not_called()


@pytest.mark.parametrize(
    "section,field",
    [
        (section, field)
        for section, fields in SETTINGS.items()
        for field, value in fields.items()
        if isinstance(value, list)
    ],
)
@pytest.mark.parametrize("invalid", [[], None, "not an array", [1.0] * 17])
def test_every_array_field_rejects_wrong_shape_before_writes(
    setup_client, tmp_path, section, field, invalid
):
    module, node, client, _ = setup_client
    fields = {**SETTINGS[section], field: invalid}
    _config_file(tmp_path, node, {section: fields})

    assert module.main() == 1

    client.call_async.assert_not_called()


@pytest.mark.parametrize(
    "section,field,index,value",
    [
        ("joint_stiffness", "joint_stiffness", 0, -1),
        ("joint_stiffness", "joint_stiffness", 0, 14251),
        ("cartesian_stiffness", "cartesian_stiffness", 0, 9),
        ("cartesian_stiffness", "cartesian_stiffness", 2, 3001),
        ("cartesian_stiffness", "cartesian_stiffness", 3, 0),
        ("cartesian_stiffness", "cartesian_stiffness", 5, 301),
        ("load", "mass", None, -1),
    ],
)
def test_invalid_ranges_fail_before_writes(
    setup_client, tmp_path, section, field, index, value
):
    module, node, client, _ = setup_client
    fields = copy.deepcopy(SETTINGS[section])
    if index is None:
        fields[field] = value
    else:
        fields[field][index] = value
    _config_file(tmp_path, node, {section: fields})

    assert module.main() == 1

    client.call_async.assert_not_called()


@pytest.mark.parametrize(
    "section,field",
    [
        (section, field)
        for section in (
            "force_torque_collision_behavior",
            "full_collision_behavior",
        )
        for field in SETTINGS[section]
    ],
)
@pytest.mark.parametrize("invalid", [-1, "inverted"])
def test_collision_thresholds_reject_negative_and_inverted_bounds(
    setup_client, tmp_path, section, field, invalid
):
    module, node, client, _ = setup_client
    fields = copy.deepcopy(SETTINGS[section])
    fields[field][0] = (
        -1 if invalid == -1 else (30 if field.startswith("lower") else 0)
    )
    _config_file(tmp_path, node, {section: fields})

    assert module.main() == 1

    client.call_async.assert_not_called()


def test_later_service_failure_stops_without_retry_or_rollback(
    setup_client, tmp_path
):
    module, node, client, future = setup_client
    _config_file(
        tmp_path,
        node,
        {"load": SETTINGS["load"], "tcp_frame": SETTINGS["tcp_frame"]},
    )
    future.result.side_effect = [
        types.SimpleNamespace(success=True, error=""),
        types.SimpleNamespace(success=False, error="Payload rejected"),
    ]

    assert module.main() == 1

    assert [call.args[1] for call in node.create_client.call_args_list] == [
        "service_server/set_joint_stiffness",
        "service_server/set_load",
    ]
    assert client.call_async.call_count == 2
    assert node.get_logger().info.call_count == 1
    assert "Payload rejected" in node.get_logger().error.call_args.args[0]


def test_installed_example_only_applies_joint_stiffness(setup_client):
    module, node, client, _ = setup_client
    node.robot_config_file = str(
        Path(__file__).parents[1] / "config/robot.example.yaml"
    )

    assert module.main() == 0

    node.create_client.assert_called_once_with(
        module.SetJointStiffness, "service_server/set_joint_stiffness"
    )
    assert client.call_async.call_args.args[0].joint_stiffness == [
        3000.0,
        3000.0,
        3000.0,
        2500.0,
        2500.0,
        2000.0,
        2000.0,
    ]


@pytest.mark.parametrize(
    "section", ["cartesian_stiffness", "full_collision_behavior"]
)
def test_commented_examples_match_official_values_and_requests(
    setup_client, tmp_path, section
):
    module, node, client, _ = setup_client
    example = (
        Path(__file__).parents[1] / "config/robot.example.yaml"
    ).read_text(encoding="utf-8")
    assert (
        "https://github.com/frankarobotics/libfranka/blob/0.18.0/"
        "examples/examples_common.cpp"
    ) in example
    commented = example.split(f"# {section}:\n", 1)[1].split("\n\n", 1)[0]
    config = yaml.safe_load(
        f"{section}:\n"
        + "\n".join(line.removeprefix("# ") for line in commented.splitlines())
    )
    expected = (
        {"cartesian_stiffness": [3000.0] * 3 + [300.0] * 3}
        if section == "cartesian_stiffness"
        else {
            f"{bound}_{kind}_thresholds_{phase}": [value] * size
            for phase, value in (("acceleration", 20.0), ("nominal", 10.0))
            for kind, size in (("torque", 7), ("force", 6))
            for bound in ("lower", "upper")
        }
    )
    assert config == {section: expected}
    _config_file(tmp_path, node, config)

    assert module.main() == 0

    node.create_client.assert_called_with(
        getattr(module, SERVICE_TYPES[section]),
        f"service_server/set_{section}",
    )
    request = client.call_async.call_args.args[0]
    for field, values in expected.items():
        assert getattr(request, field) == values
        assert all(type(value) is float for value in getattr(request, field))


def test_cartesian_range_boundaries_are_accepted(setup_client, tmp_path):
    module, node, client, _ = setup_client
    gains = [10, 3000, 10, 1, 300, 1]
    _config_file(
        tmp_path, node, {"cartesian_stiffness": {"cartesian_stiffness": gains}}
    )

    assert module.main() == 0

    assert client.call_async.call_args.args[0].cartesian_stiffness == [
        float(gain) for gain in gains
    ]


def test_equal_collision_thresholds_are_accepted(setup_client, tmp_path):
    module, node, _, _ = setup_client
    fields = copy.deepcopy(SETTINGS["force_torque_collision_behavior"])
    for field in fields:
        fields[field] = [0] * len(fields[field])
    _config_file(tmp_path, node, {"force_torque_collision_behavior": fields})

    assert module.main() == 0
