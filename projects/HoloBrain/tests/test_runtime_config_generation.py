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

# ruff: noqa: I001
import importlib.util
import json
import os
import subprocess
import sys
import types
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

PROJECT_PATH = Path(__file__).resolve().parents[1]
REPOSITORY_PATH = PROJECT_PATH.parents[1]
INFERENCE_PATH = PROJECT_PATH / "inference"
for package_path in (
    INFERENCE_PATH,
    REPOSITORY_PATH / "ros2_package/robo_orchard_control_manager_ros2",
    REPOSITORY_PATH / "ros2_package/robo_orchard_data_ros2",
    REPOSITORY_PATH / "ros2_package/robo_orchard_deploy_ros2",
):
    sys.path.insert(0, str(package_path))

qos = types.ModuleType("rclpy.qos")
qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=0, TRANSIENT_LOCAL=1)
qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=0)
qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=0)
rclpy = types.ModuleType("rclpy")
rclpy.qos = qos
sys.modules.setdefault("rclpy", rclpy)
sys.modules.setdefault("rclpy.qos", qos)

from control_channels import COMMAND_CHANNELS  # noqa: E402
import control_channels  # noqa: E402


def _load(relative_path: str, name: str):
    path = PROJECT_PATH / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_deploy_and_manager_channels_match_transport_identity():
    sync = _load("inference/gen_sync_config.py", "holobrain_sync_config")
    async_ = _load("inference/gen_async_config.py", "holobrain_async_config")
    manager = _load(
        "teleop/gen_control_manager_config.py", "holobrain_manager_config"
    )

    expected = [
        (
            channel.server_output_key,
            channel.autonomous_topic,
            "sensor_msgs/msg/JointState",
        )
        for channel in COMMAND_CHANNELS
    ]
    for deploy_channels in (
        sync.build_action_channels(),
        async_.build_action_channels(),
    ):
        assert [
            (channel.server_output_key, channel.topic, channel.msg_type)
            for channel in deploy_channels
        ] == expected
        for side, channel in zip(
            ("left", "right"), deploy_channels, strict=True
        ):
            assert channel.joint_names == [
                f"{side}_joint{index}" for index in range(1, 7)
            ] + [f"{side}_gripper"]

    config = manager.build_config("pico")
    assert [
        (
            channel.name,
            channel.autonomous_topic,
            channel.output_topic,
            channel.msg_type,
        )
        for channel in config.channels
    ] == [
        (
            f"{side}_arm",
            f"/{side}_algo_cmd",
            f"/robot/{side}/joint_cmd",
            "sensor_msgs/msg/JointState",
        )
        for side in ("left", "right")
    ]


def test_manager_generator_selects_runtime_override_and_services():
    manager = _load(
        "teleop/gen_control_manager_config.py", "holobrain_manager_services"
    )

    aloha = manager.build_config("aloha")
    pico = manager.build_config("pico")

    assert [channel.override_topic for channel in aloha.channels] == [
        "/master/joint_left",
        "/master/joint_right",
    ]
    assert [channel.override_topic for channel in pico.channels] == [
        "/pico_teleop/joint_left",
        "/pico_teleop/joint_right",
    ]
    assert aloha.inference_disable_services == [
        "/robot/inference_service/disable"
    ]
    assert pico.inference_disable_services == [
        "/robot/inference_service/disable"
    ]
    for config in (aloha, pico):
        assert config.inference_node_candidates == [
            "/robot/inference_service/sync_node",
            "/robot/inference_service/async_node",
        ]


def test_hexfellow_manager_adds_chassis_and_lift_to_pico():
    manager = _load(
        "teleop/gen_control_manager_config.py", "holobrain_hexfellow_config"
    )

    pico = manager.build_config("pico")
    combined = manager.build_config("pico", with_hexfellow=True)

    assert combined.channels[:2] == pico.channels
    assert [
        (
            channel.name,
            channel.msg_type,
            channel.autonomous_topic,
            channel.override_topic,
            channel.output_topic,
        )
        for channel in combined.channels[2:]
    ] == [
        (
            "chassis",
            "geometry_msgs/msg/Twist",
            "/chassis_algo_cmd",
            "/hexfellow_teleop/cmd_vel",
            "/chassis/cmd_vel",
        ),
        (
            "lift",
            "sensor_msgs/msg/JointState",
            "/lift_algo_cmd",
            "/hexfellow_teleop/lift_joint_cmd",
            "/lift/joint_cmd",
        ),
    ]
    assert combined.replay_time_s == 0
    assert combined.enable_services == pico.enable_services
    assert combined.reset_services == pico.reset_services
    with pytest.raises(ValueError, match="requires TELEOP_SOURCE=pico"):
        manager.build_config("aloha", with_hexfellow=True)


def test_hexfellow_environment_generates_validated_config(
    monkeypatch, tmp_path
):
    manager = _load(
        "teleop/gen_control_manager_config.py", "holobrain_hexfellow_env"
    )
    monkeypatch.setattr(manager, "__file__", str(tmp_path / "generator.py"))
    monkeypatch.setenv("TELEOP_SOURCE", "pico")
    monkeypatch.setenv("WITH_HEXFELLOW", "true")

    manager.main()

    data = json.loads(
        (tmp_path / "control_manager.json").read_text(encoding="utf-8")
    )
    assert len(manager.ControlManagerConfig.model_validate(data).channels) == 4

    monkeypatch.setenv("WITH_HEXFELLOW", "unknown")
    with pytest.raises(ValueError, match="must be true or false"):
        manager.main()


@pytest.mark.parametrize("mode", ["sync", "async"])
def test_inference_generators_preserve_default_model_fields(mode):
    deploy = _load(f"inference/gen_{mode}_config.py", f"default_{mode}")

    assert [
        channel.server_input_key for channel in deploy.build_obs_channels()
    ] == [
        "left_color",
        "left_depth",
        "left_intrinsic",
        "right_color",
        "right_depth",
        "right_intrinsic",
        "middle_color",
        "middle_depth",
        "middle_intrinsic",
        "left_arm_state",
        "right_arm_state",
    ]
    channels = deploy.build_action_channels()
    assert [channel.server_output_key for channel in channels] == [
        "left_arm_actions",
        "right_arm_actions",
    ]
    assert [channel.server_remaining_key for channel in channels] == (
        ["left_arm_remaining_actions", "right_arm_remaining_actions"]
        if mode == "async"
        else [None, None]
    )


@pytest.mark.parametrize("mode", ["sync", "async"])
def test_custom_model_channels_do_not_change_manager_wiring(
    mode, monkeypatch
):
    custom_channel = replace(
        COMMAND_CHANNELS[0],
        server_output_key="predicted_joints",
        server_remaining_key="pending_joints",
        autonomous_topic="/policy/joint_cmd",
    )
    monkeypatch.setattr(
        control_channels, "COMMAND_CHANNELS", (custom_channel,)
    )
    deploy = _load(f"inference/gen_{mode}_config.py", f"custom_{mode}")
    manager = _load(
        "teleop/gen_control_manager_config.py", f"custom_manager_{mode}"
    )

    (action,) = deploy.build_action_channels()
    assert action.server_output_key == "predicted_joints"
    assert action.server_remaining_key == (
        "pending_joints" if mode == "async" else None
    )
    assert action.topic == "/policy/joint_cmd"
    assert action.joint_names == [
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "left_gripper",
    ]
    observation = deploy.build_obs_channels()[-1]
    assert observation.server_input_key == "left_arm_state"
    assert observation.topic == "/puppet/joint_left"
    for source, override in (
        ("aloha", "/master/joint_left"),
        ("pico", "/pico_teleop/joint_left"),
    ):
        channel = manager.build_config(source).channels[0]
        assert channel.name == "left_arm"
        assert channel.autonomous_topic == "/left_algo_cmd"
        assert channel.output_topic == "/robot/left/joint_cmd"
        assert channel.override_topic == override


def test_runtime_scripts_pass_generated_manager_config():
    for filename in (
        "aloha_dagger.sh",
        "pico_dagger.sh",
        "hexfellow_pico_dagger.sh",
    ):
        script = (PROJECT_PATH / "teleop" / filename).read_text(
            encoding="utf-8"
        )
        assert "gen_control_manager_config.py" in script
        assert "control_manager_config_file:=" in script


def test_hexfellow_launch_script_and_template_wiring():
    script = (PROJECT_PATH / "teleop/hexfellow_pico_dagger.sh").read_text(
        encoding="utf-8"
    )
    assert "TELEOP_SOURCE=pico WITH_HEXFELLOW=true" in script
    assert "hexfellow_piper_pico_teleop_compat.launch.py" in script
    for argument in (
        'urdf_path:="$URDF_PATH"',
        'chassis_url:="$CHASSIS_URL"',
        'lift_url:="$LIFT_URL"',
    ):
        assert argument in script

    template = yaml.safe_load(
        (PROJECT_PATH / "launch/templates/launch.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert template["environment"]["WITH_HEXFELLOW"] == "false"
    for name in ("WITH_HEXFELLOW", "CHASSIS_URL", "LIFT_URL"):
        assert f'-e {name}="${name}"' in template["shell_command_before"][0]
    commands = template["windows"][3]["panes"][0]["shell_command"]
    command = next(line for line in commands if line.startswith("CMD="))
    service_command = (
        'if [ "$TELEOP_SOURCE" = "pico" ]; then '
        "bash /opt/apps/roboticsservice/runService.sh; fi"
    )
    assert service_command in commands
    assert commands.index(service_command) < commands.index(command)
    assert "pico:true) SCRIPT=hexfellow_pico_dagger.sh" in command
    assert "pico:false) SCRIPT=pico_dagger.sh" in command
    assert "aloha:false) SCRIPT=aloha_dagger.sh" in command


@pytest.mark.parametrize(
    "missing", ("PIPER_URDF_PATH", "CHASSIS_URL", "LIFT_URL")
)
def test_hexfellow_launch_requires_hardware_arguments(missing):
    script = PROJECT_PATH / "teleop/hexfellow_pico_dagger.sh"
    hardware = {
        "PIPER_URDF_PATH": "/example/piper.urdf",
        "CHASSIS_URL": "192.0.2.10:8439",
        "LIFT_URL": "192.0.2.11:8439",
    }
    hardware[missing] = ""
    result = subprocess.run(
        ["bash", str(script)],
        env={**os.environ, **hardware},
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2
    assert "required" in result.stderr


def test_recorder_uses_global_control_topics():
    recorder = _load(
        "data/recorder/gen_data_recorder_config.py",
        "holobrain_recorder_config",
    )
    default = recorder.build_config()
    topics = default.include_patterns

    assert "/robot/control/status" in topics
    assert "/robot/control/events" in topics
    assert not any("takeover_muxer" in topic for topic in topics)
    assert "/pico_bridge/vr_state" not in topics
    assert "/chassis/motor_states" not in topics
    assert "/chassis/motor_states" not in default.wait_for_topics


def test_recorder_pico_preserves_arm_output_and_vr_input():
    recorder = _load(
        "data/recorder/gen_data_recorder_config.py", "holobrain_pico_recorder"
    )
    pico = recorder.build_config("pico")

    assert {
        "/pico_bridge/vr_state",
        "/robot/left/joint_cmd",
        "/robot/right/joint_cmd",
    } <= set(pico.include_patterns)
    assert "/chassis/motor_states" not in pico.include_patterns
    assert "/chassis/motor_states" not in pico.wait_for_topics
    assert pico.default_topic_spec.stamp_type == "msg_header_stamp"
    for side in ("left", "right"):
        spec = pico.topic_spec[f"/robot/{side}/joint_cmd"]
        assert spec.rename_topic == f"/action/{side}/joint"
        assert spec.stamp_type == "msg_header_stamp"
        assert spec.qos_profile == pico.default_topic_spec.qos_profile
    assert "/pico_bridge/vr_state" not in pico.topic_spec


def test_recorder_hexfellow_requires_states_and_stamps_twist():
    recorder = _load(
        "data/recorder/gen_data_recorder_config.py",
        "holobrain_hexfellow_recorder",
    )
    pico = recorder.build_config("pico")
    combined = recorder.build_config("pico", with_hexfellow=True)
    states = {
        "/chassis/motor_states",
        "/chassis/odom",
        "/lift/motor_states",
    }
    commands = {"/chassis/cmd_vel", "/lift/joint_cmd"}

    assert set(pico.include_patterns) <= set(combined.include_patterns)
    assert states | commands <= set(combined.include_patterns)
    assert combined.wait_for_topics == pico.wait_for_topics | states
    assert combined.topic_spec["/chassis/cmd_vel"].stamp_type == (
        "recorder_clock"
    )
    expected_names = {
        "/chassis/motor_states": "/observation/robot_state/chassis/joint",
        "/chassis/odom": "/observation/robot_state/chassis/odom",
        "/lift/motor_states": "/observation/robot_state/lift/joint",
        "/chassis/cmd_vel": "/action/chassis/cmd_vel",
        "/lift/joint_cmd": "/action/lift/joint",
        "/robot/left/joint_cmd": "/action/left/joint",
        "/robot/right/joint_cmd": "/action/right/joint",
    }
    for topic, name in expected_names.items():
        spec = combined.topic_spec[topic]
        assert spec.rename_topic == name
        assert spec.stamp_type == (
            "recorder_clock"
            if topic == "/chassis/cmd_vel"
            else "msg_header_stamp"
        )
        assert spec.qos_profile.depth == (
            10 if topic == "/chassis/cmd_vel" else 1024
        )
    assert combined.default_topic_spec.stamp_type == "msg_header_stamp"
    with pytest.raises(ValueError, match="requires TELEOP_SOURCE=pico"):
        recorder.build_config("aloha", with_hexfellow=True)


def test_recorder_environment_selects_combined_config(monkeypatch, tmp_path):
    recorder = _load(
        "data/recorder/gen_data_recorder_config.py",
        "holobrain_recorder_env",
    )
    monkeypatch.setattr(recorder, "__file__", str(tmp_path / "generator.py"))
    monkeypatch.setenv("TELEOP_SOURCE", "pico")
    monkeypatch.setenv("WITH_HEXFELLOW", "true")
    recorder.main()

    data = json.loads(
        (tmp_path / "data_recorder.json").read_text(encoding="utf-8")
    )
    config = recorder.RecordConfig.model_validate(data)
    assert "/lift/motor_states" in config.wait_for_topics
    assert config.topic_spec["/chassis/cmd_vel"].stamp_type == (
        "recorder_clock"
    )
    assert config.topic_spec["/chassis/cmd_vel"].rename_topic == (
        "/action/chassis/cmd_vel"
    )
    assert config.topic_spec["/lift/motor_states"].rename_topic == (
        "/observation/robot_state/lift/joint"
    )

    monkeypatch.setenv("TELEOP_SOURCE", "aloha")
    previous = (tmp_path / "data_recorder.json").read_text(encoding="utf-8")
    with pytest.raises(ValueError, match="requires TELEOP_SOURCE=pico"):
        recorder.main()
    assert (tmp_path / "data_recorder.json").read_text(
        encoding="utf-8"
    ) == previous
