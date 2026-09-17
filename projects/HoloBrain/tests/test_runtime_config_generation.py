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
import sys
import types
from dataclasses import replace
from pathlib import Path

import pytest

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


def test_deploy_and_manager_channels_share_transport_identity():
    sync = _load("inference/gen_sync_config.py", "holobrain_sync_config")
    async_ = _load("inference/gen_async_config.py", "holobrain_async_config")
    manager = _load(
        "teleop/gen_control_manager_config.py", "holobrain_manager_config"
    )

    expected = [
        (channel.server_output_key, channel.autonomous_topic, channel.msg_type)
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
            channel.name,
            channel.autonomous_topic,
            channel.output_topic,
            channel.msg_type,
        )
        for channel in COMMAND_CHANNELS
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
def test_custom_channel_labels_do_not_determine_model_keys_or_topics(
    mode, monkeypatch
):
    custom_channel = replace(
        COMMAND_CHANNELS[0],
        name="manipulator",
        server_output_key="predicted_joints",
        server_remaining_key="pending_joints",
        autonomous_topic="/policy/joint_cmd",
        output_topic="/driver/joint_cmd",
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
        (channel,) = manager.build_config(source).channels
        assert channel.name == "manipulator"
        assert channel.autonomous_topic == action.topic
        assert channel.output_topic == "/driver/joint_cmd"
        assert channel.override_topic == override


def test_runtime_scripts_pass_generated_manager_config():
    for filename in ("aloha_dagger.sh", "pico_dagger.sh"):
        script = (PROJECT_PATH / "teleop" / filename).read_text(
            encoding="utf-8"
        )
        assert "gen_control_manager_config.py" in script
        assert "control_manager_config_file:=" in script


def test_recorder_uses_global_control_topics():
    recorder = _load(
        "data/recorder/gen_data_recorder_config.py",
        "holobrain_recorder_config",
    )
    topics = recorder.build_config().include_patterns

    assert "/robot/control/status" in topics
    assert "/robot/control/events" in topics
    assert not any("takeover_muxer" in topic for topic in topics)
