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

import pytest
import yaml
from pydantic import ValidationError

from robo_orchard_control_manager_ros2.config import (
    ControlManagerConfig,
    load_control_manager_config,
)


def _channel(name="left_arm", output_topic=None):
    return {
        "name": name,
        "kind": "joint_command",
        "msg_type": "sensor_msgs/msg/JointState",
        "autonomous_topic": f"/{name}/algo_cmd",
        "override_topic": f"/{name}/override_cmd",
        "output_topic": output_topic or f"/robot/{name}/joint_cmd",
    }


def _config(channels=None):
    return {
        "channels": [_channel()] if channels is None else channels,
        "enable_services": ["/robot/left/enable_ctrl"],
        "reset_services": ["/robot/left/reset_ctrl"],
        "inference_disable_services": ["/robot/inference_service/disable"],
        "replay_time_s": 2.0,
        "status_publish_rate_hz": 1.0,
    }


def test_multi_arm_and_hand_config_loads(tmp_path):
    channels = [
        _channel("left_arm"),
        _channel("right_arm"),
        _channel("left_hand"),
        _channel("right_hand"),
    ]
    config_path = tmp_path / "control_manager.yaml"
    config_path.write_text(yaml.safe_dump(_config(channels)), encoding="utf-8")

    config = load_control_manager_config(config_path)

    assert [channel.name for channel in config.channels] == [
        "left_arm",
        "right_arm",
        "left_hand",
        "right_hand",
    ]
    assert config.channels[0].kind == "joint_command"
    assert config.enable_services == ["/robot/left/enable_ctrl"]


def test_empty_channel_list_is_rejected():
    with pytest.raises(ValidationError, match="at least 1 item"):
        ControlManagerConfig.model_validate(_config(channels=[]))


def test_duplicate_channel_names_are_rejected():
    channels = [_channel("left_arm"), _channel("left_arm", "/other")]

    with pytest.raises(ValidationError, match="Duplicate channel names"):
        ControlManagerConfig.model_validate(_config(channels))


def test_duplicate_output_topics_are_rejected():
    channels = [
        _channel("left_arm", "/robot/shared_cmd"),
        _channel("right_arm", "/robot/shared_cmd"),
    ]

    with pytest.raises(ValidationError, match="Duplicate output topics"):
        ControlManagerConfig.model_validate(_config(channels))


@pytest.mark.parametrize(
    "field",
    [
        "autonomous_topic",
        "override_topic",
        "output_topic",
    ],
)
def test_empty_topics_are_rejected(field):
    channel = _channel()
    channel[field] = "  "

    with pytest.raises(ValidationError, match=field):
        ControlManagerConfig.model_validate(_config([channel]))


def test_deploy_only_fields_are_rejected():
    channel = _channel()
    channel["server_output_key"] = "left_arm_actions"
    channel["joint_names"] = ["joint1"]

    with pytest.raises(ValidationError, match="Extra inputs"):
        ControlManagerConfig.model_validate(_config([channel]))


def test_negative_replay_time_is_rejected():
    data = _config()
    data["replay_time_s"] = -0.1

    with pytest.raises(ValidationError, match="greater than or equal to 0"):
        ControlManagerConfig.model_validate(data)


@pytest.mark.parametrize(
    "field", ["service_wait_timeout_s", "service_response_timeout_s"]
)
def test_service_timeouts_must_be_positive(field):
    data = _config()
    data[field] = 0.0

    with pytest.raises(ValidationError, match="greater than 0"):
        ControlManagerConfig.model_validate(data)


def test_inference_node_discovery_is_opt_in():
    config = ControlManagerConfig.model_validate(_config())
    assert config.inference_node_candidates == []


def test_inference_candidates_accept_fully_qualified_names():
    data = _config()
    data["inference_node_candidates"] = [
        "/sync_node",
        "/robot/inference_service/async_node",
    ]
    config = ControlManagerConfig.model_validate(data)
    assert (
        config.inference_node_candidates == data["inference_node_candidates"]
    )


@pytest.mark.parametrize(
    "name", ["", "async_node", "/", "/node/", "/robot//node", "/robot/1node"]
)
def test_invalid_inference_node_names_are_rejected(name):
    data = _config()
    data["inference_node_candidates"] = [name]
    with pytest.raises(ValidationError, match="inference_node_candidates"):
        ControlManagerConfig.model_validate(data)


def test_inference_candidates_require_disable_services():
    data = _config()
    data["inference_node_candidates"] = ["/inference_node"]
    data["inference_disable_services"] = []
    with pytest.raises(
        ValidationError, match="requires inference_disable_services"
    ):
        ControlManagerConfig.model_validate(data)
