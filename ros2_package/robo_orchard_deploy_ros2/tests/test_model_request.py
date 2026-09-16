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

import json
import types

import numpy as np
import pytest

from robo_orchard_deploy_ros2 import codec
from robo_orchard_deploy_ros2.config import (
    ControlConfig,
    DeployConfig,
    ImageChannel,
    JointCommandChannel,
    JointStateChannel,
    ObservationConfig,
)
from robo_orchard_deploy_ros2.model_request import ModelInferencer


class _RecordingLogger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def warning(self, message, **kwargs):
        self.warnings.append(message)

    def error(self, message, **kwargs):
        self.errors.append(message)

    def info(self, message, **kwargs):
        pass

    def debug(self, message, **kwargs):
        pass


class _FakeNode:
    def __init__(self):
        self.logger = _RecordingLogger()

    def get_logger(self):
        return self.logger

    def get_parameter(self, name):
        return types.SimpleNamespace(
            get_parameter_value=lambda: types.SimpleNamespace(
                string_value="Do something."
            )
        )


class _FakeResponse:
    def __init__(self, payload, status_code=200, content=None):
        self.status_code = status_code
        self.content = (
            json.dumps(payload).encode() if content is None else content
        )


class _FakeSession:
    def __init__(self, payload, status_code=200, content=None):
        self._payload = payload
        self._status_code = status_code
        self._content = content
        self.posts = 0

    def post(self, url, files, data, timeout):
        self.posts += 1
        self.files = files
        self.data = data
        return _FakeResponse(
            self._payload,
            status_code=self._status_code,
            content=self._content,
        )


def _config(server_output_keys=("left_arm_actions", "right_arm_actions")):
    return DeployConfig(
        observation_config=ObservationConfig(
            channels=[ImageChannel(server_input_key="color", topic="/color")]
        ),
        control_config=ControlConfig(
            channels=[
                JointCommandChannel(
                    server_output_key=key,
                    topic=f"/{key}_cmd",
                    joint_names=["joint1"],
                )
                for key in server_output_keys
            ]
        ),
    )


def _infer(
    node,
    payload,
    config=None,
    times=1,
    status_code=200,
    content=None,
):
    inferencer = ModelInferencer(node, config or _config())
    inferencer._session = _FakeSession(
        payload, status_code=status_code, content=content
    )
    for _ in range(times):
        result = inferencer.request_inference({"color": np.zeros((2, 2))})
    return inferencer, result


@pytest.fixture
def node():
    return _FakeNode()


def test_a_matching_contract_is_silent(node):
    _infer(
        node,
        {
            "left_arm_actions": [[0.1]],
            "right_arm_actions": [[0.2]],
        },
    )

    assert node.logger.errors == []
    assert node.logger.warnings == []


def test_a_channel_the_server_never_returns_is_an_error(node):
    _, result = _infer(node, {"left_arm_actions": [[0.1]]})

    assert result is None
    assert any(
        "right_arm_actions" in message and "required action fields" in message
        for message in node.logger.errors
    )


def test_a_response_field_no_channel_claims_is_a_warning(node):
    _infer(
        node,
        {
            "left_arm_actions": [[0.1]],
            "right_arm_actions": [[0.2]],
            "right_hand_actions": [[0.3]],
        },
    )

    assert any(
        "right_hand_actions" in message for message in node.logger.warnings
    )
    assert node.logger.errors == []


def test_action_horizon_is_reported_as_unclaimed(node):
    """No action channel claims action_horizon, so the report names it.

    The HoloBrain server sets it to len(actions), so it carries nothing
    the action arrays do not already carry.
    """
    _infer(
        node,
        {
            "action_horizon": 1,
            "left_arm_actions": [[0.1]],
            "right_arm_actions": [[0.2]],
        },
    )

    assert any("action_horizon" in message for message in node.logger.warnings)


def test_extra_response_fields_are_reported_only_once(node):
    """A valid response reports harmless extra fields only once."""
    inferencer, _ = _infer(
        node,
        {
            "left_arm_actions": [[0.1]],
            "right_arm_actions": [[0.2]],
            "metadata": "value",
        },
        times=5,
    )

    assert inferencer._session.posts == 5
    assert len(node.logger.warnings) == 1


def test_the_response_is_returned_unchanged(node):
    payload = {
        "left_arm_actions": [[0.1]],
        "right_arm_actions": [[0.2]],
    }

    _, result = _infer(node, payload)

    assert result == payload


def test_non_ok_response_is_not_returned_as_actions(node):
    """An HTTP error body must not enter the action state machine."""
    inferencer, result = _infer(
        node,
        {"error": "model failed"},
        status_code=500,
    )

    assert result is None
    assert inferencer._reconciled is False
    assert any("Get an error" in message for message in node.logger.errors)


def test_invalid_json_response_is_rejected(node):
    """Malformed JSON must fail at the HTTP boundary."""
    _, result = _infer(node, None, content=b"not json")

    assert result is None
    assert any(
        "Failed to decode JSON" in message for message in node.logger.errors
    )


def test_non_object_json_response_is_rejected(node):
    """Only JSON objects can represent named action channels."""
    _, result = _infer(node, [[0.1], [0.2]])

    assert result is None
    assert any(
        "must be a JSON object" in message for message in node.logger.errors
    )


def _joint_config(widths):
    return DeployConfig(
        observation_config=ObservationConfig(
            channels=[
                JointStateChannel(
                    server_input_key=f"state{channel_index}",
                    topic=f"/state{channel_index}",
                )
                for channel_index in range(len(widths))
            ]
        ),
        control_config=ControlConfig(
            channels=[
                JointCommandChannel(
                    server_output_key=f"action{channel_index}",
                    topic=f"/cmd{channel_index}",
                    server_remaining_key=f"tail{channel_index}",
                    joint_names=[
                        f"joint{channel_index}_{joint_index}"
                        for joint_index in range(width)
                    ],
                )
                for channel_index, width in enumerate(widths)
            ]
        ),
    )


@pytest.mark.parametrize("widths", [(7, 7), (7, 7, 20, 20)])
@pytest.mark.parametrize("rtc", [False, True])
def test_channel_joint_observations_preserve_names_and_positions(
    node, widths, rtc
):
    config = _joint_config(widths)
    config.control_config.channels = config.control_config.channels[1:][::-1]
    observation = {
        channel.server_input_key: codec.decode(
            channel,
            types.SimpleNamespace(
                name=[
                    f"joint{channel_index}_{joint_index}"
                    for joint_index in range(widths[channel_index])
                ],
                position=np.arange(widths[channel_index], dtype=np.float64),
            ),
        )
        for channel_index, channel in enumerate(
            config.observation_config.channels
        )
    }
    observation["color"] = np.zeros((2, 2, 3), dtype=np.uint8)
    controls = config.control_config.channels
    if rtc:
        observation.update(
            {
                channel.server_remaining_key: np.zeros(
                    (2, len(channel.joint_names))
                )
                for channel in controls
            }
        )
    response = {
        channel.server_output_key: [[0] * len(channel.joint_names)]
        for channel in controls
    }
    inferencer = ModelInferencer(node, config)
    session = _FakeSession(response)
    inferencer._session = session

    assert inferencer.request_inference(observation) == response
    for key, value in observation.items():
        if isinstance(value, dict):
            assert key not in session.files
            assert json.loads(session.data[key]) == value
            assert set(value) == {"name", "position"}
        else:
            np.testing.assert_array_equal(
                np.load(session.files[key][1]), value
            )
            assert key not in session.data
    assert set(session.files) == {
        key
        for key, value in observation.items()
        if isinstance(value, np.ndarray)
    }
    assert set(session.data) == {
        channel.server_input_key
        for channel in config.observation_config.channels
    } | {"instruction", "delay_horizon"}


def test_joint_only_requests_do_not_require_binary_files(node):
    inferencer = ModelInferencer(node, _joint_config([1]))
    response = {"action0": [[0.2]]}
    inferencer._session = _FakeSession(response)
    observation = {"state0": {"name": ["left_joint1"], "position": [0.1]}}

    assert inferencer.request_inference(observation) == response
    assert inferencer._session.files == {}
    assert (
        json.loads(inferencer._session.data["state0"]) == observation["state0"]
    )


@pytest.mark.parametrize(
    "invalid",
    [
        np.array([0.1]),
        {"position": [0.1]},
        {"name": ["left_joint1"], "position": [float("nan")]},
    ],
)
def test_invalid_joint_packets_never_send_http(node, invalid):
    inferencer = ModelInferencer(node, _joint_config([1]))
    inferencer._session = _FakeSession({"action0": [[0.2]]})

    assert inferencer.request_inference({"state0": invalid}) is None
    assert inferencer._session.posts == 0
    assert node.logger.errors


def test_empty_request_does_not_send_http(node):
    inferencer = ModelInferencer(node, _joint_config([1]))
    inferencer._session = _FakeSession({"action0": [[0.2]]})

    assert inferencer.request_inference({}) is None
    assert inferencer._session.posts == 0


@pytest.mark.parametrize("key", ["instruction", "delay_horizon"])
def test_joint_channel_cannot_shadow_request_form_fields(key):
    with pytest.raises(ValueError, match="conflicts"):
        ObservationConfig(
            channels=[JointStateChannel(server_input_key=key, topic="/state")]
        )


@pytest.mark.parametrize(
    "remaining_key",
    ["state0", "color", "tail0", "instruction", "delay_horizon"],
)
def test_loaded_rtc_keys_cannot_overwrite_observations_or_each_other(
    remaining_key,
):
    config = _joint_config([1, 1]).model_dump(mode="json")
    config["observation_config"]["channels"].append(
        ImageChannel(server_input_key="color", topic="/color").model_dump(
            mode="json"
        )
    )
    config["control_config"]["channels"][1]["server_remaining_key"] = (
        remaining_key
    )

    with pytest.raises(ValueError, match="server_remaining_key.*conflicts"):
        DeployConfig.model_validate_json(json.dumps(config))


@pytest.mark.parametrize(
    "channel_type", [JointStateChannel, JointCommandChannel]
)
def test_removed_server_joint_names_are_not_silently_ignored(channel_type):
    values = {
        "topic": "/joints",
        "joint_names": ["joint1"],
        "server_joint_names": ["left_joint1"],
    }
    key = (
        "server_input_key"
        if channel_type is JointStateChannel
        else "server_output_key"
    )
    values[key] = "joints"

    with pytest.raises(ValueError, match="server_joint_names"):
        channel_type(**values)


def test_unnamed_channels_keep_existing_request(node):
    inferencer = ModelInferencer(node, _config())
    files, data = inferencer._pack_request_data({"color": np.zeros((2, 2, 3))})
    assert set(files) == {"color"}
    assert set(data) == {"instruction", "delay_horizon"}
