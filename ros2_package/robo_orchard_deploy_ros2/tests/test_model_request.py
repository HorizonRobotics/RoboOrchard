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

from robo_orchard_deploy_ros2.config import (
    ControlConfig,
    DeployConfig,
    ImageChannel,
    JointCommandChannel,
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
