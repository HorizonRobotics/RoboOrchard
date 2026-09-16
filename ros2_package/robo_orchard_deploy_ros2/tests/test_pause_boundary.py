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

"""What survives a pause, in both deploy nodes.

Nothing should. State kept across the boundary is a claim about where the
robot is, and a pause is exactly the interval in which that claim stops
being supported. Both nodes publish through the same ActionExecutor, so a
guard wired into only one of them is a guard with a hole in it.
"""

from __future__ import annotations
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robo_orchard_deploy_ros2.node import async_node, sync_node


class _Logger:
    def __init__(self):
        self.debug_messages = []

    def debug(self, message, **kwargs):
        self.debug_messages.append(message)

    def warning(self, message, **kwargs):
        pass

    def error(self, message, **kwargs):
        pass

    def info(self, message, **kwargs):
        pass


class _Executor:
    def __init__(self):
        self.resets = 0

    def reset_limiter(self):
        self.resets += 1


def _async():
    node = async_node.DeployNode.__new__(async_node.DeployNode)
    node.shared_state_lock = threading.Lock()
    node._inference_generation = 0
    node._status_publisher = Mock()
    node._event_publisher = Mock()
    node.get_clock = Mock()
    node.state = async_node.NodeState.EXECUTING
    node.current_actions = {"actions": [[1.0]]}
    node.current_action_idx = 7
    node._chunk_start_idx = 3
    node._pending_actions = (10, {"actions": [[2.0]]}, 0)
    node.action_executor = _Executor()
    node._stitcher = type("_S", (), {"resets": 0, "reset": lambda s: None})()
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    return node


def _sync():
    node = sync_node.DeployNode.__new__(sync_node.DeployNode)
    node.state_lock = threading.Lock()
    node._inference_generation = 0
    node._status_publisher = Mock()
    node._event_publisher = Mock()
    node.get_clock = Mock()
    node.state = sync_node.NodeState.EXECUTING
    node.current_actions = {"actions": [[1.0]]}
    node.current_action_index = 7
    node.action_executor = _Executor()
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    return node


class _Response:
    success = None
    message = None


@pytest.mark.parametrize("build", [_async, _sync], ids=["async", "sync"])
def test_pausing_drops_the_limiter_reference(build):
    node = build()

    node._disable_inference_callback(None, _Response())

    assert node.action_executor.resets == 1
    assert node.current_actions is None


@pytest.mark.parametrize("build", [_async, _sync], ids=["async", "sync"])
def test_resuming_drops_the_limiter_reference(build):
    node = build()
    node._disable_inference_callback(None, _Response())

    node._enable_inference_callback(None, _Response())

    assert node.action_executor.resets == 2
    assert node.current_actions is None


def test_pausing_drops_the_live_trajectory():
    node = _async()
    dropped = []
    node._stitcher = type(
        "_S", (), {"reset": lambda s: dropped.append(True)}
    )()

    node._disable_inference_callback(None, _Response())

    assert dropped == [True]
    assert node._chunk_start_idx == 0
    assert node.current_action_idx == 0
    assert node._pending_actions is None


def test_resuming_drops_a_pending_handover():
    node = _async()
    node.state = async_node.NodeState.PAUSED

    node._enable_inference_callback(None, _Response())

    assert node._pending_actions is None


@pytest.mark.parametrize(
    ("build", "action_index"),
    [(_async, "current_action_idx"), (_sync, "current_action_index")],
    ids=["async", "sync"],
)
def test_repeated_disable_still_invalidates_and_clears_state(
    build, action_index
):
    node = build()
    node._disable_inference_callback(None, _Response())
    assert getattr(node, action_index) == 0
    node.current_actions = {"actions": [[2.0]]}
    setattr(node, action_index, 1)
    node._disable_inference_callback(None, _Response())

    assert node._inference_generation == 2
    assert node.current_actions is None
    assert getattr(node, action_index) == 0
    assert node.action_executor.resets == 2


@pytest.mark.parametrize("resume", [False, True])
def test_sync_late_response_cannot_restore_actions(resume):
    node = _sync()
    node.state = sync_node.NodeState.IDLE
    node.current_actions = None
    node.current_action_index = 0
    node.obs_manager = SimpleNamespace(
        get_observations=lambda: {"observation": [1.0]}
    )

    def delayed_response(observations):
        node._disable_inference_callback(None, _Response())
        if resume:
            node._enable_inference_callback(None, _Response())
        return {"actions": [[0.2]]}

    node.model_inferencer = SimpleNamespace(request_inference=delayed_response)
    node._model_infer_callback()

    assert node.current_actions is None
    assert node.current_action_index == 0
    assert node._inference_generation == (2 if resume else 1)
    assert node._logger.debug_messages == [
        "Discarding stale inference response."
    ]
    if resume:
        node.model_inferencer.request_inference = lambda obs: {
            "actions": [[0.3]]
        }
        node._model_infer_callback()
        assert node.current_actions == {"actions": [[0.3]]}
        assert node.state == sync_node.NodeState.EXECUTING


def test_disabling_sync_initial_state_also_clears_actions():
    node = _sync()
    node.state = sync_node.NodeState.INIT

    response = node._disable_inference_callback(None, _Response())

    assert response.success
    assert node.current_actions is None
    assert node.current_action_index == 0
    assert node._inference_generation == 1
