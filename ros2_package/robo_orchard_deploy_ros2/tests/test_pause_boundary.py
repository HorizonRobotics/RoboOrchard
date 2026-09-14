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

import pytest

from robo_orchard_deploy_ros2.node import async_node, sync_node


class _Logger:
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
    node.state = async_node.NodeState.EXECUTING
    node.current_actions = {"actions": [[1.0]]}
    node.current_action_idx = 7
    node._chunk_start_idx = 3
    node._pending_actions = (10, {"actions": [[2.0]]}, 0)
    node.action_executor = _Executor()
    node._stitcher = type("_S", (), {"resets": 0, "reset": lambda s: None})()
    node.get_logger = lambda: _Logger()
    return node


def _sync():
    node = sync_node.DeployNode.__new__(sync_node.DeployNode)
    node.state_lock = threading.Lock()
    node.state = sync_node.NodeState.EXECUTING
    node.current_actions = {"actions": [[1.0]]}
    node.current_action_index = 7
    node.action_executor = _Executor()
    node.get_logger = lambda: _Logger()
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
