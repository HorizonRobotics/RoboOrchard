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

from itertools import count
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robo_orchard_deploy_msg_ros2.msg import InferenceEvent, InferenceStatus
from robo_orchard_deploy_ros2.node import async_node, sync_node


@pytest.fixture(params=[sync_node, async_node], ids=["sync", "async"])
def runtime(request, monkeypatch):
    module = request.param
    messages = {"status": [], "events": []}
    publishers = {}
    timers = []
    services = []
    timestamps = count(100)
    config = SimpleNamespace(
        infer_frequency=10.0,
        control_config=SimpleNamespace(control_frequency=50.0, channels=[]),
        server_url="http://localhost:8000",
        max_delay_horizon=None,
        trajectory_stitch=None,
    )

    def create_publisher(node, message_type, topic, depth):
        publishers[topic] = (message_type, depth)
        return SimpleNamespace(publish=messages[topic].append)

    def create_timer(node, period, callback, *args, **kwargs):
        timer = SimpleNamespace(period=period, callback=callback)
        timers.append(timer)
        return timer

    monkeypatch.setattr(module.Node, "__init__", lambda node, name: None)
    monkeypatch.setattr(
        module.DeployNode,
        "_initialize",
        lambda node: setattr(node, "config", config),
    )
    for name in ("ObservationManager", "ModelInferencer", "ActionExecutor"):
        monkeypatch.setattr(module, name, Mock())
    for name, implementation in {
        "create_publisher": create_publisher,
        "create_timer": create_timer,
        "create_service": lambda node, kind, name, callback: services.append(
            name
        ),
        "get_logger": lambda node: Mock(),
        "get_clock": lambda node: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: next(timestamps))
        ),
    }.items():
        monkeypatch.setattr(module.Node, name, implementation, raising=False)
    node = module.DeployNode()
    return SimpleNamespace(
        node=node,
        module=module,
        messages=messages,
        publishers=publishers,
        timers=timers,
        services=services,
    )


def test_startup_snapshot_and_heartbeat(runtime):
    assert runtime.publishers == {
        "status": (InferenceStatus, 10),
        "events": (InferenceEvent, 10),
    }
    assert runtime.services == ["enable", "disable"]
    assert [message.data for message in runtime.messages["status"]] == [
        InferenceStatus.DISABLED
    ]
    assert runtime.messages["events"] == []
    timer = runtime.node._status_timer
    assert timer in runtime.timers
    assert timer.period == 1.0
    assert timer.callback == runtime.node._publish_status
    first_stamp = runtime.messages["status"][-1].header.stamp

    timer.callback()

    assert runtime.messages["status"][-1].data == InferenceStatus.DISABLED
    assert runtime.messages["status"][-1].header.stamp > first_stamp
    assert runtime.messages["events"] == []


def test_only_real_lifecycle_transitions_emit_events(runtime):
    node = runtime.node
    for _ in range(2):
        assert node._disable_inference_callback(
            None, SimpleNamespace()
        ).success
    assert runtime.messages["events"] == []
    assert len(runtime.messages["status"]) == 1

    for _ in range(2):
        assert node._enable_inference_callback(None, SimpleNamespace()).success
    assert runtime.messages["status"][-1].data == InferenceStatus.ENABLED
    assert len(runtime.messages["status"]) == 2
    assert [message.event_type for message in runtime.messages["events"]] == [
        InferenceEvent.ENABLE_TRIGGERED
    ]
    node._publish_status()
    assert len(runtime.messages["events"]) == 1

    for _ in range(2):
        assert node._disable_inference_callback(
            None, SimpleNamespace()
        ).success
    assert runtime.messages["status"][-1].data == InferenceStatus.DISABLED
    assert len(runtime.messages["status"]) == 4
    assert [message.event_type for message in runtime.messages["events"]] == [
        InferenceEvent.ENABLE_TRIGGERED,
        InferenceEvent.DISABLE_TRIGGERED,
    ]
    assert [message.details for message in runtime.messages["events"]] == [
        "Inference enabled.",
        "Inference disabled.",
    ]
    assert all(
        message.header.stamp is not None
        for message in runtime.messages["events"]
    )


def test_public_state_does_not_depend_on_available_actions(runtime):
    node = runtime.node
    states = runtime.module.NodeState
    enabled_states = [states.EXECUTING]
    disabled_states = [states.PAUSED]
    if runtime.module is sync_node:
        enabled_states.append(states.IDLE)
        disabled_states.append(states.INIT)

    for state in enabled_states:
        node.state = state
        node.current_actions = None
        node._publish_status()
        assert runtime.messages["status"][-1].data == InferenceStatus.ENABLED
    for state in disabled_states:
        node.state = state
        node._publish_status()
        assert runtime.messages["status"][-1].data == InferenceStatus.DISABLED
    assert runtime.messages["events"] == []
