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
import importlib.util
from pathlib import Path

import test_pico_vr_node as pico_stubs

MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "robo_orchard_teleop_ros2"
    / "bridge"
    / "pico"
    / "intent"
    / "activation.py"
)


class _Clock:
    def __init__(self, now: float = 0.0):
        self.now = now

    def __call__(self) -> float:
        return self.now


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "topic_activation_intent_under_test", MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _message(state: int):
    message_type = pico_stubs.sys.modules[
        "robo_orchard_teleop_msg_ros2.msg"
    ].TeleopActivationState
    return message_type(state=state)


def test_active_requires_an_inactive_heartbeat_first():
    module = _load_module()
    clock = _Clock()
    intent = module.TopicActivationIntent(timeout_s=0.2, monotonic=clock)

    intent.update(_message(module.TeleopActivationState.ACTIVE))
    assert intent.status() == (False, "release_required")
    assert intent.is_pressed(object())

    intent.update(_message(module.TeleopActivationState.INACTIVE))
    intent.update(_message(module.TeleopActivationState.ACTIVE))
    assert intent.status() == (True, "active")


def test_timeout_and_explicit_rearm_fail_closed():
    module = _load_module()
    clock = _Clock()
    intent = module.TopicActivationIntent(timeout_s=0.2, monotonic=clock)
    intent.update(_message(module.TeleopActivationState.INACTIVE))
    intent.update(_message(module.TeleopActivationState.ACTIVE))

    clock.now = 0.21
    assert intent.status() == (False, "activation_heartbeat_timeout")
    assert not intent.is_pressed(object())

    clock.now = 0.22
    intent.update(_message(module.TeleopActivationState.ACTIVE))
    assert intent.status() == (False, "release_required")

    intent.update(_message(module.TeleopActivationState.INACTIVE))
    intent.update(_message(module.TeleopActivationState.ACTIVE))
    intent.require_rearm()
    assert intent.status() == (False, "release_required")
