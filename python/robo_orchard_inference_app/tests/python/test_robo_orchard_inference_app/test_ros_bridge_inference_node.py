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

import sys
import types

roslibpy = types.SimpleNamespace(
    Service=None,
    ServiceRequest=lambda data=None: types.SimpleNamespace(data=data),
    Ros=object,
    Topic=object,
    core=types.SimpleNamespace(RosTimeoutError=RuntimeError),
)
sys.modules["roslibpy"] = roslibpy
sys.modules["robo_orchard_inference_app.version"] = types.SimpleNamespace(
    __full_version__="0.0.0",
    __git_hash__="test",
    __version__="0.0.0",
)
sys.modules["streamlit"] = types.SimpleNamespace(
    session_state=types.SimpleNamespace(
        app_state=types.SimpleNamespace(logs=[])
    ),
    toast=lambda *args, **kwargs: None,
    cache_resource=lambda fn: fn,
)
sys.modules["psutil"] = types.SimpleNamespace(
    signal=types.SimpleNamespace(SIGINT=2, SIGKILL=9),
    pid_exists=lambda pid: False,
    Process=object,
    NoSuchProcess=RuntimeError,
)

from robo_orchard_inference_app.config import ROSBridgeCfg  # noqa: E402
from robo_orchard_inference_app.ros_bridge import (  # noqa: E402
    RosServiceHelper,
)
from robo_orchard_inference_app.state import InferenceState  # noqa: E402


class DummyLogger:
    def info(self, msg):
        pass

    def warning(self, msg):
        pass

    def error(self, msg):
        pass


def _make_helper(candidates):
    return RosServiceHelper(
        ros_client=types.SimpleNamespace(is_connected=True),
        ros_bridge_cfg=ROSBridgeCfg(inference_node_candidates=candidates),
        inference_state=InferenceState(),
        logger=DummyLogger(),
    )


def test_inference_node_active_true_when_candidate_running(monkeypatch):
    helper = _make_helper(["/inference_node", "/other_inference"])
    monkeypatch.setattr(
        helper, "get_node_names", lambda: ["/foo", "/inference_node"]
    )

    assert helper.is_inference_node_active() is True


def test_inference_node_active_false_when_no_candidate_running(monkeypatch):
    helper = _make_helper(["/inference_node"])
    monkeypatch.setattr(helper, "get_node_names", lambda: ["/foo", "/bar"])

    assert helper.is_inference_node_active() is False
