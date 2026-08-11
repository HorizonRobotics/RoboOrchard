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
import threading
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
    def __init__(self):
        self.infos = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.errors.append(message)


class DummyRosClient:
    is_connected = True

    def __init__(self, services):
        self.services = services

    def get_services(self):
        return self.services


def _make_helper(service_names):
    logger = DummyLogger()
    helper = RosServiceHelper(
        ros_client=DummyRosClient(service_names),
        ros_bridge_cfg=ROSBridgeCfg(
            reset_arm_service_name=service_names,
        ),
        inference_state=InferenceState(),
        logger=logger,
    )
    return helper, logger


def test_reset_arm_calls_services_concurrently(monkeypatch):
    import robo_orchard_inference_app.ros_bridge as ros_bridge_module

    service_names = ["/robot/left/reset_ctrl", "/robot/right/reset_ctrl"]
    barrier = threading.Barrier(len(service_names))
    calls = []
    calls_lock = threading.Lock()

    class DummyService:
        def __init__(self, ros_client, service_name, service_type):
            self.service_name = service_name

        def call(self, request, timeout):
            with calls_lock:
                calls.append(self.service_name)
            barrier.wait(timeout=1.0)
            return {"success": True, "message": "ok"}

    monkeypatch.setattr(ros_bridge_module.roslibpy, "Service", DummyService)
    helper, logger = _make_helper(service_names)

    assert helper.reset_arm() is True
    assert set(calls) == set(service_names)
    assert logger.infos == ["Robot arm controllers reset successfully!"]
    assert logger.errors == []


def test_parallel_calls_wait_for_all_services_when_one_fails(monkeypatch):
    import robo_orchard_inference_app.ros_bridge as ros_bridge_module

    service_names = ["/robot/left/reset_ctrl", "/robot/right/reset_ctrl"]
    barrier = threading.Barrier(len(service_names))
    calls = []
    calls_lock = threading.Lock()

    class DummyService:
        def __init__(self, ros_client, service_name, service_type):
            self.service_name = service_name

        def call(self, request, timeout):
            with calls_lock:
                calls.append(self.service_name)
            barrier.wait(timeout=1.0)
            return {
                "success": self.service_name != service_names[0],
                "message": "reset failed",
            }

    monkeypatch.setattr(ros_bridge_module.roslibpy, "Service", DummyService)
    helper, logger = _make_helper(service_names)

    assert helper.reset_arm() is False
    assert set(calls) == set(service_names)
    assert logger.infos == []
    assert logger.errors == [
        "Service /robot/left/reset_ctrl failed: reset failed"
    ]


def test_parallel_calls_preflight_all_services(monkeypatch):
    import robo_orchard_inference_app.ros_bridge as ros_bridge_module

    configured_services = [
        "/robot/left/reset_ctrl",
        "/robot/right/reset_ctrl",
    ]
    calls = []

    class DummyService:
        def __init__(self, ros_client, service_name, service_type):
            calls.append(service_name)

    monkeypatch.setattr(ros_bridge_module.roslibpy, "Service", DummyService)
    helper, logger = _make_helper(configured_services)
    helper.ros_client.services = [configured_services[0]]

    assert helper.reset_arm() is False
    assert calls == []
    assert logger.errors == ["Service /robot/right/reset_ctrl not found!"]
