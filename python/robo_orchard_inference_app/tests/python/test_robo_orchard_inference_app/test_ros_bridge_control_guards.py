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

# ruff: noqa: I001

import sys
import types


def _install_stub_modules():
    version = types.ModuleType("robo_orchard_inference_app.version")
    version.__version__ = "0.0.0"
    version.__full_version__ = "0.0.0"
    version.__git_hash__ = "test"
    sys.modules.setdefault("robo_orchard_inference_app.version", version)

    st = types.ModuleType("streamlit")
    st.session_state = types.SimpleNamespace()
    st.toast = lambda *args, **kwargs: None
    st.cache_resource = lambda func: func
    sys.modules.setdefault("streamlit", st)

    roslibpy = types.ModuleType("roslibpy")

    class FakeTimeoutError(Exception):
        pass

    class FakeService:
        def __init__(self, client, name, service_type):
            self.client = client
            self.name = name
            self.service_type = service_type

        def call(self, request, timeout=5.0):
            return self.client.service_results[self.name]

    class FakeServiceRequest(dict):
        def __init__(self, data=None):
            super().__init__(data or {})

    roslibpy.Service = FakeService
    roslibpy.ServiceRequest = FakeServiceRequest
    roslibpy.core = types.SimpleNamespace(RosTimeoutError=FakeTimeoutError)
    roslibpy.Topic = object
    roslibpy.Ros = object
    sys.modules.setdefault("roslibpy", roslibpy)


_install_stub_modules()
sys.path.insert(0, "python/robo_orchard_inference_app")

from robo_orchard_inference_app.config import ROSBridgeCfg  # noqa: E402
from robo_orchard_inference_app.ros_bridge import RosServiceHelper  # noqa: E402
from robo_orchard_inference_app.state import InferenceState  # noqa: E402


class FakeTopic:
    instances = []

    def __init__(self, client, name, message_type):
        self.client = client
        self.name = name
        self.message_type = message_type
        self.callback = None
        FakeTopic.instances.append(self)

    def subscribe(self, callback):
        self.callback = callback

    def unsubscribe(self):
        self.callback = None

    def emit(self, message):
        self.callback(message)


class FakeRosClient:
    def __init__(self, services):
        self.is_connected = True
        self.service_results = {
            service: {"success": True, "message": "ok"} for service in services
        }

    def get_services(self):
        return list(self.service_results.keys())

    def get_nodes(self):
        return []


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.errors.append(message)


def _build_helper(monkeypatch):
    import robo_orchard_inference_app.ros_bridge as ros_bridge_module

    monkeypatch.setattr(ros_bridge_module.roslibpy, "Topic", FakeTopic)

    cfg = ROSBridgeCfg(
        takeover_service_name=[
            "/robot/left/takeover_muxer/trigger_takeover",
            "/robot/right/takeover_muxer/trigger_takeover",
        ],
        release_service_name=[
            "/robot/left/takeover_muxer/release_control",
            "/robot/right/takeover_muxer/release_control",
        ],
        stop_service_name=[
            "/robot/left/takeover_muxer/stop",
            "/robot/right/takeover_muxer/stop",
        ],
        enable_arm_service_name=[
            "/robot/left_master/enable_ctrl",
            "/robot/right_master/enable_ctrl",
        ],
        disable_inference_service_name=["/robot/inference_service/disable"],
        master_status_topics={
            "left": "/master/status_left",
            "right": "/master/status_right",
        },
        master_enable_ctrl_service_names={
            "left": "/robot/left_master/enable_ctrl",
            "right": "/robot/right_master/enable_ctrl",
        },
    )
    all_services = (
        cfg.takeover_service_name
        + cfg.release_service_name
        + cfg.stop_service_name
        + cfg.enable_arm_service_name
        + cfg.disable_inference_service_name
    )
    helper = RosServiceHelper(
        ros_client=FakeRosClient(all_services),
        ros_bridge_cfg=cfg,
        inference_state=InferenceState(),
        logger=FakeLogger(),
    )
    return helper


def _emit_status(side: str, ctrl_mode: int, teach_status: int):
    topic_name = f"/master/status_{side}"
    for topic in FakeTopic.instances:
        if topic.name == topic_name:
            topic.emit({"ctrl_mode": ctrl_mode, "teach_status": teach_status})
            return
    raise AssertionError(f"Topic {topic_name} not found")


def test_takeover_requires_both_arms_in_teach_mode(monkeypatch):
    FakeTopic.instances.clear()
    helper = _build_helper(monkeypatch)
    assert {
        topic.name: topic.message_type for topic in FakeTopic.instances
    } == {
        "/master/status_left": "robo_orchard_piper_msg_ros2/PiperStatusMsg",
        "/master/status_right": "robo_orchard_piper_msg_ros2/PiperStatusMsg",
    }
    _emit_status("left", ctrl_mode=0x02, teach_status=1)
    _emit_status("right", ctrl_mode=0x01, teach_status=0)

    assert helper.takeover_control() is False
    assert helper.state.control_mode == "auto"
    assert helper.logger.errors == [
        "Cannot switch to takeover: right master is idle "
        "(teach button not pressed). "
        "Press the teach button on the master arm to enter teach mode, "
        "then retry."
    ]


def test_takeover_succeeds_when_both_arms_are_in_teach_mode(monkeypatch):
    FakeTopic.instances.clear()
    helper = _build_helper(monkeypatch)
    _emit_status("left", ctrl_mode=0x02, teach_status=1)
    _emit_status("right", ctrl_mode=0x02, teach_status=1)

    assert helper.takeover_control() is True
    assert helper.state.control_mode == "takeover"


def test_auto_rejects_when_any_arm_still_in_teach_mode(monkeypatch):
    FakeTopic.instances.clear()
    helper = _build_helper(monkeypatch)
    _emit_status("left", ctrl_mode=0x02, teach_status=1)
    _emit_status("right", ctrl_mode=0x02, teach_status=2)

    assert helper.release_to_auto() is False
    assert helper.logger.errors == [
        "Cannot switch to auto: left master is in active teach mode "
        "(teach button currently pressed). "
        "Press the teach button on the master arm again to exit teach mode, "
        "then retry."
    ]


def test_teach_status_guard_explains_missing_master_status(monkeypatch):
    FakeTopic.instances.clear()
    helper = _build_helper(monkeypatch)
    _emit_status("left", ctrl_mode=0x02, teach_status=1)

    assert helper.takeover_control() is False
    assert helper.logger.errors == [
        "Cannot switch to takeover: right master status has not been "
        "received yet (is the arm powered and CAN bridge up?). "
        "Press the teach button on the master arm to enter teach mode, "
        "then retry."
    ]


def test_auto_recovers_only_teach_ctrl_mode_arms_before_release(monkeypatch):
    FakeTopic.instances.clear()
    helper = _build_helper(monkeypatch)
    _emit_status("left", ctrl_mode=0x02, teach_status=2)
    _emit_status("right", ctrl_mode=0x01, teach_status=2)

    calls = []
    original = helper._call_services

    def wrapped_call_services(*args, **kwargs):
        service_names = kwargs["service_names"]
        calls.append(list(service_names))
        return original(*args, **kwargs)

    monkeypatch.setattr(helper, "_call_services", wrapped_call_services)

    assert helper.release_to_auto() is True
    assert calls[0] == ["/robot/left_master/enable_ctrl"]
    assert calls[1] == [
        "/robot/left/takeover_muxer/release_control",
        "/robot/right/takeover_muxer/release_control",
    ]


def test_auto_aborts_if_ctrl_mode_recovery_fails(monkeypatch):
    FakeTopic.instances.clear()
    helper = _build_helper(monkeypatch)
    _emit_status("left", ctrl_mode=0x02, teach_status=2)
    _emit_status("right", ctrl_mode=0x01, teach_status=2)
    helper.ros_client.service_results["/robot/left_master/enable_ctrl"] = {
        "success": False,
        "message": "failed",
    }

    assert helper.release_to_auto() is False
    assert helper.state.control_mode == "auto"


def test_teach_mode_detection_reflects_latest_master_status(monkeypatch):
    FakeTopic.instances.clear()
    helper = _build_helper(monkeypatch)
    _emit_status("left", ctrl_mode=0x02, teach_status=1)
    _emit_status("right", ctrl_mode=0x01, teach_status=2)
    assert helper.is_any_master_in_teach_mode() is True

    _emit_status("left", ctrl_mode=0x01, teach_status=0)
    _emit_status("right", ctrl_mode=0x01, teach_status=2)
    assert helper.is_any_master_in_teach_mode() is False


def test_disable_inference_treats_not_running_as_success(monkeypatch):
    FakeTopic.instances.clear()
    helper = _build_helper(monkeypatch)
    helper.state.is_inference_service_running = True
    helper.ros_client.service_results["/robot/inference_service/disable"] = {
        "success": False,
        "message": "inference is not running",
    }

    assert helper.disable_inference() is True
    assert helper.state.is_inference_service_running is False
    assert helper.logger.errors == []


def test_disable_inference_can_ignore_missing_service(monkeypatch):
    FakeTopic.instances.clear()
    helper = _build_helper(monkeypatch)
    helper.state.is_inference_service_running = True
    helper.cfg.disable_inference_service_name = [
        "/robot/inference_service/miss"
    ]

    assert helper.disable_inference(allow_missing_service=True) is True
    assert helper.state.is_inference_service_running is False
    assert helper.logger.errors == []
