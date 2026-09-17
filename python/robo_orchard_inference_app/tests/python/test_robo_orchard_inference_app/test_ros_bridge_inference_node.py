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

import atexit
import subprocess
import sys
import types
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from textwrap import dedent

import pytest
from pydantic import ValidationError

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
from robo_orchard_inference_app.state import (  # noqa: E402
    CollectingState,
    EpisodeMeta,
    InferenceState,
)


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


@pytest.fixture
def monitor(monkeypatch):
    import robo_orchard_inference_app.ros_bridge as bridge

    topics = []
    listeners = {}
    clock = [10.0]

    class Topic:
        def __init__(self, client, name, message_type):
            self.name = name
            self.message_type = message_type
            self.unsubscribed = 0
            topics.append(self)

        def subscribe(self, callback):
            self.callback = callback

        def unsubscribe(self):
            self.unsubscribed += 1

    monkeypatch.setattr(bridge.roslibpy, "Topic", Topic)
    monkeypatch.setattr(bridge.time, "monotonic", lambda: clock[0])
    helper = _make_helper(["/inference_node"])
    helper.ros_client.on = lambda name, callback: listeners.setdefault(
        name, []
    ).append(callback)
    helper.ros_client.off = lambda name, callback: listeners[name].remove(
        callback
    )
    helper.start_status_monitor()
    yield helper, topics, listeners, clock
    helper.cleanup()
    atexit.unregister(helper.cleanup)


def test_initial_inference_state_is_unknown():
    assert InferenceState().is_inference_service_running is None
    assert (
        CollectingState().inference_state.is_inference_service_running is None
    )


@pytest.mark.parametrize("timeout", [0, -1, float("inf"), float("nan")])
def test_status_timeout_must_be_positive_and_finite(timeout):
    with pytest.raises(ValidationError):
        ROSBridgeCfg(status_timeout_s=timeout)


def test_status_monitor_subscribes_once_to_both_nodes(monitor):
    helper, topics, listeners, _clock = monitor
    helper.start_status_monitor()

    assert len(topics) == 2
    assert topics[0].name == "/robot/inference_service/status"
    assert topics[0].message_type == (
        "robo_orchard_deploy_msg_ros2/msg/InferenceStatus"
    )
    assert topics[1].name == "/mcap_recorder_service/status"
    assert topics[1].message_type == (
        "robo_orchard_data_msg_ros2/msg/RecorderStatus"
    )
    assert len(listeners["close"]) == 1
    assert set(listeners) == {"close"}
    assert helper.cfg.status_timeout_s == 5.0
    assert helper.status_snapshot("control") is None
    assert helper.status_snapshot("recorder") is None


def test_status_monitor_honors_configured_topic(monitor):
    helper, topics, _listeners, _clock = monitor
    helper.cleanup()
    helper.cfg.inference_status_topic = "/custom/inference/status"
    helper.start_status_monitor()

    assert topics[-2].name == "/custom/inference/status"
    assert topics[0].unsubscribed == 1


def test_node_status_caches_expire_independently(monitor, monkeypatch):
    helper, topics, listeners, clock = monitor
    topics[0].callback({"data": "enabled"})
    clock[0] += helper.cfg.status_timeout_s
    topics[1].callback({"data": "recording", "session_id": "session"})
    clock[0] += 0.001
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is None
    assert helper.status_snapshot("recorder")["data"] == "recording"

    topics[0].callback({"data": "disabled"})
    helper.ros_client.get_services = lambda: [
        f"{helper.cfg.recorder_name}/stop_recording"
    ]
    monkeypatch.setattr(
        helper, "_call_service_result", lambda **kwargs: (True, None)
    )
    assert helper.stop_recording()
    assert helper.status_snapshot("recorder") is None
    assert helper.status_snapshot("inference") == {"data": "disabled"}

    topics[1].callback({"data": "completed", "session_id": "session"})
    listeners["close"][0](None)
    assert helper.status_snapshot("inference") is None
    assert helper.status_snapshot("recorder") is None


def test_second_subscription_failure_releases_both_topics(
    monitor, monkeypatch
):
    helper, topics, listeners, _clock = monitor
    helper.cleanup()
    topic_type = type(topics[0])
    subscribe = topic_type.subscribe

    def subscribe_or_fail(topic, callback):
        if topic.name == f"{helper.cfg.recorder_name}/status":
            raise RuntimeError("recorder subscription failed")
        subscribe(topic, callback)
        callback({"data": "enabled"})

    monkeypatch.setattr(topic_type, "subscribe", subscribe_or_fail)
    with pytest.raises(RuntimeError, match="recorder subscription failed"):
        helper.start_status_monitor()
    assert all(topic.unsubscribed == 1 for topic in topics)
    assert listeners["close"] == []
    assert helper.status_snapshot("inference") is None
    assert helper._status_topics == []


@pytest.mark.parametrize("failed_topic", ["inference", "recorder"])
def test_topic_construction_failure_does_not_register_listener(
    monitor, monkeypatch, failed_topic
):
    import robo_orchard_inference_app.ros_bridge as bridge

    helper, topics, listeners, _clock = monitor
    helper.cleanup()
    topic_type = type(topics[0])
    failed_name = (
        helper.cfg.inference_status_topic
        if failed_topic == "inference"
        else f"{helper.cfg.recorder_name}/status"
    )

    def create_topic(client, name, message_type):
        if name == failed_name:
            raise RuntimeError("invalid topic")
        return topic_type(client, name, message_type)

    monkeypatch.setattr(bridge.roslibpy, "Topic", create_topic)
    with pytest.raises(RuntimeError, match="invalid topic"):
        helper.start_status_monitor()
    assert listeners["close"] == []
    assert helper._status_topics == []


def test_callback_caches_only_and_ui_refresh_preserves_legacy_control(monitor):
    helper, topics, _listeners, _clock = monitor
    helper.state.control_mode = "takeover"
    helper.state.arm_ctrl_status = "disabled"
    with ThreadPoolExecutor(max_workers=1) as executor:
        executor.submit(topics[0].callback, {"data": "enabled"}).result()

    assert helper.state.is_inference_service_running is None
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is True
    assert helper.state.control_mode == "takeover"
    assert helper.state.arm_ctrl_status == "disabled"


def test_status_uses_receive_time_and_expires(monitor):
    helper, topics, _listeners, clock = monitor
    topics[0].callback({"data": "enabled", "header": {"stamp": {"sec": 0}}})
    clock[0] += helper.cfg.status_timeout_s
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is True
    clock[0] += 0.001
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is None

    topics[0].callback({"data": "disabled"})
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is False


@pytest.mark.parametrize("value", [None, "", "ENABLED", "idle", [], {}, True])
def test_unrecognized_status_is_unknown_not_disabled(monitor, value):
    helper, topics, _listeners, _clock = monitor
    topics[0].callback({"data": "enabled"})
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is True

    topics[0].callback({"data": value})
    helper.refresh_runtime_state()

    assert helper.state.is_inference_service_running is None


def test_disconnect_and_reconnect_require_a_new_snapshot(monitor):
    helper, topics, listeners, _clock = monitor
    topics[0].callback({"data": "enabled"})
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is True

    helper.ros_client.is_connected = False
    listeners["close"][0](None)
    topics[0].callback({"data": "enabled"})
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is None
    helper.ros_client.is_connected = True
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is None

    topics[0].callback({"data": "disabled"})
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is False


def test_snapshot_never_reports_a_disconnected_client(monitor):
    helper, topics, _listeners, _clock = monitor
    topics[0].callback({"data": "enabled"})
    helper.ros_client.is_connected = False
    assert helper.status_snapshot("inference") is None
    helper.ros_client.is_connected = True
    assert helper.status_snapshot("inference") is None


def test_cache_and_snapshots_do_not_share_mutable_message_data(monitor):
    helper, topics, _listeners, _clock = monitor
    message = {"data": "enabled", "header": {"stamp": {"sec": 1}}}
    topics[0].callback(message)
    message["data"] = "disabled"
    message["header"]["stamp"]["sec"] = 2
    snapshot = helper.status_snapshot("inference")
    assert snapshot == {"data": "enabled", "header": {"stamp": {"sec": 1}}}
    snapshot["header"]["stamp"]["sec"] = 3
    assert helper.status_snapshot("inference")["header"]["stamp"]["sec"] == 1


def test_cleanup_is_idempotent_and_ignores_late_callbacks(monitor):
    helper, topics, listeners, _clock = monitor
    topics[0].callback({"data": "enabled"})
    helper.cleanup()
    helper.cleanup()
    topics[0].callback({"data": "enabled"})

    assert topics[0].unsubscribed == 1
    assert listeners["close"] == []
    assert helper.status_snapshot("inference") is None


def test_failed_subscription_releases_resources_and_can_retry(
    monitor, monkeypatch
):
    helper, topics, listeners, _clock = monitor
    helper.cleanup()
    topic_type = type(topics[0])
    subscribe = topic_type.subscribe

    def fail_subscribe(topic, callback):
        raise RuntimeError("subscription failed")

    monkeypatch.setattr(topic_type, "subscribe", fail_subscribe)
    with pytest.raises(RuntimeError, match="subscription failed"):
        helper.start_status_monitor()
    assert topics[-2].unsubscribed == 1
    assert topics[-1].unsubscribed == 0
    assert listeners["close"] == []
    assert helper._status_topics == []

    monkeypatch.setattr(topic_type, "subscribe", subscribe)
    helper.start_status_monitor()
    topics[-2].callback({"data": "enabled"})
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is True


@pytest.mark.parametrize("enable", [False, True])
def test_service_success_does_not_replace_reported_inference_state(
    monitor, monkeypatch, enable
):
    helper, topics, _listeners, _clock = monitor
    calls = []
    helper.state.is_inference_service_running = not enable
    helper.ros_client.get_nodes = lambda: ["/inference_node"]

    def set_param(**kwargs):
        calls.append(("instruction", kwargs))
        return True

    def call_services(**kwargs):
        calls.append(("service", kwargs))
        topics[0].callback({"data": "enabled" if enable else "disabled"})
        if kwargs.get("success_callback"):
            kwargs["success_callback"]()
        return True

    monkeypatch.setattr(helper, "_set_param", set_param)
    monkeypatch.setattr(helper, "_call_services", call_services)
    if enable:
        assert helper.enable_inference(EpisodeMeta(instruction="Pick up"))
        assert calls[0] == (
            "instruction",
            {
                "node_name": "/inference_node",
                "request_data": {
                    "parameters": [
                        {
                            "name": "instruction",
                            "value": {"type": 4, "string_value": "Pick up"},
                        }
                    ]
                },
            },
        )
    else:
        assert helper.disable_inference()
    assert calls[-1][0] == "service"
    assert helper.state.is_inference_service_running is not enable
    helper.refresh_runtime_state()
    assert helper.state.is_inference_service_running is enable


def test_cleanup_releases_recorder_subscription_if_inference_unsubscribe_fails(
    monitor, monkeypatch
):
    helper, topics, _listeners, _clock = monitor
    stops = []

    def stop_recording():
        stops.append(True)
        return True

    def fail_unsubscribe():
        raise RuntimeError("unsubscribe failed")

    monkeypatch.setattr(helper, "stop_recording", stop_recording)
    monkeypatch.setattr(topics[0], "unsubscribe", fail_unsubscribe)
    with pytest.raises(RuntimeError, match="unsubscribe failed"):
        helper.cleanup()
    assert stops == []
    assert topics[1].unsubscribed == 1


def test_cleanup_uses_real_roslibpy_event_and_topic_apis():
    script = dedent("""
        import atexit
        import signal
        import sys
        import types
        from unittest.mock import Mock, PropertyMock, patch

        import roslibpy

        sys.path.insert(0, sys.argv[1])
        sys.modules["robo_orchard_inference_app.version"] = (
            types.SimpleNamespace(
                __version__="0.0.0", __full_version__="0.0.0",
                __git_hash__="test",
            )
        )
        sys.modules["psutil"] = types.SimpleNamespace(signal=signal)
        sys.modules["streamlit"] = types.SimpleNamespace(
            cache_resource=lambda callback: callback,
            session_state=types.SimpleNamespace(),
        )

        from robo_orchard_inference_app.config import ROSBridgeCfg
        from robo_orchard_inference_app.ros_bridge import RosServiceHelper
        from robo_orchard_inference_app.state import InferenceState

        sent = []
        with (
            patch.object(roslibpy.Ros, "connect"),
            patch.object(roslibpy.Ros, "call_later") as call_later,
            patch.object(
                roslibpy.Ros, "send_on_ready",
                lambda client, message: sent.append(dict(message)),
            ),
            patch.object(
                roslibpy.Ros, "is_connected",
                new_callable=PropertyMock, return_value=True,
            ),
        ):
            client = roslibpy.Ros(host="localhost", port=9090)
            helper = RosServiceHelper(
                client, ROSBridgeCfg(), InferenceState(), Mock()
            )
            helper._receive_status = Mock(wraps=helper._receive_status)
            helper._invalidate_status = Mock(wraps=helper._invalidate_status)
            topic = helper.cfg.inference_status_topic
            try:
                helper.start_status_monitor()
                helper.start_status_monitor()
                client.emit(topic, {"data": "enabled"})
                helper.refresh_runtime_state()
                assert helper.state.is_inference_service_running is True
                assert helper._receive_status.call_count == 1
                helper.cleanup()
                helper.cleanup()
                client.emit(topic, {"data": "disabled"})
                client.emit("close", None)
                assert helper._receive_status.call_count == 1
                helper._invalidate_status.assert_not_called()
                call_later.assert_not_called()
                assert [message["op"] for message in sent] == [
                    "subscribe", "subscribe", "unsubscribe", "unsubscribe"
                ]
                helper.start_status_monitor()
                client.emit(topic, {"data": "disabled"})
                assert helper._receive_status.call_count == 2
                helper.refresh_runtime_state()
                assert helper.state.is_inference_service_running is False
            finally:
                helper.cleanup()
                atexit.unregister(helper.cleanup)
    """)
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            script,
            str(Path(__file__).resolve().parents[3]),
        ],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
