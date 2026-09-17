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
from pathlib import Path
from textwrap import dedent

import pytest
from pydantic import ValidationError
from test_ros_bridge_inference_node import _make_helper


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
    helper.ros_client.get_services = lambda: [
        f"{helper.cfg.recorder_name}/start_recording",
        f"{helper.cfg.recorder_name}/stop_recording",
    ]
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


@pytest.mark.parametrize("timeout", [0, -1, float("inf"), float("nan")])
def test_status_timeout_must_be_positive_and_finite(timeout):
    from robo_orchard_inference_app.config import ROSBridgeCfg

    with pytest.raises(ValidationError):
        ROSBridgeCfg(status_timeout_s=timeout)


def test_snapshot_never_reports_a_disconnected_client(monitor):
    helper, topics, _listeners, _clock = monitor
    topics[1].callback({"data": "recording"})
    helper.ros_client.is_connected = False
    assert helper.status_snapshot("recorder") is None
    helper.ros_client.is_connected = True
    assert helper.status_snapshot("recorder") is None


def test_monitor_subscribes_once_to_recorder_with_inference(monitor):
    helper, topics, listeners, _clock = monitor
    helper.start_status_monitor()
    assert len(topics) == 2
    assert topics[1].name == f"{helper.cfg.recorder_name}/status"
    assert (
        topics[1].message_type
        == "robo_orchard_data_msg_ros2/msg/RecorderStatus"
    )
    assert len(listeners["close"]) == 1
    assert helper.status_snapshot("recorder") is None
    helper.cleanup()
    helper.cfg.recorder_name = "/custom_recorder"
    helper.start_status_monitor()
    assert topics[-1].name == "/custom_recorder/status"


def test_recorder_freshness_uses_receive_time_and_disconnect_clears_cache(
    monitor,
):
    helper, topics, listeners, clock = monitor
    state = helper.state.model_dump()
    topics[1].callback({"data": "recording", "header": {"stamp": {"sec": 0}}})
    clock[0] += helper.cfg.status_timeout_s
    assert helper.status_snapshot("recorder")["data"] == "recording"
    clock[0] += 0.001
    assert helper.status_snapshot("recorder") is None
    topics[1].callback({"data": "completed"})
    listeners["close"][0](None)
    assert helper.status_snapshot("recorder") is None
    topics[1].callback({"data": "idle"})
    assert helper.status_snapshot("recorder")["data"] == "idle"
    assert helper.state.model_dump() == state


@pytest.mark.parametrize("success", [False, True])
def test_recorder_calls_preserve_status_received_during_request(
    monitor, monkeypatch, success
):
    helper = monitor[0]
    for call, status in (
        (lambda: helper.start_recording("/episode"), "waiting"),
        (helper.stop_recording, "completed"),
    ):
        helper._receive_status("recorder", {"data": "idle"})

        def respond(status=status, **kwargs):
            assert helper.status_snapshot("recorder") is None
            helper._receive_status("recorder", {"data": status})
            return success

        monkeypatch.setattr(helper, "_call_services", respond)
        monkeypatch.setattr(
            helper,
            "_call_service_result",
            lambda **kwargs: (respond(**kwargs), None),
        )
        assert call() is success
        assert helper.status_snapshot("recorder")["data"] == status


def test_service_acknowledgement_does_not_replace_node_status(
    monitor, monkeypatch
):
    helper, topics, _listeners, _clock = monitor
    monkeypatch.setattr(helper, "_call_services", lambda **kwargs: True)
    monkeypatch.setattr(
        helper, "_call_service_result", lambda **kwargs: (True, None)
    )
    topics[1].callback({"data": "idle"})
    assert helper.start_recording("/episode")
    assert helper.status_snapshot("recorder") is None
    topics[1].callback({"data": "recording"})
    assert helper.stop_recording()
    assert helper.status_snapshot("recorder") is None


@pytest.mark.parametrize("terminal", ["completed", "idle", "failed"])
@pytest.mark.parametrize("later", ["another_session", "disconnect", "stale"])
def test_stop_result_survives_newer_or_unavailable_live_status(
    monitor, monkeypatch, terminal, later
):
    helper, topics, listeners, clock = monitor
    monkeypatch.setattr(
        helper, "_call_service_result", lambda **kwargs: (None, None)
    )
    assert helper.stop_recording(session=("owned-session", "/episode")) is None
    topics[1].callback(
        {
            "data": terminal,
            "session_id": "another-session",
            "destination": "/episode",
        }
    )
    assert helper.recorder_stop_result("owned-session", "/episode") is None
    completed = {
        "data": terminal,
        "session_id": "owned-session",
        "destination": "/episode",
    }
    topics[1].callback(completed)
    completed["data"] = "mutated"
    if later == "another_session":
        topics[1].callback(
            {
                "data": "completed",
                "session_id": "another-session",
                "destination": "/another-episode",
            }
        )
        assert helper.status_snapshot("recorder")["session_id"] == (
            "another-session"
        )
    elif later == "disconnect":
        helper.ros_client.is_connected = False
        listeners["close"][0]()
        assert helper.status_snapshot("recorder") is None
    else:
        clock[0] += helper.cfg.status_timeout_s + 1
        assert helper.status_snapshot("recorder") is None

    assert helper.recorder_stop_result("other-session", "/episode") is None
    assert helper.recorder_stop_result("owned-session", "/other") is None
    assert helper.recorder_stop_result("owned-session", "/episode") == {
        "data": terminal,
        "session_id": "owned-session",
        "destination": "/episode",
    }
    assert helper.recorder_stop_result("owned-session", "/episode") is None


def test_stop_preserves_completion_received_just_before_request(
    monitor, monkeypatch
):
    helper, topics, _listeners, _clock = monitor
    monkeypatch.setattr(
        helper, "_call_service_result", lambda **kwargs: (True, None)
    )
    completed = {
        "data": "completed",
        "session_id": "owned-session",
        "destination": "/episode",
    }
    topics[1].callback(completed)
    assert helper.stop_recording(session=("owned-session", "/episode"))
    assert helper.status_snapshot("recorder") is None
    assert helper.recorder_stop_result("owned-session", "/episode") == (
        completed
    )


def test_repeated_stop_does_not_erase_received_completion(
    monitor, monkeypatch
):
    helper, topics, _listeners, _clock = monitor
    monkeypatch.setattr(
        helper, "_call_service_result", lambda **kwargs: (True, None)
    )
    assert helper.stop_recording(session=("owned-session", "/episode"))
    topics[1].callback(
        {
            "data": "completed",
            "session_id": "owned-session",
            "destination": "/episode",
        }
    )
    topics[1].callback({"data": "recording", "session_id": "another-session"})
    monkeypatch.setattr(
        helper, "_call_service_result", lambda **kwargs: (False, None)
    )
    assert (
        helper.stop_recording(session=("owned-session", "/episode")) is False
    )
    assert (
        helper.recorder_stop_result("owned-session", "/episode")["data"]
        == "completed"
    )


@pytest.mark.parametrize("operation", ["start", "cleanup"])
def test_new_start_or_cleanup_releases_previous_stop_tracking(
    monitor, monkeypatch, operation
):
    helper, topics, _listeners, _clock = monitor
    monkeypatch.setattr(
        helper, "_call_service_result", lambda **kwargs: (True, None)
    )
    assert helper.stop_recording(session=("owned-session", "/episode"))
    topics[1].callback(
        {
            "data": "completed",
            "session_id": "owned-session",
            "destination": "/episode",
        }
    )
    if operation == "start":
        assert helper.start_recording("/new-episode")
    else:
        helper.cleanup()
    assert helper.recorder_stop_result("owned-session", "/episode") is None


@pytest.mark.parametrize(
    "outcome, expected",
    [
        ("accepted", True),
        ("rejected", False),
        ("timeout", None),
        ("transport_error", None),
        ("missing_success", None),
        ("invalid_success", None),
        ("invalid_reply", None),
    ],
)
@pytest.mark.parametrize("operation", ["start", "stop"])
def test_recording_request_distinguishes_rejection_from_unknown_dispatch(
    monitor, monkeypatch, outcome, expected, operation
):
    import robo_orchard_inference_app.ros_bridge as bridge

    helper = monitor[0]
    calls = []

    def call(request, timeout):
        calls.append((request, timeout))
        if outcome == "timeout":
            raise bridge.roslibpy.core.RosTimeoutError("reply lost")
        if outcome == "transport_error":
            raise OSError("connection lost")
        return {
            "accepted": {"success": True},
            "rejected": {"success": False, "message": "busy"},
            "missing_success": {},
            "invalid_success": {"success": "true"},
            "invalid_reply": None,
        }[outcome]

    monkeypatch.setattr(
        bridge.roslibpy,
        "Service",
        lambda *args: types.SimpleNamespace(call=call),
    )
    monkeypatch.setattr(bridge.roslibpy, "ServiceRequest", dict)

    if operation == "start":
        assert helper.start_recording("/episode") is expected
        assert calls == [({"destination": "/episode"}, 5.0)]
    else:
        assert helper.stop_recording() is expected
        assert calls == [({}, 5.0)]
    assert helper._call_service(
        service_name="/legacy-service",
        timeout=5.0,
        service_type="std_srvs/srv/Trigger",
        request_data={},
    ) is (expected is True)


@pytest.mark.parametrize(
    "failure",
    ["disconnected", "missing", "discovery", "invalid_list", "prepare"],
)
@pytest.mark.parametrize("operation", ["start", "stop"])
def test_recording_request_failure_before_dispatch_is_definitive(
    monitor, monkeypatch, failure, operation
):
    import robo_orchard_inference_app.ros_bridge as bridge

    helper = monitor[0]
    calls = []

    def service(*args):
        if failure == "prepare":
            raise ValueError("cannot construct request")
        return types.SimpleNamespace(
            call=lambda *args, **kwargs: calls.append(True)
        )

    def discover():
        raise bridge.roslibpy.core.RosTimeoutError("discovery timeout")

    monkeypatch.setattr(bridge.roslibpy, "Service", service)
    if failure == "disconnected":
        helper.ros_client.is_connected = False
    elif failure == "missing":
        helper.ros_client.get_services = lambda: []
    elif failure == "discovery":
        helper.ros_client.get_services = discover
    elif failure == "invalid_list":
        helper.ros_client.get_services = lambda: None

    if operation == "start":
        assert helper.start_recording("/episode") is False
    else:
        assert helper.stop_recording() is False
    assert calls == []


def test_cleanup_does_not_stop_recorder_even_when_unsubscribe_fails(
    monitor, monkeypatch
):
    helper, topics, _listeners, _clock = monitor
    calls = []
    monkeypatch.setattr(helper, "stop_recording", lambda: calls.append("stop"))
    topics[1].callback({"data": "recording"})

    def fail():
        raise RuntimeError("unsubscribe failed")

    monkeypatch.setattr(topics[1], "unsubscribe", fail)
    with pytest.raises(RuntimeError, match="unsubscribe failed"):
        helper.cleanup()
    assert calls == []
    assert helper.status_snapshot("recorder") is None


def test_cache_and_snapshots_do_not_share_mutable_message_data(monitor):
    helper, topics, _listeners, _clock = monitor
    message = {"data": "recording", "header": {"stamp": {"sec": 1}}}
    topics[1].callback(message)
    message["data"] = "idle"
    message["header"]["stamp"]["sec"] = 2
    snapshot = helper.status_snapshot("recorder")
    assert snapshot == {"data": "recording", "header": {"stamp": {"sec": 1}}}
    snapshot["header"]["stamp"]["sec"] = 3
    assert helper.status_snapshot("recorder")["header"]["stamp"]["sec"] == 1


def test_cleanup_is_idempotent_and_ignores_late_callbacks(monitor):
    helper, topics, listeners, _clock = monitor
    topics[1].callback({"data": "recording"})
    helper.cleanup()
    helper.cleanup()
    topics[1].callback({"data": "recording"})

    assert all(topic.unsubscribed == 1 for topic in topics)
    assert listeners["close"] == []
    assert helper.status_snapshot("recorder") is None


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
    topics[-1].callback({"data": "recording"})
    assert helper.status_snapshot("recorder")["data"] == "recording"


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
            topic = f"{helper.cfg.recorder_name}/status"
            try:
                helper.start_status_monitor()
                helper.start_status_monitor()
                client.emit(topic, {"data": "recording"})
                assert (
                    helper.status_snapshot("recorder")["data"] == "recording"
                )
                assert helper._receive_status.call_count == 1
                helper.cleanup()
                helper.cleanup()
                client.emit(topic, {"data": "idle"})
                client.emit("close", None)
                assert helper._receive_status.call_count == 1
                helper._invalidate_status.assert_not_called()
                call_later.assert_not_called()
                assert [message["op"] for message in sent] == [
                    "subscribe", "subscribe", "unsubscribe", "unsubscribe"
                ]
                helper.start_status_monitor()
                client.emit(topic, {"data": "idle"})
                assert helper._receive_status.call_count == 2
                assert helper.status_snapshot("recorder")["data"] == "idle"
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
