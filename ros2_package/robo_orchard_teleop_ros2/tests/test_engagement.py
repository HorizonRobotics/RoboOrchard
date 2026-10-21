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

import importlib.util
import sys
import types
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


@pytest.fixture
def source(monkeypatch):
    from test_robotiq_pico_vr import source as source_fixture

    return source_fixture.__wrapped__(monkeypatch)


@pytest.fixture
def gate(monkeypatch, tmp_path):
    import test_pico_vr_node as ros_stubs

    diagnostics = types.ModuleType("diagnostic_msgs.msg")
    diagnostics.DiagnosticArray = SimpleNamespace

    class DiagnosticStatus(SimpleNamespace):
        OK = 0

    diagnostics.DiagnosticStatus = DiagnosticStatus
    diagnostics.KeyValue = SimpleNamespace
    monkeypatch.setitem(sys.modules, "diagnostic_msgs.msg", diagnostics)
    messages = sys.modules["robo_orchard_teleop_msg_ros2.msg"]
    monkeypatch.setattr(
        messages, "TeleopMatchState", SimpleNamespace, raising=False
    )
    monkeypatch.setattr(
        ros_stubs._Node,
        "resolve_topic_name",
        lambda self, name: name if name.startswith("/") else f"/group/{name}",
        raising=False,
    )
    monkeypatch.setattr(
        ros_stubs._Node,
        "get_fully_qualified_name",
        lambda self: "/group/gate",
        raising=False,
    )
    path = (
        Path(__file__).resolve().parents[1]
        / "robo_orchard_teleop_ros2/engagement.py"
    )
    spec = importlib.util.spec_from_file_location(
        "engagement_under_test", path
    )
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    config = {
        "command_routes": [
            {"input_topic": "a", "output_topic": "out_a"},
            {"input_topic": "b", "output_topic": "out_b"},
        ],
        "match_state_topic": "/match",
        "recapture_service": "/recapture",
    }
    config_path = tmp_path / "engagement.yaml"
    config_path.write_text(yaml.safe_dump(config))
    node = module.EngagementNode(
        parameter_overrides=[
            ros_stubs._Parameter("config_file", str(config_path)),
        ]
    )
    clock = SimpleNamespace(ros=10.0, local=1.0)
    monkeypatch.setattr(module, "monotonic", lambda: clock.local)

    def header(stamp=None):
        sec, nanosec = divmod(
            int((clock.ros if stamp is None else stamp) * 1e9), 1000000000
        )
        return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))

    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(
            nanoseconds=int(clock.ros * 1e9),
            to_msg=lambda: header().stamp,
        )
    )
    requests = []

    def request(_):
        future = Future()
        requests.append(future)
        return future

    node._client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=request
    )

    def advance(seconds=0.01):
        clock.ros += seconds
        clock.local += seconds

    def state(key, value, stamp=None, loss_id=0, candidate_topic="/group/b"):
        message = SimpleNamespace(header=header(stamp))
        if key == "activation":
            message.state = value
        elif key == "manager":
            message.data = value
        else:
            message.ready, message.matched = value
            message.loss_id = loss_id
            message.candidate_topic = candidate_topic
            message.reason = "source"
        node._state_message(key, message)

    def candidate(topic="a", stamp=None):
        message = module.JointState(
            header=header(stamp), name=["joint"], position=[0.1]
        )
        node._candidate(f"/group/{topic}", message)
        return message

    def refresh(activation=2, matched=True):
        advance()
        state("manager", "takeover")
        state("match", (True, matched))
        state("activation", activation)

    def start(matched=True):
        refresh(activation=1, matched=matched)
        refresh(activation=3, matched=matched)
        refresh(matched=matched)
        node._tick()

    def response(future=None, success=True, topic="/group/a", stamp=None):
        (future or requests[-1]).set_result(
            SimpleNamespace(
                success=success,
                candidate_topic=topic,
                stamp_barrier=header(stamp).stamp,
            )
        )
        node._tick()

    def engage():
        start()
        response()
        advance()
        candidate("a")
        candidate("b")
        state("match", (True, True))
        node._tick()
        assert node._phase == "active"

    return SimpleNamespace(
        node=node,
        module=module,
        clock=clock,
        header=header,
        state=state,
        candidate=candidate,
        refresh=refresh,
        start=start,
        response=response,
        advance=advance,
        engage=engage,
        requests=requests,
        config=config,
        config_path=config_path,
        ros_stubs=ros_stubs,
    )


def assert_closed(gate):
    before = [len(pub.messages) for pub in gate.node._outputs.values()]
    gate.node._tick()
    assert [len(pub.messages) for pub in gate.node._outputs.values()] == before
    assert gate.node._phase == "idle"


@pytest.mark.parametrize("failure", ["vr", "joints", "status", "timeout"])
def test_source_loss_survives_recovery_and_skipped_heartbeat(
    gate, source, failure
):
    gate.engage()
    source.node.match_dwell_s = 0.0
    source.node.joint_command_publisher.topic_name = "/group/b"

    def advance(seconds=0.01):
        gate.advance(seconds)
        source.clock.ros = gate.clock.ros
        source.clock.local = gate.clock.local

    def publish(deliver=True):
        source.node._publish()
        heartbeat = source.node.match_state_publisher.messages[-1]
        if deliver:
            gate.state("manager", "takeover")
            gate.state("activation", 2)
            gate.node._state_message("match", heartbeat)
            gate.candidate("a")
            gate.node._candidate(
                "/group/b", source.node.joint_command_publisher.messages[-1]
            )
            gate.node._tick()
        return heartbeat

    advance()
    source.refresh()
    assert publish().loss_id == 0
    if failure == "timeout":
        # Keep downstream streams fresh until just before VR input expiry.
        for seconds in (0.1, 0.1, 0.09):
            advance(seconds)
            source.node._joint_callback(source.joints())
            source.node._status_callback(source.status())
            publish()
        advance(0.02)
    else:
        advance()
        if failure == "vr":
            source.node._vr_callback(source.vr(trigger=float("nan")))
        elif failure == "joints":
            source.node._joint_callback(source.joints(opening=-1.0))
        else:
            source.values["online"] = "false"
            source.node._status_callback(source.status())
            source.values["online"] = "true"
    source.refresh()
    recovered = publish(deliver=False)
    assert recovered.ready and recovered.loss_id == 1
    assert gate.node._phase == "active"
    before = [len(pub.messages) for pub in gate.node._outputs.values()]
    advance()
    source.refresh()
    assert publish().loss_id == 1
    assert_closed(gate)
    assert [len(pub.messages) for pub in gate.node._outputs.values()] == before
    advance()
    source.refresh()
    publish()
    assert_closed(gate)  # Held recovery must not resume the old session.
    gate.state("activation", 1, stamp=gate.clock.ros + 0.001)
    advance()
    source.refresh()
    publish()
    assert gate.node._phase == "recapturing"
    assert len(gate.requests) == 2
    gate.response()
    advance()
    source.refresh()
    publish()
    assert gate.node._phase == "active"


def test_pending_handshake_matching_and_post_barrier_verification(gate):
    gate.candidate("a")
    gate.candidate("b")
    gate.start(matched=False)
    assert gate.node._phase == "matching"
    assert not gate.requests
    gate.refresh(matched=True)
    gate.node._tick()
    assert len(gate.requests) == 1
    assert not any(topic in gate.node._samples for topic in gate.node._outputs)
    barrier = gate.clock.ros
    gate.candidate("a", stamp=barrier - 0.001)
    gate.response(stamp=barrier)
    assert gate.node._phase == "verifying"
    gate.advance()
    a = gate.candidate("a")
    gate.node._tick()
    assert gate.node._phase == "verifying"
    b = gate.candidate("b")
    gate.state("match", (True, True))
    gate.node._tick()
    assert gate.node._phase == "active"
    assert gate.node._outputs["/group/a"].messages[-1] is a
    assert gate.node._outputs["/group/b"].messages[-1] is b
    gate.candidate("a", stamp=barrier)
    gate.node._tick()
    assert gate.node._phase == "active"
    assert gate.node._outputs["/group/a"].messages[-1] is a
    assert len(gate.requests) == 1


@pytest.mark.parametrize("initial", ["startup", "unavailable"])
def test_held_startup_or_fault_never_arms(gate, initial):
    if initial == "unavailable":
        gate.refresh(activation=1)
        gate.refresh(activation=0)
    gate.refresh(activation=3)
    gate.refresh(activation=2)
    assert_closed(gate)
    assert not gate.requests


def test_post_barrier_candidate_before_response_can_be_verified(gate):
    gate.start()
    barrier = gate.clock.ros
    gate.advance()
    gate.candidate("a")
    gate.candidate("b")
    gate.state("match", (True, True))
    assert not any(pub.messages for pub in gate.node._outputs.values())
    gate.response(stamp=barrier)
    assert gate.node._phase == "active"


@pytest.mark.parametrize("first", ["candidate", "match"])
@pytest.mark.parametrize("matched", [True, False])
def test_initial_cutover_requires_matching_candidate_frame(
    gate, source, first, matched
):
    gate.start()
    gate.response()
    # A cached positive match belongs to this older gripper candidate.
    gate.candidate("b")
    gate.advance()
    gate.candidate("a")
    source.clock.ros = gate.clock.ros
    source.clock.local = gate.clock.local
    source.node.match_dwell_s = 0.0
    source.node.joint_command_publisher.topic_name = "/group/b"
    source.refresh(trigger=0.5 if matched else 1.0)
    source.node._publish()
    candidate = source.node.joint_command_publisher.messages[-1]
    match = source.node.match_state_publisher.messages[-1]
    deliveries = {
        "candidate": lambda: gate.node._candidate("/group/b", candidate),
        "match": lambda: gate.node._state_message("match", match),
    }
    deliveries[first]()
    gate.node._tick()
    assert not any(pub.messages for pub in gate.node._outputs.values())
    assert gate.node._phase != "active"
    deliveries["match" if first == "candidate" else "candidate"]()
    gate.node._tick()
    if matched:
        assert gate.node._phase == "active"
        assert gate.node._outputs["/group/b"].messages == [candidate]
        assert len(gate.node._outputs["/group/a"].messages) == 1
    else:
        assert_closed(gate)
        assert not any(pub.messages for pub in gate.node._outputs.values())


@pytest.mark.parametrize("topic", ["", "/unknown", "b"])
def test_initial_match_must_identify_a_resolved_configured_route(gate, topic):
    gate.start()
    gate.response()
    gate.advance()
    gate.candidate("a")
    gate.candidate("b")
    gate.state("match", (True, True), candidate_topic=topic)
    gate.node._tick()
    assert gate.node._phase == "verifying"
    assert not any(pub.messages for pub in gate.node._outputs.values())
    gate.advance(0.31)
    assert_closed(gate)


def test_barrier_cannot_precede_cached_old_baseline_stamp(gate):
    gate.refresh(activation=1)
    gate.refresh(activation=3)
    gate.refresh()
    gate.candidate("a")
    old_stamp = gate.clock.ros
    gate.node._tick()
    gate.response(stamp=old_stamp - 0.001)
    assert_closed(gate)


def test_active_mismatch_keeps_group_open_without_another_recapture(gate):
    gate.engage()
    gate.refresh(matched=False)
    gate.candidate("a")
    gate.candidate("b")
    gate.node._tick()
    assert gate.node._phase == "active"
    assert len(gate.requests) == 1


@pytest.mark.parametrize(
    "loss",
    [
        "release",
        "activation",
        "manager",
        "match",
        "stream",
        "stale",
        "regression",
    ],
)
def test_loss_closes_all_routes_and_requires_release(gate, loss):
    gate.engage()
    gate.advance()
    if loss == "release":
        gate.state("activation", 1)
    elif loss == "activation":
        gate.state("activation", 0)
    elif loss == "manager":
        gate.state("manager", "stop")
    elif loss == "match":
        gate.state("match", (False, False))
    elif loss == "regression":
        gate.candidate("b", stamp=10.035)
    elif loss == "stale":
        gate.advance(1.0)
    else:
        gate.advance(0.3)
        gate.refresh()
    assert_closed(gate)
    if loss != "release":
        gate.refresh()
        assert_closed(gate)
        assert len(gate.requests) == 1
        gate.refresh(activation=1)
    gate.refresh(activation=3)
    gate.refresh()
    gate.node._tick()
    assert len(gate.requests) == 2


@pytest.mark.parametrize(
    "failure",
    [
        "false",
        "topic",
        "zero",
        "future",
        "exception",
        "timeout",
        "unavailable",
    ],
)
def test_recapture_failure_requires_release(gate, failure):
    if failure == "unavailable":
        gate.node._client.service_is_ready = lambda: False
    gate.start()
    if failure == "unavailable":
        assert_closed(gate)
        return
    if failure == "exception":
        gate.requests[-1].set_exception(RuntimeError("response failed"))
        gate.node._tick()
    elif failure == "timeout":
        gate.advance(2.1)
        gate.node._tick()
    else:
        gate.response(
            success=failure != "false",
            topic="/unknown" if failure == "topic" else "/group/a",
            stamp=0.0
            if failure == "zero"
            else (gate.clock.ros + 1.0 if failure == "future" else None),
        )
    assert_closed(gate)
    gate.refresh()
    assert_closed(gate)


def test_late_service_result_cannot_replace_new_attempt(gate):
    gate.start()
    old = gate.requests[-1]
    gate.refresh(activation=1)
    gate.refresh()
    gate.node._tick()
    new = gate.requests[-1]
    assert new is not old
    old.set_result(SimpleNamespace(success=True))
    gate.node._recapture_result(old)
    assert gate.node._future is new
    assert gate.node._phase == "recapturing"
    gate.response(future=new)
    assert gate.node._phase == "verifying"


def test_match_loss_during_recapture_and_verification_timeout(gate):
    gate.start()
    gate.refresh(matched=False)
    assert_closed(gate)
    gate.refresh(activation=1)
    gate.refresh()
    gate.node._tick()
    gate.response()
    gate.advance(2.1)
    assert_closed(gate)


def test_transient_match_loss_before_timer_invalidates_cutover(gate):
    gate.start()
    gate.refresh(matched=False)
    gate.refresh(matched=True)
    assert_closed(gate)
    assert gate.node._future is None


def test_payload_forwarding_preserves_optional_joint_state_arrays(gate):
    gate.engage()
    gate.advance()
    message = gate.module.JointState(
        header=gate.header(),
        name=["joint"],
        position=[0.2],
        velocity=[0.3],
        effort=[0.4],
    )
    gate.node._candidate("/group/a", message)
    gate.node._tick()
    assert gate.node._outputs["/group/a"].messages[-1] is message


def test_pending_does_not_hide_stale_match_or_manager_recovery(gate):
    gate.refresh(activation=1)
    gate.refresh(activation=3)
    gate.advance(0.4)
    gate.refresh(activation=3)
    gate.refresh()
    assert_closed(gate)
    assert not gate.requests


def test_default_one_hz_manager_allows_long_hold_and_engagement(gate):
    gate.refresh(activation=1)
    assert gate.node.manager_status_timeout_s == 2.5
    # Source heartbeats arrive at 30 Hz, Manager at its public default 1 Hz.
    for tick in range(1, 100):
        gate.advance(1.0 / 30.0)
        if tick % 30 == 0:
            gate.state("manager", "takeover")
        gate.state("match", (True, True))
        gate.state("activation", 3 if tick <= 40 else 2)
        gate.candidate("a")
        gate.candidate("b")
        gate.node._tick()
        if gate.node._phase == "recapturing":
            gate.response()
        if tick > 41:
            assert gate.node._phase == "active"
        elif tick <= 40:
            assert gate.node._armed
    assert len(gate.requests) == 1


@pytest.mark.parametrize("age_clock", ["local", "ros"])
def test_manager_timeout_checks_both_clocks_and_requires_rearm(
    gate, age_clock
):
    gate.engage()
    # Other streams remain fresh even when one Manager age clock expires.
    for tick in range(26):
        gate.advance(0.001)
        setattr(gate.clock, age_clock, getattr(gate.clock, age_clock) + 0.1)
        gate.state("match", (True, True))
        gate.state("activation", 2)
        gate.candidate("a")
        gate.candidate("b")
        gate.node._tick()
        if tick < 24:
            assert gate.node._phase == "active"
    assert not gate.node._fresh("manager")
    assert all(gate.node._fresh(key) for key in ("activation", "match"))
    assert_closed(gate)
    gate.refresh()
    assert_closed(gate)
    gate.refresh(activation=1)
    gate.refresh()
    gate.node._tick()
    assert len(gate.requests) == 2


def test_manager_stamp_acceptance_uses_separate_timeout(gate):
    gate.state("manager", "takeover", stamp=gate.clock.ros - 1.0)
    assert gate.node._fresh("manager")
    gate.state("match", (True, True), stamp=gate.clock.ros - 1.0)
    assert not gate.node._fresh("match")
    gate.advance()
    gate.state("manager", "takeover", stamp=gate.clock.ros - 3.0)
    assert not gate.node._fresh("manager")


@pytest.mark.parametrize("value", [0.0, -1.0, float("nan")])
def test_manager_timeout_requires_positive_finite_value(gate, value):
    with pytest.raises(ValueError, match="manager_status_timeout_s"):
        gate.module.EngagementNode(
            parameter_overrides=[
                gate.ros_stubs._Parameter(
                    "config_file", str(gate.config_path)
                ),
                gate.ros_stubs._Parameter("manager_status_timeout_s", value),
            ]
        )


def test_missing_candidates_before_activation_never_deadlocks(gate):
    gate.start()
    assert gate.node._phase == "recapturing"
    assert len(gate.requests) == 1


@pytest.mark.parametrize("key", ["activation", "manager", "match"])
def test_invalid_heartbeat_cannot_renew_active_session(gate, key):
    gate.engage()
    gate.advance()
    value = {"activation": 2, "manager": "takeover", "match": (True, True)}
    gate.state(key, value[key], stamp=0.0)
    assert_closed(gate)


@pytest.mark.parametrize(
    "invalid",
    ["nan", "names", "length", "zero_stamp", "old_stamp", "future_stamp"],
)
def test_invalid_candidate_closes_the_group(gate, invalid):
    gate.engage()
    gate.advance()
    message = gate.module.JointState(
        header=gate.header(), name=["joint"], position=[0.1]
    )
    if invalid == "nan":
        message.position[0] = float("nan")
    elif invalid == "names":
        message.name = [""]
    elif invalid == "length":
        message.position = []
    else:
        message.header = gate.header(
            {
                "zero_stamp": 0.0,
                "old_stamp": 9.0,
                "future_stamp": 11.0,
            }[invalid]
        )
    gate.node._candidate("/group/b", message)
    assert_closed(gate)


@pytest.mark.parametrize(
    "change",
    ["empty", "missing", "self", "duplicate", "cross", "resolved_duplicate"],
)
def test_config_validation(gate, change):
    config = gate.config
    if change == "empty":
        config["command_routes"] = []
    elif change == "missing":
        del config["recapture_service"]
    elif change == "self":
        config["command_routes"][0]["output_topic"] = "a"
    elif change == "duplicate":
        config["command_routes"][1]["output_topic"] = "out_a"
    elif change == "cross":
        config["command_routes"][1]["output_topic"] = "a"
    else:
        config["command_routes"][1]["input_topic"] = "/group/a"
    gate.config_path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError):
        gate.module.EngagementNode(
            parameter_overrides=[
                gate.ros_stubs._Parameter(
                    "config_file", str(gate.config_path)
                ),
            ]
        )
