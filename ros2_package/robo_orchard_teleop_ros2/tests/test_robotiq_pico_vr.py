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
from pathlib import Path
from types import SimpleNamespace

import pytest
import test_pico_vr_node as ros_stubs


@pytest.fixture
def source(monkeypatch):
    """Reuse node/message stubs, adding only these two new message types."""
    diagnostics = types.ModuleType("diagnostic_msgs.msg")
    diagnostics.DiagnosticArray = SimpleNamespace
    diagnostics.DiagnosticStatus = SimpleNamespace(OK=0)
    monkeypatch.setitem(sys.modules, "diagnostic_msgs.msg", diagnostics)
    teleop_messages = sys.modules["robo_orchard_teleop_msg_ros2.msg"]
    monkeypatch.setattr(
        teleop_messages, "TeleopMatchState", SimpleNamespace, raising=False
    )
    path = (
        Path(__file__).resolve().parents[1]
        / "robo_orchard_teleop_ros2/robot/robotiq/pico_vr.py"
    )
    spec = importlib.util.spec_from_file_location("robotiq_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = SimpleNamespace(ros=10.0, local=1.0)
    monkeypatch.setattr(module, "monotonic", lambda: clock.local)
    node = module.RobotiqPicoVRTeleOpNode()
    node.joint_command_publisher.topic_name = "/robotiq_teleop/candidate"
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(
            nanoseconds=int(clock.ros * 1e9),
            to_msg=lambda: header().stamp,
        )
    )

    def header(stamp=None):
        nanoseconds = int((clock.ros if stamp is None else stamp) * 1e9)
        sec, nanosec = divmod(nanoseconds, 1_000_000_000)
        return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))

    values = {
        "online": "true",
        "motion_enabled": "true",
        "operation_busy": "false",
        "activated": "true",
        "activation_state": "3",
        "gripper_fault": "0",
        "controller_fault": "0",
    }

    def status(stamp=None):
        return SimpleNamespace(
            header=header(stamp),
            status=[
                SimpleNamespace(
                    hardware_id=node.joint_name,
                    level=0,
                    values=[
                        SimpleNamespace(key=k, value=v)
                        for k, v in values.items()
                    ],
                )
            ],
        )

    def vr(trigger=0.5, stamp=None, valid=1):
        return SimpleNamespace(
            header=header(stamp),
            right_controller=SimpleNamespace(status=valid, trigger=trigger),
            left_controller=SimpleNamespace(status=1, trigger=0.0),
        )

    def joints(opening=0.0425, stamp=None):
        return module.JointState(
            header=header(stamp),
            name=[node.joint_name],
            position=[opening],
        )

    def refresh(trigger=0.5, opening=0.0425):
        node._vr_callback(vr(trigger))
        node._joint_callback(joints(opening))
        node._status_callback(status())

    def advance(seconds=0.1):
        clock.ros += seconds
        clock.local += seconds

    return SimpleNamespace(
        node=node,
        module=module,
        clock=clock,
        header=header,
        values=values,
        status=status,
        vr=vr,
        joints=joints,
        refresh=refresh,
        advance=advance,
    )


def test_default_wiring_and_standalone_contract(source):
    node = source.node
    assert [args[1] for args, _ in node.subscriptions] == [
        "/pico_bridge/vr_state",
        "/robotiq/joint_states",
        "/robotiq/status",
    ]
    assert (
        node._parameters["joint_command_topic"] == "/robotiq_teleop/candidate"
    )
    node._publish()
    heartbeat = node.match_state_publisher.messages[-1]
    assert not heartbeat.ready and not heartbeat.matched
    assert heartbeat.reason
    assert node.joint_command_publisher.messages == []


@pytest.mark.parametrize("trigger", [0.0, 0.5, 1.0])
def test_trigger_mapping_and_named_candidate_without_activation(
    source, trigger
):
    source.refresh(trigger=trigger)
    source.node._publish()
    candidate = source.node.joint_command_publisher.messages[-1]
    assert candidate.name == ["robotiq_gripper"]
    assert candidate.position == pytest.approx([(1 - trigger) * 0.085])
    assert candidate.velocity == candidate.effort == []
    match = source.node.match_state_publisher.messages[-1]
    assert match.ready
    assert match.candidate_topic == "/robotiq_teleop/candidate"
    assert match.header.stamp == candidate.header.stamp


def test_selected_left_controller(source):
    source.node.source_type = "left"
    source.refresh()
    source.node._publish()
    assert source.node.joint_command_publisher.messages[-1].position == [0.085]


def test_stable_match_dwell_and_transient_mismatch_reset(source):
    source.refresh()
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].matched
    for _ in range(4):
        source.advance()
        source.refresh()
        source.node._publish()
    assert source.node.match_state_publisher.messages[-1].matched
    source.advance()
    source.node._vr_callback(source.vr(1.0))
    source.advance(0.01)
    source.refresh()
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].matched
    assert source.node.match_state_publisher.messages[-1].ready


@pytest.mark.parametrize(
    "field",
    [
        "online",
        "motion_enabled",
        "operation_busy",
        "activated",
        "activation_state",
        "gripper_fault",
        "controller_fault",
    ],
)
def test_driver_unready_suppresses_candidates_and_resets_dwell(source, field):
    source.refresh()
    source.node._publish()
    source.advance()
    source.values[field] = "bad"
    source.refresh()
    source.node._publish()
    state = source.node.match_state_publisher.messages[-1]
    assert not state.ready and not state.matched
    assert field in state.reason
    assert len(source.node.joint_command_publisher.messages) == 1
    assert source.node._match_since is None


@pytest.mark.parametrize("stream", ["joints", "status"])
@pytest.mark.parametrize("stamp", [0.0, 9.0, 10.2, 10.0])
def test_bad_or_replayed_stamp_cannot_renew_readiness(source, stream, stamp):
    source.refresh()
    source.node._publish()
    source.advance(0.01)
    callback_name = "joint" if stream == "joints" else stream
    callback = getattr(source.node, f"_{callback_name}_callback")
    callback(getattr(source, stream)(stamp=stamp))
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready
    assert source.node._received[stream] is None


@pytest.mark.parametrize("stamp", [0.0, 9.0, 10.0, 140.0])
def test_vr_freshness_uses_receipt_time_not_header_stamp(source, stamp):
    source.refresh()
    source.node._publish()
    for _ in range(4):
        source.advance()
        # Repeat an arbitrary device stamp: frame ordering belongs to Bridge,
        # not to this consumer. Driver messages retain their ROS timestamps.
        source.node._vr_callback(source.vr(stamp=stamp))
        source.node._joint_callback(source.joints())
        source.node._status_callback(source.status())
        source.node._publish()
        assert source.node.match_state_publisher.messages[-1].ready
    assert source.node.match_state_publisher.messages[-1].matched


def test_vr_loss_blocks_candidates_with_live_driver_and_resets_dwell(source):
    source.node.match_dwell_s = 0.1
    source.refresh()
    source.node._publish()
    source.advance(0.15)
    source.refresh()
    source.node._publish()
    assert source.node.match_state_publisher.messages[-1].matched
    for _ in range(2):
        source.advance(0.2)
        source.node._joint_callback(source.joints())
        source.node._status_callback(source.status())
    source.node._publish()
    state = source.node.match_state_publisher.messages[-1]
    assert not state.ready and not state.matched
    assert "vr input" in state.reason
    assert len(source.node.joint_command_publisher.messages) == 2
    source.node._vr_callback(source.vr(stamp=140.0))
    source.node._publish()
    state = source.node.match_state_publisher.messages[-1]
    assert state.ready and not state.matched


def test_monotonic_loss_and_source_age_detect_stale_input(source):
    source.refresh()
    source.node._publish()
    source.clock.local += 1.0
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready
    source.advance()
    source.refresh()
    source.node._publish()
    assert source.node.match_state_publisher.messages[-1].ready
    assert not source.node.match_state_publisher.messages[-1].matched
    source.clock.ros += 1.0
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready


def test_recovery_before_timer_still_resets_match_dwell(source):
    source.node.match_dwell_s = 0.1
    source.refresh()
    source.node._publish()
    source.advance(0.15)
    source.refresh()
    source.node._publish()
    assert source.node.match_state_publisher.messages[-1].matched
    source.advance(0.4)
    source.refresh()
    source.node._publish()
    assert source.node.match_state_publisher.messages[-1].ready
    assert not source.node.match_state_publisher.messages[-1].matched


@pytest.mark.parametrize("trigger", [float("nan"), float("inf"), -0.1, 1.1])
def test_invalid_trigger(source, trigger):
    source.refresh(trigger=trigger)
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready


@pytest.mark.parametrize("opening", [float("nan"), -0.1, 0.09])
def test_invalid_opening(source, opening):
    source.refresh(opening=opening)
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready


def test_invalid_controller_and_named_feedback(source):
    source.refresh()
    source.advance()
    source.node._vr_callback(source.vr(valid=0))
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready
    source.advance()
    source.refresh()
    source.advance()
    message = source.joints()
    message.name = ["wrong_joint"]
    source.node._joint_callback(message)
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready


def test_diagnostic_identity_and_missing_fields(source):
    source.refresh()
    source.advance()
    message = source.status()
    message.status[0].hardware_id = "other"
    source.node._status_callback(message)
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready
    source.advance()
    del source.values["online"]
    source.refresh()
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready


@pytest.mark.parametrize("invalid", ["duplicate", "length", "missing"])
def test_feedback_layout_validation(source, invalid):
    source.refresh()
    source.advance()
    message = source.joints()
    if invalid == "duplicate":
        message.name *= 2
        message.position *= 2
    elif invalid == "length":
        message.position = []
    else:
        message.name = ["other_joint"]
    source.node._joint_callback(message)
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready


@pytest.mark.parametrize("invalid", ["duplicate", "level"])
def test_diagnostic_layout_and_level(source, invalid):
    source.refresh()
    source.advance()
    message = source.status()
    if invalid == "duplicate":
        message.status *= 2
    else:
        message.status[0].level = 2
    source.node._status_callback(message)
    source.node._publish()
    assert not source.node.match_state_publisher.messages[-1].ready


def test_candidate_and_heartbeat_stamps_increase_on_same_tick(source):
    source.refresh()
    for _ in range(3):
        source.node._publish()

    def stamp_ns(message):
        return (
            message.header.stamp.sec * 1000000000
            + message.header.stamp.nanosec
        )

    for publisher in (
        source.node.joint_command_publisher,
        source.node.match_state_publisher,
    ):
        stamps = [stamp_ns(message) for message in publisher.messages]
        assert all(
            0 < earlier < later
            for earlier, later in zip(stamps, stamps[1:], strict=False)
        )


@pytest.mark.parametrize(
    "parameter,value",
    [
        ("source_type", "both"),
        ("publish_hz", 0.0),
        ("stale_timeout_s", -1.0),
        ("match_dwell_s", float("nan")),
        ("match_tolerance_m", -1.0),
        ("max_clock_skew_s", -1.0),
        ("joint_name", ""),
    ],
)
def test_invalid_parameters(source, parameter, value):
    with pytest.raises(ValueError, match=parameter):
        source.module.RobotiqPicoVRTeleOpNode(
            parameter_overrides=[
                ros_stubs._Parameter(parameter, value),
            ]
        )
