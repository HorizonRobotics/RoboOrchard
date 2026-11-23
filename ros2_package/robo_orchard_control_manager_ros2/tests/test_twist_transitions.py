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

"""Neutral velocity ordering and joint-only replay under mixed routing."""

import threading

import pytest
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger

from robo_orchard_control_manager_ros2.node import (
    AUTO_SERVICE,
    EVENTS_TOPIC,
    RESET_SERVICE,
    STATUS_TOPIC,
    STOP_SERVICE,
    TAKEOVER_SERVICE,
    ControlManagerNode,
    ControlState,
)
from robo_orchard_teleop_msg_ros2.msg import TakeOverEvent


def _make_node(
    tmp_path,
    *,
    joints=2,
    velocities=1,
    twist_type="geometry_msgs/msg/Twist",
    **options,
):
    Node.reset_service_behaviors()
    channels = [
        {"name": f"joint{i}", "msg_type": "sensor_msgs/msg/JointState"}
        for i in range(joints)
    ] + [
        {"name": f"motion{i}", "msg_type": twist_type}
        for i in range(velocities)
    ]
    for channel in channels:
        name = channel["name"]
        channel.update(
            autonomous_topic=f"/{name}/auto",
            override_topic=f"/{name}/override",
            output_topic=f"/{name}/output",
        )
    config = {
        "channels": channels,
        "replay_time_s": 2.0,
        "service_wait_timeout_s": 0.01,
        "service_response_timeout_s": 0.1,
        **options,
    }
    path = tmp_path / "manager.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    Node.config_file = str(path)
    return ControlManagerNode()


def _call(node, service):
    return node.services[service].callback(
        Trigger.Request(), Trigger.Response()
    )


def _emit(node, name, source, value):
    message = node.message_types[name]()
    if isinstance(message, Twist):
        message.linear.x = value
        message.angular.z = -value
    else:
        message.data = value
    node.subscriptions[f"/{name}/{source}"].callback(message)
    return message


def _motion(node, name="motion0"):
    return node.publishers[f"/{name}/output"].published


def _components(message):
    return [
        getattr(getattr(message, vector), axis)
        for vector in ("linear", "angular")
        for axis in "xyz"
    ]


def test_startup_stop_neutralizes_only_declared_twist_channels(tmp_path):
    node = _make_node(tmp_path, velocities=2)
    assert node.state == ControlState.STOP
    for index in range(2):
        assert _components(_motion(node, f"motion{index}")[0]) == [0] * 6
        assert node.publishers[f"/joint{index}/output"].published == []
    assert all(
        channel.config.kind == "joint_command"
        for channel in node._channels.values()
    )


@pytest.mark.parametrize("source", [ControlState.AUTO, ControlState.TAKEOVER])
def test_stop_neutralizes_and_rejects_both_sources_without_joint_command(
    tmp_path, source
):
    node = _make_node(tmp_path)
    node._change_state(source)
    _emit(
        node,
        "motion0",
        "auto" if source == ControlState.AUTO else "override",
        0.3,
    )
    _emit(
        node,
        "joint0",
        "auto" if source == ControlState.AUTO else "override",
        "hold",
    )
    joint_messages = node.publishers["/joint0/output"].published.copy()
    assert _call(node, STOP_SERVICE).success
    assert _components(_motion(node)[-1]) == [0] * 6
    count = len(_motion(node))
    _emit(node, "motion0", "auto", 0.8)
    _emit(node, "motion0", "override", 0.9)
    assert len(_motion(node)) == count
    assert node.publishers["/joint0/output"].published == joint_messages
    assert all(not channel.history for channel in node._channels.values())


def test_same_stop_reasserts_zero_but_auto_and_takeover_do_not_interrupt(
    tmp_path,
):
    node = _make_node(tmp_path)
    statuses = node.publishers[STATUS_TOPIC].published
    events = node.publishers[EVENTS_TOPIC].published
    assert _call(node, STOP_SERVICE).success
    assert len(_motion(node)) == 2
    assert len(statuses) == 1
    assert events == []
    for service, source in (
        (AUTO_SERVICE, "auto"),
        (TAKEOVER_SERVICE, "override"),
    ):
        assert _call(node, service).success
        _emit(node, "motion0", source, 0.3)
        count = len(_motion(node))
        event_count = len(events)
        assert _call(node, service).success
        assert len(_motion(node)) == count
        assert len(events) == event_count


@pytest.mark.parametrize("missing_joint", [False, True])
def test_takeover_zeroes_twist_then_replays_all_or_none_of_joint_channels(
    tmp_path, missing_joint
):
    node = _make_node(tmp_path)
    node._change_state(ControlState.AUTO)
    for seconds in (1, 2):
        node.clock.nanoseconds = seconds * 1_000_000_000
        _emit(node, "motion0", "auto", seconds)
        _emit(node, "joint0", "auto", seconds)
        if not missing_joint or seconds == 2:
            _emit(node, "joint1", "auto", seconds)
    assert not node._channels["motion0"].history
    order = []
    for channel in node._channels.values():
        publisher = channel.publisher
        publisher.published.clear()
        original_publish = publisher.publish

        def publish(
            message, name=channel.config.name, original=original_publish
        ):
            order.append(name)
            original(message)

        publisher.publish = publish
    node.clock.nanoseconds = 3_500_000_000
    assert node._change_state(ControlState.TAKEOVER) is not missing_joint
    assert _components(_motion(node)[0]) == [0] * 6
    assert len(_motion(node)) == 1
    assert order == (
        ["motion0"] if missing_joint else ["motion0", "joint0", "joint1"]
    )
    for name in ("joint0", "joint1"):
        messages = node.publishers[f"/{name}/output"].published
        assert [msg.data for msg in messages] == ([] if missing_joint else [1])
    _emit(node, "motion0", "auto", 8)
    assert len(_motion(node)) == 1
    _emit(node, "motion0", "override", 0.4)
    assert _motion(node)[-1].linear.x == 0.4


@pytest.mark.parametrize(
    "twist_type", ["geometry_msgs/msg/Twist", "geometry_msgs/Twist"]
)
def test_twist_only_takeover_does_not_report_replay_or_reuse_velocity(
    tmp_path,
    twist_type,
):
    node = _make_node(tmp_path, joints=0, twist_type=twist_type)
    node._change_state(ControlState.AUTO)
    _emit(node, "motion0", "auto", 0.3)
    assert not node._channels["motion0"].history
    node.clock.nanoseconds += 3_000_000_000
    assert _call(node, TAKEOVER_SERVICE).success
    assert _components(_motion(node)[-1]) == [0] * 6
    assert TakeOverEvent.REPLAY_COMMAND_SENT not in [
        event.event_type for event in node.publishers[EVENTS_TOPIC].published
    ]
    _emit(node, "motion0", "override", 0.5)
    assert _call(node, STOP_SERVICE).success
    assert _components(_motion(node)[-1]) == [0] * 6
    assert _call(node, AUTO_SERVICE).success
    assert _components(_motion(node)[-1]) == [0] * 6
    assert not node._channels["motion0"].history


@pytest.mark.parametrize("failure", ["disable", "reset", None])
def test_reset_is_neutral_before_service_calls_and_stays_stopped_on_failure(
    tmp_path, failure
):
    node = _make_node(
        tmp_path,
        inference_disable_services=["/disable"],
        reset_services=["/reset"],
    )
    node._change_state(ControlState.AUTO)
    _emit(node, "motion0", "auto", 0.3)
    Node.configure_service("/disable", success=failure != "disable")
    Node.configure_service("/reset", success=failure != "reset")
    for client in node.clients.values():
        original = client.call_async

        def call(request, original=original):
            assert node.state == ControlState.RESETTING
            assert not node._state_lock.locked()
            assert _components(_motion(node)[-1]) == [0] * 6
            _emit(node, "motion0", "auto", 0.8)
            _emit(node, "motion0", "override", 0.9)
            return original(request)

        client.call_async = call
    response = _call(node, RESET_SERVICE)
    assert response.success is (failure is None)
    assert node.state == ControlState.STOP
    assert _components(_motion(node)[-1]) == [0] * 6
    assert node.service_call_order == (
        ["/disable"] if failure == "disable" else ["/disable", "/reset"]
    )


def test_failed_auto_preparation_neutralizes_takeover_before_enable(tmp_path):
    node = _make_node(tmp_path, enable_services=["/enable"])
    node._change_state(ControlState.TAKEOVER)
    _emit(node, "motion0", "override", 0.3)
    Node.configure_service("/enable", success=False)
    original = node.clients["/enable"].call_async

    def enable(request):
        assert node.state == ControlState.STOP
        assert not node._state_lock.locked()
        assert _components(_motion(node)[-1]) == [0] * 6
        return original(request)

    node.clients["/enable"].call_async = enable
    assert not _call(node, AUTO_SERVICE).success
    assert node.state == ControlState.STOP
    _emit(node, "motion0", "override", 0.9)
    assert _components(_motion(node)[-1]) == [0] * 6


def test_old_source_cannot_publish_after_blocked_neutral_handover(tmp_path):
    node = _make_node(tmp_path)
    node._change_state(ControlState.AUTO)
    entered = threading.Event()
    release = threading.Event()
    old_started = threading.Event()
    old_done = threading.Event()
    publisher = node.publishers["/motion0/output"]
    original = publisher.publish

    def neutral(message):
        assert node._state_lock.locked()
        entered.set()
        assert release.wait(timeout=2)
        original(message)

    publisher.publish = neutral
    transition = threading.Thread(target=_call, args=(node, TAKEOVER_SERVICE))
    transition.start()
    assert entered.wait(timeout=2)

    def old_command():
        old_started.set()
        _emit(node, "motion0", "auto", 0.9)
        old_done.set()

    old = threading.Thread(target=old_command)
    old.start()
    assert old_started.wait(timeout=2)
    assert not old_done.wait(timeout=0.01)
    release.set()
    transition.join(timeout=2)
    old.join(timeout=2)
    assert not transition.is_alive() and not old.is_alive()
    assert node.state == ControlState.TAKEOVER
    assert _components(_motion(node)[-1]) == [0] * 6
    assert all(message.linear.x != 0.9 for message in _motion(node))


def test_stop_zero_follows_inflight_source_publication(tmp_path):
    node = _make_node(tmp_path)
    node._change_state(ControlState.AUTO)
    publisher = node.publishers["/motion0/output"]
    publisher.published.clear()
    entered = threading.Event()
    release = threading.Event()
    original = publisher.publish

    def publish(message):
        if message.linear.x:
            entered.set()
            assert release.wait(timeout=2)
        original(message)

    publisher.publish = publish
    source = threading.Thread(
        target=_emit, args=(node, "motion0", "auto", 0.3)
    )
    source.start()
    assert entered.wait(timeout=2)
    transition = threading.Thread(target=_call, args=(node, STOP_SERVICE))
    transition.start()
    release.set()
    source.join(timeout=2)
    transition.join(timeout=2)
    assert not source.is_alive() and not transition.is_alive()
    assert [message.linear.x for message in _motion(node)] == [0.3, 0]


def test_neutral_publication_failure_does_not_skip_other_velocity_channels(
    tmp_path,
):
    node = _make_node(tmp_path, velocities=2)
    node._change_state(ControlState.AUTO)
    _emit(node, "motion1", "auto", 0.3)

    def unavailable(message):
        raise RuntimeError("publisher unavailable")

    node.publishers["/motion0/output"].publish = unavailable
    assert _call(node, STOP_SERVICE).success
    assert node.state == ControlState.STOP
    assert _components(_motion(node, "motion1")[-1]) == [0] * 6
    assert any("motion0" in error for error in node.logger.error_messages)
