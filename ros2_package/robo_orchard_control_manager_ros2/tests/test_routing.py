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

import pytest
import yaml
from rclpy.node import Node

from robo_orchard_control_manager_ros2.node import (
    EVENTS_TOPIC,
    STATUS_TOPIC,
    ControlManagerNode,
    ControlState,
)
from robo_orchard_teleop_msg_ros2.msg import TakeOverEvent


def _channel(name):
    return {
        "name": name,
        "kind": "joint_command",
        "msg_type": "sensor_msgs/msg/JointState",
        "autonomous_topic": f"/{name}/algo_cmd",
        "override_topic": f"/{name}/override_cmd",
        "output_topic": f"/robot/{name}/joint_cmd",
    }


def _make_node(tmp_path, replay_time_s=2.0):
    config_path = tmp_path / "control_manager.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "channels": [
                    _channel("left_arm"),
                    _channel("right_arm"),
                ],
                "replay_time_s": replay_time_s,
                "status_publish_rate_hz": 2.0,
            }
        ),
        encoding="utf-8",
    )
    Node.config_file = str(config_path)
    return ControlManagerNode()


def _message(node, data):
    message = node.message_types["left_arm"]()
    message.data = data
    return message


def _emit(node, topic, message):
    node.subscriptions[topic].callback(message)


def _outputs(node):
    return {
        name: node.publishers[f"/robot/{name}/joint_cmd"]
        for name in ("left_arm", "right_arm")
    }


@pytest.mark.parametrize(
    ("state", "autonomous_forwarded", "override_forwarded"),
    [
        (ControlState.AUTO, True, False),
        (ControlState.TAKEOVER, False, True),
        (ControlState.STOP, False, False),
        (ControlState.RESETTING, False, False),
    ],
)
def test_routing_matrix(
    tmp_path, state, autonomous_forwarded, override_forwarded
):
    node = _make_node(tmp_path)
    node._change_state(state)
    outputs = _outputs(node)
    autonomous = _message(node, "autonomous")
    override = _message(node, "override")

    _emit(node, "/left_arm/algo_cmd", autonomous)
    _emit(node, "/left_arm/override_cmd", override)

    published = outputs["left_arm"].published
    assert (autonomous in published) is autonomous_forwarded
    assert (override in published) is override_forwarded
    assert outputs["right_arm"].published == []
    assert node.state == state


def test_startup_and_heartbeat_publish_authoritative_status(tmp_path):
    node = _make_node(tmp_path)
    status_publisher = node.publishers[STATUS_TOPIC]

    assert node.state == ControlState.STOP
    assert [message.data for message in status_publisher.published] == ["stop"]
    assert node._status_timer.period == 0.5

    node.clock.nanoseconds = 1_000_000_000
    node._change_state(ControlState.AUTO)
    node.clock.nanoseconds = 2_000_000_000
    node._status_timer.callback()

    assert [message.data for message in status_publisher.published] == [
        "stop",
        "auto",
        "auto",
    ]
    assert status_publisher.published[-1].header.stamp == 2_000_000_000


def test_takeover_replays_one_common_time_command_per_channel(tmp_path):
    node = _make_node(tmp_path, replay_time_s=2.0)
    node._change_state(ControlState.AUTO)
    outputs = _outputs(node)
    early = {}

    for timestamp_s, suffix in ((1, "early"), (2, "late")):
        node.clock.nanoseconds = timestamp_s * 1_000_000_000
        for name in ("left_arm", "right_arm"):
            message = _message(node, f"{name}-{suffix}")
            _emit(node, f"/{name}/algo_cmd", message)
            if suffix == "early":
                early[name] = message

    for publisher in outputs.values():
        publisher.published.clear()

    node.clock.nanoseconds = 3_500_000_000
    replayed = node._change_state(ControlState.TAKEOVER)

    assert replayed is True
    assert outputs["left_arm"].published == [early["left_arm"]]
    assert outputs["right_arm"].published == [early["right_arm"]]
    event_types = [
        event.event_type for event in node.publishers[EVENTS_TOPIC].published
    ]
    assert event_types[-2:] == [
        TakeOverEvent.REPLAY_COMMAND_SENT,
        TakeOverEvent.TAKEOVER_TRIGGERED,
    ]


def test_replay_is_all_or_none_when_one_channel_has_no_old_command(tmp_path):
    node = _make_node(tmp_path, replay_time_s=2.0)
    node._change_state(ControlState.AUTO)
    outputs = _outputs(node)

    node.clock.nanoseconds = 1_000_000_000
    _emit(node, "/left_arm/algo_cmd", _message(node, "left-old"))
    node.clock.nanoseconds = 3_000_000_000
    _emit(node, "/right_arm/algo_cmd", _message(node, "right-new"))
    for publisher in outputs.values():
        publisher.published.clear()

    node.clock.nanoseconds = 3_500_000_000
    replayed = node._change_state(ControlState.TAKEOVER)

    assert replayed is False
    assert all(not publisher.published for publisher in outputs.values())
    event_types = [
        event.event_type for event in node.publishers[EVENTS_TOPIC].published
    ]
    assert TakeOverEvent.REPLAY_COMMAND_SENT not in event_types


def test_zero_replay_time_disables_history_and_replay(tmp_path):
    node = _make_node(tmp_path, replay_time_s=0.0)
    node._change_state(ControlState.AUTO)

    for name in ("left_arm", "right_arm"):
        _emit(node, f"/{name}/algo_cmd", _message(node, name))

    assert all(not channel.history for channel in node._channels.values())
    assert node._change_state(ControlState.TAKEOVER) is False
    event_types = [
        event.event_type for event in node.publishers[EVENTS_TOPIC].published
    ]
    assert TakeOverEvent.REPLAY_COMMAND_SENT not in event_types


def test_autonomous_commands_are_cached_only_in_auto(tmp_path):
    node = _make_node(tmp_path)
    left = node._channels["left_arm"]

    _emit(node, "/left_arm/algo_cmd", _message(node, "stop"))
    node._change_state(ControlState.TAKEOVER)
    _emit(node, "/left_arm/algo_cmd", _message(node, "takeover"))
    assert not left.history

    node._change_state(ControlState.AUTO)
    _emit(node, "/left_arm/algo_cmd", _message(node, "auto"))

    assert [message.data for _, message in left.history] == ["auto"]


def test_events_remain_simple_dagger_markers(tmp_path):
    node = _make_node(tmp_path, replay_time_s=0.0)
    events = node.publishers[EVENTS_TOPIC].published

    node._change_state(ControlState.AUTO)
    node._change_state(ControlState.TAKEOVER)
    node._change_state(ControlState.STOP)
    node._change_state(ControlState.RESETTING)

    assert [event.event_type for event in events] == [
        TakeOverEvent.RELEASE_TRIGGERED,
        TakeOverEvent.TAKEOVER_TRIGGERED,
        TakeOverEvent.STOP_TRIGGERED,
    ]


def test_reset_completion_does_not_emit_stop_event(tmp_path):
    node = _make_node(tmp_path, replay_time_s=0.0)
    events = node.publishers[EVENTS_TOPIC].published
    statuses = node.publishers[STATUS_TOPIC].published

    node._change_state(ControlState.RESETTING)
    node._change_state(ControlState.STOP)

    assert events == []
    assert [status.data for status in statuses[-2:]] == ["resetting", "stop"]


@pytest.mark.parametrize(
    "previous", [ControlState.AUTO, ControlState.TAKEOVER]
)
def test_operator_stop_transitions_still_emit_stop_event(tmp_path, previous):
    node = _make_node(tmp_path, replay_time_s=0.0)
    events = node.publishers[EVENTS_TOPIC].published
    node._change_state(previous)
    events.clear()

    node._change_state(ControlState.STOP)

    assert [event.event_type for event in events] == [
        TakeOverEvent.STOP_TRIGGERED
    ]
