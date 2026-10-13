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

from types import SimpleNamespace

import pytest
from hexfellow_stubs import JointState, load_node_module, vr_state


@pytest.fixture
def node():
    return load_node_module("lift").HexfellowLiftPicoVRTeleOpNode()


def feedback(node, position=0.3):
    node._on_state(JointState(name=["joint1"], position=[position]))


def move(node, axis_y=1.0):
    feedback(node)
    node._on_vr_state(vr_state(axis_y=axis_y))
    node.time_s += 0.1
    node._publish_joint_cmd()


def test_defaults_and_wait_for_feedback(node):
    assert node.vr_topic == "/pico_bridge/vr_state"
    assert node.state_topic == "/lift/motor_states"
    assert node.publishers[0].topic == "/hexfellow_teleop/lift_joint_cmd"
    node._on_vr_state(vr_state(axis_y=1.0))
    node.time_s = 0.1
    node._publish_joint_cmd()
    assert node.publishers[0].messages == []


@pytest.mark.parametrize("axis_y", [-1.0, 1.0])
def test_integrates_right_stick_from_measured_position(node, axis_y):
    move(node, axis_y)
    command = node.publishers[0].messages[-1]
    assert command.name == ["joint1"]
    assert command.position == pytest.approx([0.3 - axis_y * 0.08 * 0.1])


def test_left_controller_does_not_affect_lift(node):
    feedback(node)
    message = vr_state(axis_y=0.0)
    message.left_controller.axis_y = 1.0
    node._on_vr_state(message)
    node.time_s = 0.1
    node._publish_joint_cmd()
    assert node.publishers[0].messages == []


def test_limits_target_lead_and_tracks_moving_feedback(node):
    for tick in range(1, 10):
        node.time_s = tick * 0.1
        feedback(node)
        node._on_vr_state(vr_state(axis_y=1.0))
        node._publish_joint_cmd()
    assert node.target_position == pytest.approx(0.285)
    node.time_s = 1.0
    feedback(node, 0.28)
    node._on_vr_state(vr_state(axis_y=1.0))
    node._publish_joint_cmd()
    assert node.target_position == pytest.approx(0.277)


@pytest.mark.parametrize("position,axis_y", [(0.0, 1.0), (0.6, -1.0)])
def test_stays_within_lift_travel(node, position, axis_y):
    feedback(node, position)
    node._on_vr_state(vr_state(axis_y=axis_y))
    node.time_s = 0.1
    node._publish_joint_cmd()
    assert node.target_position == position


@pytest.mark.parametrize("reason", ["deadzone", "invalid", "stale"])
def test_input_stop_holds_latest_feedback_once(node, reason):
    move(node)
    node.time_s = 0.5 if reason == "stale" else 0.2
    feedback(node, 0.295)
    if reason != "stale":
        node._on_vr_state(
            vr_state(
                axis_y=0.05 if reason == "deadzone" else 1.0,
                status=0 if reason == "invalid" else 1,
            )
        )
    node._publish_joint_cmd()
    assert node.publishers[0].messages[-1].position == [0.295]
    assert not node._motion_active
    count = len(node.publishers[0].messages)
    node._publish_joint_cmd()
    assert len(node.publishers[0].messages) == count


def test_feedback_timeout_suppresses_commands_and_reseeds_target(node):
    move(node)
    count = len(node.publishers[0].messages)
    node.time_s = 0.5
    node._on_vr_state(vr_state(axis_y=1.0))
    node._publish_joint_cmd()
    assert len(node.publishers[0].messages) == count
    assert node._motion_active
    feedback(node, 0.4)
    node.time_s = 0.6
    node._publish_joint_cmd()
    assert node.target_position == pytest.approx(0.392)


@pytest.mark.parametrize("reason", ["deadzone", "invalid", "stale"])
def test_feedback_recovery_holds_after_input_stop(node, reason):
    move(node)
    node.time_s = 0.5
    if reason != "stale":
        node._on_vr_state(
            vr_state(
                axis_y=0.0 if reason == "deadzone" else 1.0,
                status=0 if reason == "invalid" else 1,
            )
        )
    node._publish_joint_cmd()
    assert len(node.publishers[0].messages) == 1

    node.time_s = 0.6
    feedback(node, 0.296)
    node._publish_joint_cmd()
    assert [msg.position for msg in node.publishers[0].messages] == [
        pytest.approx([0.292]),
        pytest.approx([0.296]),
    ]
    assert not node._motion_active
    node.time_s = 0.7
    feedback(node, 0.297)
    node._publish_joint_cmd()
    assert len(node.publishers[0].messages) == 2


def test_feedback_recovery_without_prior_motion_does_not_publish(node):
    feedback(node)
    node.time_s = 0.5
    node._publish_joint_cmd()
    node.time_s = 0.6
    feedback(node, 0.296)
    node._on_vr_state(vr_state(axis_y=0.0))
    node._publish_joint_cmd()
    assert node.publishers[0].messages == []


@pytest.mark.parametrize(
    "names,positions",
    [
        (["joint1"], [float("nan")]),
        (["joint1"], [float("inf")]),
        (["other"], [0.3]),
        (["joint1"], []),
        ([], [0.2, 0.3]),
    ],
)
def test_invalid_feedback_does_not_initialize_or_refresh(
    node, names, positions
):
    message = JointState(name=names, position=positions)
    node._on_state(message)
    assert not node._has_fresh_state()
    feedback(node)
    node.time_s = 0.5
    node._on_state(message)
    assert not node._has_fresh_state()


@pytest.mark.parametrize(
    "names,positions", [(["joint0", "joint1"], [0.0, 0.3]), ([], [0.3])]
)
def test_selects_lift_joint_feedback(node, names, positions):
    node._on_state(JointState(name=names, position=positions))
    assert node.target_position == 0.3


@pytest.mark.parametrize("axis_y", [float("nan"), float("inf"), 0.01])
def test_invalid_or_small_axis_does_not_move(node, axis_y):
    move(node, axis_y)
    assert node.publishers[0].messages == []


def test_output_and_feedback_topics_are_configurable():
    node = load_node_module("lift").HexfellowLiftPicoVRTeleOpNode(
        parameter_overrides=[
            SimpleNamespace(name="joint_cmd_topic", value="/override/lift"),
            SimpleNamespace(name="state_topic", value="/measured/lift"),
        ]
    )
    assert node.publishers[0].topic == "/override/lift"
    assert node.subscriptions[1].topic == "/measured/lift"
