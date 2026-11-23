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
from hexfellow_stubs import load_node_module, vr_state


@pytest.fixture
def node():
    return load_node_module("chassis").HexfellowChassisPicoVRTeleOpNode()


def activate(node):
    node._on_vr_state(vr_state(axis_click=True))
    node.time_s += node.toggle_hold_time_s
    node._on_vr_state(vr_state(axis_click=True))
    assert node.control_active
    node._on_vr_state(vr_state(axis_click=False))


def assert_stopped(node):
    command = node.publishers[0].messages[-1]
    assert command.linear.x == command.linear.y == command.linear.z == 0.0
    assert command.angular.x == command.angular.y == command.angular.z == 0.0


def test_defaults_publish_only_override_candidates(node):
    assert node.teleop_frequency_hz == 30.0
    assert node.stale_timeout_s == 0.5
    assert node.vr_topic == "/pico_bridge/vr_state"
    assert node.publishers[0].topic == "/hexfellow_teleop/cmd_vel"
    assert node.timers[0].period == pytest.approx(1 / 30)
    node._on_vr_state(vr_state(axis_y=1.0))
    node._publish_cmd_vel()
    assert not node.control_active
    assert_stopped(node)


@pytest.mark.parametrize("frequency", [60.0, 90.0])
def test_configured_control_rate(frequency):
    node = load_node_module("chassis").HexfellowChassisPicoVRTeleOpNode(
        parameter_overrides=[
            SimpleNamespace(name="teleop_frequency_hz", value=frequency)
        ]
    )
    assert node.timers[0].period == pytest.approx(1 / frequency)


@pytest.mark.parametrize("frequency", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_control_rate(frequency):
    with pytest.raises(ValueError, match="teleop_frequency_hz"):
        load_node_module("chassis").HexfellowChassisPicoVRTeleOpNode(
            parameter_overrides=[
                SimpleNamespace(name="teleop_frequency_hz", value=frequency)
            ]
        )


def test_long_press_toggles_once_until_released(node):
    node._on_vr_state(vr_state(axis_click=True))
    node.time_s = 0.99
    node._on_vr_state(vr_state(axis_click=True))
    assert not node.control_active
    node.time_s = 1.0
    node._on_vr_state(vr_state(axis_click=True))
    assert node.control_active
    node.time_s = 2.0
    node._on_vr_state(vr_state(axis_click=True))
    assert node.control_active
    node._on_vr_state(vr_state(axis_click=False))
    node._on_vr_state(vr_state(axis_click=True))
    node.time_s = 3.0
    node._on_vr_state(vr_state(axis_click=True))
    assert not node.control_active
    assert_stopped(node)


def test_maps_left_stick_to_forward_speed_and_yaw(node):
    activate(node)
    node._on_vr_state(vr_state(axis_x=0.25, axis_y=0.5))
    node._publish_cmd_vel()
    command = node.publishers[0].messages[-1]
    assert command.linear.x == pytest.approx(0.1)
    assert command.angular.z == pytest.approx(-0.05)
    assert command.linear.y == 0.0
    node._on_vr_state(vr_state())
    node._publish_cmd_vel()
    assert_stopped(node)


@pytest.mark.parametrize("value", [0.05, float("nan"), float("inf")])
def test_deadzone_and_nonfinite_axes_stop(node, value):
    activate(node)
    node._on_vr_state(vr_state(axis_x=value, axis_y=value))
    node._publish_cmd_vel()
    assert_stopped(node)


def test_configured_topics_speed_limits_and_axis_directions():
    node = load_node_module("chassis").HexfellowChassisPicoVRTeleOpNode(
        parameter_overrides=[
            SimpleNamespace(name=name, value=value)
            for name, value in {
                "vr_topic": "/input/vr_state",
                "cmd_vel_topic": "/override/chassis",
                "max_linear_x": 0.4,
                "max_angular_z": 0.3,
                "axis_y_sign": -1.0,
                "axis_x_sign": 1.0,
            }.items()
        ]
    )
    activate(node)
    node._on_vr_state(vr_state(axis_x=2.0, axis_y=-2.0))
    node._publish_cmd_vel()
    assert node.subscriptions[0].topic == "/input/vr_state"
    assert node.publishers[0].topic == "/override/chassis"
    command = node.publishers[0].messages[-1]
    assert command.linear.x == pytest.approx(0.4)
    assert command.angular.z == pytest.approx(0.3)


@pytest.mark.parametrize("invalid_controller", [False, True])
def test_input_loss_stops_and_deactivates(node, invalid_controller):
    activate(node)
    if invalid_controller:
        node._on_vr_state(vr_state(status=0))
    else:
        node.time_s += 0.51
        node._publish_cmd_vel()
    assert not node.control_active
    assert_stopped(node)
    node._on_vr_state(vr_state(axis_y=1.0))
    node._publish_cmd_vel()
    assert_stopped(node)


def test_timeout_discards_partial_long_press(node):
    node._on_vr_state(vr_state(axis_click=True))
    node.time_s = 0.5
    node._publish_cmd_vel()
    node.time_s = 2.0
    node._on_vr_state(vr_state(axis_click=True))
    assert not node.control_active
    node.time_s = 3.0
    node._on_vr_state(vr_state(axis_click=True))
    assert node.control_active


@pytest.mark.parametrize("timeout", [None, 0.25, 1.0])
def test_stale_timeout_stops_at_default_or_configured_boundary(timeout):
    overrides = (
        []
        if timeout is None
        else [SimpleNamespace(name="stale_timeout_s", value=timeout)]
    )
    node = load_node_module("chassis").HexfellowChassisPicoVRTeleOpNode(
        parameter_overrides=overrides
    )
    timeout = 0.5 if timeout is None else timeout
    activate(node)
    node._on_vr_state(vr_state(axis_y=1.0))
    received = node.time_s
    node.time_s = received + timeout - 0.01
    node._publish_cmd_vel()
    assert node.control_active
    assert node.publishers[0].messages[-1].linear.x == pytest.approx(0.2)
    node.time_s = received + timeout
    node._publish_cmd_vel()
    assert not node.control_active
    assert_stopped(node)
