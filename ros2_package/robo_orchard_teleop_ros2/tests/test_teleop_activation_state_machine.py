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

from robo_orchard_teleop_ros2.activation.state_machine import (
    ActivationState,
    HoldActionStateMachine,
    HoldActivationStateMachine,
)


def test_activation_requires_hold_and_releases_immediately():
    machine = HoldActivationStateMachine(debounce_s=1.0)
    machine.connect(initially_pressed=False)

    machine.key_event(value=1, now=2.0)
    assert not machine.advance(now=2.99)
    assert machine.state == ActivationState.INACTIVE
    assert machine.advance(now=3.0)
    assert machine.state == ActivationState.ACTIVE

    assert machine.key_event(value=0, now=3.1)
    assert machine.state == ActivationState.INACTIVE


def test_activation_connect_while_held_requires_release():
    machine = HoldActivationStateMachine(debounce_s=1.0)
    machine.connect(initially_pressed=True)

    assert not machine.advance(now=10.0)
    machine.key_event(value=0, now=10.1)
    machine.key_event(value=1, now=10.2)
    assert machine.advance(now=11.2)


def test_reset_action_fires_once_until_release():
    machine = HoldActionStateMachine(hold_s=1.0)
    machine.connect(initially_pressed=False)

    machine.key_event(value=1, now=1.0)
    assert not machine.advance(now=1.99)
    assert machine.advance(now=2.0)
    assert not machine.advance(now=3.0)

    machine.key_event(value=0, now=3.1)
    machine.key_event(value=1, now=3.2)
    assert machine.advance(now=4.2)


def test_disconnect_fails_activation_closed_and_cancels_reset():
    activation = HoldActivationStateMachine(debounce_s=0.0)
    reset = HoldActionStateMachine(hold_s=0.0)
    activation.connect(initially_pressed=False)
    reset.connect(initially_pressed=False)
    activation.key_event(value=1, now=1.0)
    reset.key_event(value=1, now=1.0)
    assert activation.advance(now=1.0)

    activation.disconnect()
    reset.disconnect()

    assert activation.state == ActivationState.UNAVAILABLE
    assert not reset.advance(now=2.0)
