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

"""Checks on the per-tick command limiter."""

from __future__ import annotations
import math

import numpy as np
import pytest
from pydantic import ValidationError

from robo_orchard_deploy_ros2.command_limiter import CommandLimiter
from robo_orchard_deploy_ros2.config import (
    ControlConfig,
    DeployConfig,
    ImageChannel,
    JointCommandChannel,
    ObservationConfig,
)

CONTROL_HZ = 200.0
N_JOINTS = 7
MAX_VELOCITY = 3.0
ALLOWED = MAX_VELOCITY / CONTROL_HZ


class _Logger:
    def __init__(self):
        self.errors = []

    def error(self, message, **kwargs):
        self.errors.append(message)


def _limiter(max_velocity=MAX_VELOCITY):
    log = _Logger()
    return CommandLimiter(max_velocity, CONTROL_HZ, log), log


def test_motion_under_the_limit_is_untouched():
    lim, log = _limiter()
    step = ALLOWED / 2.0
    position = np.zeros(N_JOINTS)
    for _ in range(50):
        position = position + step
        np.testing.assert_allclose(
            np.asarray(lim.apply(position.tolist(), "arm")), position
        )
    assert lim.n_clamped == 0
    assert log.errors == []


@pytest.mark.parametrize("jump_deg", [5.0, 17.0, 25.0])
def test_a_step_over_the_limit_is_clamped_and_reported(jump_deg):
    lim, log = _limiter()
    lim.apply(np.zeros(N_JOINTS).tolist(), "arm")
    target = np.zeros(N_JOINTS)
    target[3] = math.radians(jump_deg)

    out = np.asarray(lim.apply(target.tolist(), "arm"))

    assert np.abs(out).max() == pytest.approx(ALLOWED)
    assert lim.n_clamped == 1
    assert lim.worst_step == pytest.approx(math.radians(jump_deg))
    assert any("over the" in m for m in log.errors)


def test_clamping_still_converges_on_the_target():
    """Clamped, not dropped: the robot keeps going where the plan wants."""
    lim, _ = _limiter()
    lim.apply(np.zeros(N_JOINTS).tolist(), "arm")
    target = np.zeros(N_JOINTS)
    target[0] = math.radians(17.0)

    out = np.zeros(N_JOINTS)
    for _ in range(int(math.radians(17.0) / ALLOWED) + 2):
        out = np.asarray(lim.apply(target.tolist(), "arm"))

    np.testing.assert_allclose(out, target, atol=1e-9)


def test_channels_are_limited_independently():
    lim, _ = _limiter()
    lim.apply(np.zeros(N_JOINTS).tolist(), "a")
    lim.apply(np.zeros(N_JOINTS).tolist(), "b")

    lim.apply((np.ones(N_JOINTS) * math.radians(20.0)).tolist(), "a")
    step = ALLOWED / 2.0
    moved = (np.ones(N_JOINTS) * step).tolist()
    np.testing.assert_allclose(
        np.asarray(lim.apply(moved, "b")), np.full(N_JOINTS, step)
    )
    assert lim.n_clamped == 1


def test_the_first_command_after_reset_is_not_a_step():
    """Documented gap: with no predecessor there is nothing to bound."""
    lim, _ = _limiter()
    lim.apply(np.zeros(N_JOINTS).tolist(), "arm")
    lim.reset()
    far = (np.ones(N_JOINTS) * math.radians(30.0)).tolist()

    np.testing.assert_allclose(np.asarray(lim.apply(far, "arm")), far)
    assert lim.n_clamped == 0


def test_a_channel_that_changes_width_restarts_the_comparison():
    lim, _ = _limiter()
    lim.apply(np.zeros(N_JOINTS).tolist(), "arm")
    wider = (np.ones(N_JOINTS + 1) * math.radians(30.0)).tolist()

    np.testing.assert_allclose(np.asarray(lim.apply(wider, "arm")), wider)
    assert lim.n_clamped == 0


def test_limiter_can_be_switched_off():
    lim, _ = _limiter(max_velocity=None)
    assert not lim.enabled
    lim.apply(np.zeros(N_JOINTS).tolist(), "arm")
    far = (np.ones(N_JOINTS) * math.radians(30.0)).tolist()
    assert lim.apply(far, "arm") is far


def test_a_none_command_passes_straight_through():
    lim, _ = _limiter()
    assert lim.apply(None, "arm") is None


def test_bounds_reject_a_velocity_that_would_disable_the_clamp():
    """A non-positive limit is a configuration error, not 'no limit'."""
    for bad in (0.0, -1.0):
        with pytest.raises(ValidationError):
            DeployConfig(
                observation_config=ObservationConfig(
                    channels=[
                        ImageChannel(server_input_key="color", topic="/color")
                    ]
                ),
                control_config=ControlConfig(
                    channels=[
                        JointCommandChannel(
                            server_output_key="a",
                            topic="/a",
                            joint_names=["j1"],
                        )
                    ]
                ),
                max_command_velocity=bad,
            )
