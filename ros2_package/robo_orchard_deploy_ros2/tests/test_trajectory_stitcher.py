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

"""Offline checks for the chunk stitcher.

Runs without ROS: the stitcher needs only numpy and osqp, so the handover
arithmetic can be exercised against synthetic chunks on a workstation.
"""

from __future__ import annotations

import numpy as np
import pytest

from robo_orchard_deploy_ros2.config import TrajectoryStitchConfig
from robo_orchard_deploy_ros2.trajectory_stitcher import (
    TrajectoryStitcher,
    _eval_cubic,
)

CONTROL_HZ = 200.0
N_JOINTS = 7
CHUNK_LEN = 400
KEYS = ["channel_a", "channel_b"]


def _config(**overrides):
    fields = {
        "max_velocity": 3.0,
        "max_acceleration": 10.0,
        "max_jerk": 500.0,
    }
    fields.update(overrides)
    return TrajectoryStitchConfig(**fields)


def _chunk(n_steps=CHUNK_LEN, seed=0):
    """A smooth two-channel chunk with the shape the model returns."""
    rng = np.random.default_rng(seed)
    t = np.arange(n_steps) / CONTROL_HZ
    amp = rng.uniform(0.2, 0.6, size=2 * N_JOINTS)
    freq = rng.uniform(0.3, 0.8, size=2 * N_JOINTS)
    phase = rng.uniform(0.0, 2 * np.pi, size=2 * N_JOINTS)
    full = amp * np.sin(2 * np.pi * freq * t[:, None] + phase)
    return {
        KEYS[0]: full[:, :N_JOINTS].tolist(),
        KEYS[1]: full[:, N_JOINTS:].tolist(),
    }


def _array(actions):
    return np.hstack([np.asarray(actions[key]) for key in KEYS])


def _stitcher(log=None, config=None):
    st = TrajectoryStitcher(config or _config(), CONTROL_HZ, KEYS, log)
    if not st.enabled:
        pytest.skip("osqp is not installed")
    return st


class _Log:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, message, **kwargs):
        self.errors.append(message)

    def warning(self, message, **kwargs):
        self.warnings.append(message)


def _handoff_error(prev_ran, install_idx=50):
    """How far the handoff state is from the step the robot is really on.

    The stitcher reads the state from the previous solve, ``prev_ran``
    steps in. Inside the solved window the two are the same trajectory, so
    the difference is the solver residual and nothing else.
    """
    st = _stitcher()
    installed = _array(st.stitch(_chunk(), install_idx, 0))
    st.commit()
    live, live_t0 = st._live
    dt_qp = st._config.solver_dt
    state = np.array(
        [
            _eval_cubic(*live[d], dt_qp, live_t0 + prev_ran / CONTROL_HZ)
            for d in range(2 * N_JOINTS)
        ]
    )
    return float(np.abs(state[:, 0] - installed[install_idx + prev_ran]).max())


def test_a_null_config_leaves_the_chunk_alone():
    st = TrajectoryStitcher(None, CONTROL_HZ, KEYS)
    assert not st.enabled
    chunk = _chunk()
    before = _array(chunk).copy()

    np.testing.assert_array_equal(_array(st.stitch(chunk, 50, 0)), before)


def test_the_tail_from_the_install_step_is_rewritten():
    log = _Log()
    st = _stitcher(log)
    chunk = _chunk()
    before = _array(chunk).copy()

    out = _array(st.stitch(chunk, 50, 0))

    np.testing.assert_array_equal(out[:50], before[:50])
    assert not np.allclose(out[50:], before[50:])
    assert log.warnings == []


def test_handoff_is_exact_for_any_reachable_prev_ran():
    for prev_ran in (10, 50, 73, 120, 205, 238, 300):
        assert _handoff_error(prev_ran) < 1e-3, prev_ran


def test_a_limit_below_the_requested_motion_falls_back():
    """A limit below the chunk's own motion falls back to the chunk.

    The limits are an envelope, not a shaping target: below the motion
    already requested the solver runs out of iterations.
    """
    log = _Log()
    st = _stitcher(log, config=_config(max_velocity=0.2))
    chunk = _chunk()
    before = _array(chunk).copy()

    out = _array(st.stitch(chunk, 0, 0))

    np.testing.assert_array_equal(out, before)
    assert st.n_failed == 1
    assert st.n_solved == 0
    assert any("unchanged" in m for m in log.warnings)


def test_an_envelope_above_the_requested_motion_solves():
    st = _stitcher(config=_config(max_velocity=6.0))
    chunk = _chunk()
    before = _array(chunk).copy()

    out = _array(st.stitch(chunk, 0, 0))

    assert st.n_solved == 1
    assert st.n_failed == 0
    assert not np.allclose(out, before)


def test_horizons_are_quantised_so_solvers_are_reused():
    """Building a solver costs far more than a solve."""
    st = _stitcher(config=_config(horizon_quantum=16))
    for install_idx in range(0, 64, 4):
        st.stitch(_chunk(), install_idx, 0)
        st.commit()

    assert st.n_solved == 16
    assert len(st._qp) <= 3


def test_a_handoff_past_the_live_window_is_counted_and_logged():
    log = _Log()
    st = _stitcher(log)
    short = _chunk(n_steps=64)
    st.stitch(short, 0, 0)
    st.commit()
    assert st.n_overrun == 0

    st.stitch(_chunk(n_steps=64, seed=1), 0, 400)

    assert st.n_overrun == 1
    assert any("extrapolated" in m for m in log.errors)


def test_solver_failure_installs_the_chunk_unchanged():
    """A failed solve degrades to the model's chunk, not a stopped robot."""
    st = _stitcher()
    st.stitch(_chunk(), 0, 0)
    st.commit()
    # An unreachable handoff state makes the QP infeasible.
    live, t0 = st._live
    st._live = ([(x, v * 1e4, a * 1e4, u) for x, v, a, u in live], t0)

    chunk = _chunk(seed=2)
    before = _array(chunk).copy()
    out = _array(st.stitch(chunk, 0, 600))

    assert st.n_failed == 1
    np.testing.assert_array_equal(out, before)
    # A chunk installed without a solve behind it must not leave the next
    # handoff reading a trajectory the robot never executed.
    st.commit()
    assert st._live is None


def test_a_rejected_chunk_leaves_the_live_trajectory_alone():
    st = _stitcher()
    st.stitch(_chunk(), 0, 0)
    st.commit()
    live = st._live

    st.stitch(_chunk(seed=4), 50, 20)

    assert st._live is live


def test_reset_drops_the_live_trajectory():
    st = _stitcher()
    st.stitch(_chunk(), 0, 0)
    st.commit()

    st.reset()

    assert st._live is None
    assert st._pending is None


def test_channels_of_unequal_width_are_split_back_correctly():
    """A gripper is not the same width as an arm, and must come back whole."""
    keys = ["wide", "narrow"]
    st = TrajectoryStitcher(_config(), CONTROL_HZ, keys)
    if not st.enabled:
        pytest.skip("osqp is not installed")
    rng = np.random.default_rng(3)
    actions = {
        "wide": rng.standard_normal((CHUNK_LEN, 20)).tolist(),
        "narrow": rng.standard_normal((CHUNK_LEN, 7)).tolist(),
    }

    out = st.stitch(actions, 50, 0)

    assert np.shape(out["wide"]) == (CHUNK_LEN, 20)
    assert np.shape(out["narrow"]) == (CHUNK_LEN, 7)


def test_a_malformed_chunk_is_left_alone():
    st = _stitcher()
    ragged = {KEYS[0]: [[0.0] * N_JOINTS] * 10, KEYS[1]: [[0.0] * N_JOINTS]}

    assert st.stitch(ragged, 0, 0) is ragged
    assert st.n_solved == 0


def test_an_exhausted_chunk_restarts_from_its_held_position():
    stitcher = _stitcher()
    previous = stitcher.stitch(_chunk(n_steps=64))
    stitcher.commit()
    held_position = _array(previous)[-1]
    live = stitcher._live[0]
    previous_state = np.array(
        [
            _eval_cubic(*joint, stitcher._config.solver_dt, 63 / CONTROL_HZ)
            for joint in live
        ]
    )
    assert np.max(np.abs(previous_state[:, 1:])) > 1e-3
    hold = {key: [previous[key][-1]] * 64 for key in KEYS}

    result = stitcher.stitch(hold, 0, 64, held_actions=previous)

    np.testing.assert_allclose(
        _array(result), np.tile(held_position, (64, 1)), atol=1e-4
    )
    initial_state = np.array(
        [
            _eval_cubic(*joint, stitcher._config.solver_dt, 0.0)
            for joint in stitcher._pending[0]
        ]
    )
    np.testing.assert_allclose(initial_state[:, 0], held_position, atol=1e-4)
    np.testing.assert_allclose(initial_state[:, 1:], 0.0, atol=1e-4)
    assert stitcher.n_failed == 0
    assert stitcher.n_overrun == 0


def test_a_restart_without_a_live_curve_still_uses_the_held_position():
    stitcher = _stitcher()
    previous = {key: [[0.1] * N_JOINTS] for key in KEYS}
    request = {key: [[0.2] * N_JOINTS] * CHUNK_LEN for key in KEYS}

    result = stitcher.stitch(request, held_actions=previous)

    np.testing.assert_allclose(_array(result)[0], 0.1, atol=1e-4)
    assert np.all(_array(result)[-1] > 0.15)
    assert stitcher.n_failed == 0


@pytest.mark.parametrize(
    "step_count, start_idx",
    [(10, 7), (10, 8), (10, 9), (1, 0), (2, 0), (3, 0)],
)
def test_short_windows_keep_the_requested_origin(step_count, start_idx):
    stitcher = _stitcher()
    stitcher.stitch(_chunk(n_steps=64))
    stitcher.commit()
    expected_state = np.array(
        [
            _eval_cubic(*joint, stitcher._config.solver_dt, 20 / CONTROL_HZ)
            for joint in stitcher._live[0]
        ]
    )
    targets = expected_state[:, 0] + 0.02
    request = {
        KEYS[0]: np.tile(targets[:N_JOINTS], (step_count, 1)).tolist(),
        KEYS[1]: np.tile(targets[N_JOINTS:], (step_count, 1)).tolist(),
    }
    before = _array(request).copy()

    result = _array(stitcher.stitch(request, start_idx, 20))

    assert stitcher.n_solved == 2
    assert stitcher.n_failed == 0
    np.testing.assert_array_equal(result[:start_idx], before[:start_idx])
    np.testing.assert_allclose(
        result[start_idx], expected_state[:, 0], atol=1e-4
    )
    actual_state = np.array(
        [
            _eval_cubic(*joint, stitcher._config.solver_dt, 0.0)
            for joint in stitcher._pending[0]
        ]
    )
    np.testing.assert_allclose(actual_state, expected_state, atol=1e-4)
