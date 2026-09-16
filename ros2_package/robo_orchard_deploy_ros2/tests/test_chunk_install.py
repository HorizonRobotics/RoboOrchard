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

"""Where a new chunk is installed, and what the solve is handed.

Drives the inference callback directly against fakes, because the
arithmetic it does -- which step the chunk starts at, and how far the one
it replaces got -- is not visible from the outside.
"""

from __future__ import annotations
import threading
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from robo_orchard_deploy_ros2.config import TrajectoryStitchConfig
from robo_orchard_deploy_ros2.node.async_node import DeployNode, NodeState
from robo_orchard_deploy_ros2.trajectory_stitcher import (
    TrajectoryStitcher,
    _eval_cubic,
    _PiecewiseJerkQP,
)

CHUNK = {"actions": [[float(i)] for i in range(100)]}


class _Logger:
    def __init__(self):
        self.warnings = []
        self.debug_messages = []

    def debug(self, message, **kwargs):
        self.debug_messages.append(message)

    def warning(self, message, **kwargs):
        self.warnings.append(message)

    def error(self, message, **kwargs):
        pass

    def info(self, message, **kwargs):
        pass


class _Stitcher:
    """Records what it was asked to solve, and how the caller used it."""

    enabled = True

    def __init__(self):
        self.calls = []
        self.commits = 0
        self.held_actions = None

    def stitch(self, actions, install_idx, prev_ran, *, held_actions=None):
        self.calls.append((install_idx, prev_ran))
        self.held_actions = held_actions
        return actions

    def commit(self):
        self.commits += 1

    def reset(self):
        pass

    def discard_pending(self):
        pass


class _ActionExecutor:
    def __init__(self):
        self.sent = []

    def action_step_count(self, actions):
        return len(actions["actions"])

    def send_action(self, actions, index):
        self.sent.append(actions["actions"][index])

    def reset_limiter(self):
        pass

    def remaining_actions(self, actions, index):
        steps = actions["actions"]
        return {"actions": steps[index:]} if index < len(steps) else {}


def _node(
    current_actions,
    current_action_idx,
    chunk_start_idx,
    response,
    elapsed=0,
):
    """Build a node mid-run.

    ``elapsed`` is how many control steps the robot gets through while the
    request is in flight, which is what decides where the answer can be
    installed.
    """
    node = DeployNode.__new__(DeployNode)
    node.shared_state_lock = threading.Lock()
    node._inference_generation = 0
    node._status_publisher = Mock()
    node._event_publisher = Mock()
    node.get_clock = Mock()
    node.state = NodeState.EXECUTING
    node.current_actions = current_actions
    node.current_action_idx = current_action_idx
    node._chunk_start_idx = chunk_start_idx
    node._pending_actions = None
    node._stitch_lead_steps = 0
    node.config = SimpleNamespace(
        control_config=SimpleNamespace(control_frequency=200.0)
    )
    node.max_delay_horizon = 100
    node._stitcher = _Stitcher()
    node.action_executor = _ActionExecutor()
    node._logger = _Logger()
    node.get_logger = lambda: node._logger
    node.obs_manager = type(
        "_Obs", (), {"get_observations": lambda self: {"image": 1}}
    )()

    def request_inference(obs):
        node.current_action_idx += elapsed
        return response

    node.model_inferencer = type(
        "_Model", (), {"request_inference": staticmethod(request_inference)}
    )()
    return node


def test_a_slow_solve_switches_at_its_planned_step():
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)
    node._stitch_lead_steps = 4
    stitcher = node._stitcher

    def slow(actions, install_idx, prev_ran, **kwargs):
        assert not node.shared_state_lock.locked()
        stitcher.calls.append((install_idx, prev_ran))
        node._action_timer_callback()
        node._action_timer_callback()
        return actions

    stitcher.stitch = slow
    node._model_infer_timer_callback()

    assert stitcher.calls == [(16, 46)]
    assert node.current_actions == CHUNK
    assert stitcher.commits == 0
    node._action_timer_callback()
    node._action_timer_callback()
    assert node.current_action_idx == 46
    node._action_timer_callback()
    assert node._chunk_start_idx == 16
    assert node.current_action_idx == 17
    assert node.action_executor.sent == [[42.0], [43.0], [44.0], [45.0], [9.0]]
    assert stitcher.commits == 1


def test_a_solve_that_misses_its_planned_step_keeps_the_live_chunk():
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)
    node._stitch_lead_steps = 2
    stitcher = node._stitcher

    def slow(actions, install_idx, prev_ran, **kwargs):
        stitcher.calls.append((install_idx, prev_ran))
        for _ in range(3):
            node._action_timer_callback()
        return actions

    stitcher.stitch = slow
    node._model_infer_timer_callback()

    assert stitcher.calls == [(14, 44)]
    assert node.current_actions == CHUNK
    assert node.current_action_idx == 45
    assert node._pending_actions is None
    assert stitcher.commits == 0


def test_the_first_chunk_installs_at_the_top():
    node = _node(None, 0, 0, dict(CHUNK))

    node._model_infer_timer_callback()

    assert node._stitcher.calls == [(0, 0)]
    assert node.current_action_idx == 0
    assert node._chunk_start_idx == 0
    assert node._stitcher.commits == 1


def test_the_round_trip_sets_the_install_step():
    """The robot joins the chunk however far the round trip carried it."""
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)

    node._model_infer_timer_callback()

    assert node._stitcher.calls == [(12, 42)]
    assert node.current_action_idx == 12
    assert node._chunk_start_idx == 12


def test_prev_ran_accumulates_across_a_rejected_chunk():
    """A rejected chunk leaves the start index alone.

    The next solve then sees how far the live chunk really got, not how
    far the last install was.
    """
    node = _node(dict(CHUNK), 90, 20, {"actions": [[9.0]] * 100}, elapsed=5)

    node._model_infer_timer_callback()

    _, prev_ran = node._stitcher.calls[0]
    assert prev_ran == 75


def test_an_exhausted_chunk_hands_over_from_its_end():
    """An exhausted chunk hands over from its end.

    No steps remain, so restart from its held position rather than
    continuing the velocity and acceleration of its old curve.
    """
    node = _node(dict(CHUNK), 100, 0, {"actions": [[9.0]] * 100})

    node._model_infer_timer_callback()

    install_idx, prev_ran = node._stitcher.calls[0]
    assert install_idx == 0
    assert prev_ran == 100
    assert node._stitcher.held_actions == CHUNK


def test_an_excessive_round_trip_installs_nothing():
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=30)
    node.max_delay_horizon = 10

    node._model_infer_timer_callback()

    assert node._stitcher.calls == []
    assert node._stitcher.commits == 0
    assert node.current_actions == CHUNK
    assert any("Excessive latency" in m for m in node._logger.warnings)


def test_an_unchanged_response_is_not_reinstalled():
    node = _node(dict(CHUNK), 30, 0, dict(CHUNK), elapsed=12)

    node._model_infer_timer_callback()

    assert node._stitcher.calls == []
    assert node.current_action_idx == 42


def test_a_solve_finishing_at_the_planned_step_switches_immediately():
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)
    node._stitch_lead_steps = 4
    stitcher = node._stitcher

    def slow(actions, install_idx, prev_ran, **kwargs):
        stitcher.calls.append((install_idx, prev_ran))
        for _ in range(4):
            node._action_timer_callback()
        return actions

    stitcher.stitch = slow
    node._model_infer_timer_callback()

    assert stitcher.calls == [(16, 46)]
    assert node.current_action_idx == 16
    assert node._chunk_start_idx == 16
    assert node._pending_actions is None
    assert stitcher.commits == 1


def test_a_pending_handover_is_not_overwritten_by_another_inference():
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)
    node._stitch_lead_steps = 4
    node._model_infer_timer_callback()
    pending = node._pending_actions

    node._model_infer_timer_callback()

    assert node._pending_actions is pending
    assert node._stitcher.calls == [(16, 46)]
    assert node.current_action_idx == 42


@pytest.mark.parametrize("response", [None])
def test_no_response_installs_nothing(response):
    node = _node(dict(CHUNK), 30, 0, response)
    node.get_logger().error = lambda *a, **k: None

    node._model_infer_timer_callback()

    assert node._stitcher.calls == []
    assert node.current_actions == CHUNK


def test_the_planned_step_does_not_exceed_the_latency_limit():
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)
    node.max_delay_horizon = 14
    node._stitch_lead_steps = 4
    node._model_infer_timer_callback()

    assert node._stitcher.calls == [(14, 44)]
    assert node._pending_actions[0] == 44
    assert node.current_actions == CHUNK


def test_the_previous_solve_duration_sets_the_next_lead_time(monkeypatch):
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)
    times = iter([0.0, 0.012])
    monkeypatch.setattr(
        "robo_orchard_deploy_ros2.node.async_node.perf_counter",
        lambda: next(times),
    )
    node._model_infer_timer_callback()

    assert node._stitch_lead_steps == 4


def test_the_next_handover_reads_the_full_elapsed_span():
    """Two installs in a row: the second must count from the first solve."""
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)
    node._stitch_lead_steps = 3
    stitcher = node._stitcher

    def slow(actions, install_idx, prev_ran, **kwargs):
        stitcher.calls.append((install_idx, prev_ran))
        for _ in range(3):
            node._action_timer_callback()
        return actions

    stitcher.stitch = slow
    node._model_infer_timer_callback()
    assert node._chunk_start_idx == 15
    for _ in range(40):
        node._action_timer_callback()
    node._stitch_lead_steps = 3

    node.model_inferencer = type(
        "_M",
        (),
        {
            "request_inference": staticmethod(
                lambda obs: {"actions": [[1.0]] * 100}
            )
        },
    )()
    node._model_infer_timer_callback()

    assert stitcher.calls[-1] == (3, 43)
    assert node._chunk_start_idx == 3
    assert stitcher.commits == 2


def test_without_stitching_the_chunk_switches_without_waiting():
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)
    node._stitch_lead_steps = 4
    node._stitcher.enabled = False

    node._model_infer_timer_callback()

    assert node.current_action_idx == 12
    assert node.current_actions == {"actions": [[9.0]] * 100}
    assert node._pending_actions is None
    assert node._stitcher.calls == []


@pytest.mark.parametrize(
    "old_length, new_length, start_idx, switch_idx",
    [(44, 100, 13, 43), (100, 15, 14, 44)],
)
def test_the_planned_step_stays_within_both_chunks(
    old_length, new_length, start_idx, switch_idx
):
    node = _node(
        {"actions": CHUNK["actions"][:old_length]},
        30,
        0,
        {"actions": [[9.0]] * new_length},
        elapsed=12,
    )
    node._stitch_lead_steps = 4

    node._model_infer_timer_callback()

    assert node._stitcher.calls == [(start_idx, switch_idx)]
    assert node._pending_actions[0] == switch_idx
    assert node._stitcher.held_actions is None


def test_real_solver_matches_the_state_at_the_scheduled_handover():
    config = TrajectoryStitchConfig(
        max_velocity=3.0, max_acceleration=10.0, max_jerk=500.0
    )
    stitcher = TrajectoryStitcher(config, 200.0, ["actions"])
    if not stitcher.enabled:
        pytest.skip("osqp is not installed")
    previous = stitcher.stitch(
        {"actions": np.linspace(0.0, 0.1, 100)[:, None].tolist()}
    )
    stitcher.commit()
    live = stitcher._live
    expected = _eval_cubic(*live[0][0], config.solver_dt, 46 / 200.0)
    node = _node(previous, 30, 0, {"actions": [[0.2]] * 100}, elapsed=12)
    node._stitcher = stitcher
    node._stitch_lead_steps = 4
    solve = stitcher.stitch

    def slow(actions, install_idx, prev_ran, **kwargs):
        result = solve(actions, install_idx, prev_ran, **kwargs)
        node._action_timer_callback()
        node._action_timer_callback()
        return result

    stitcher.stitch = slow
    node._model_infer_timer_callback()

    assert stitcher._live is live
    assert stitcher.n_failed == 0
    pending = stitcher._pending
    actual = _eval_cubic(*pending[0][0], config.solver_dt, 0.0)
    np.testing.assert_allclose(actual, expected, atol=1e-4)
    for _ in range(3):
        node._action_timer_callback()
    assert stitcher._live is pending
    assert node._chunk_start_idx == 16
    np.testing.assert_allclose(
        node.action_executor.sent[-1], [expected[0]], atol=1e-4
    )


@pytest.mark.parametrize("resume", [False, True])
@pytest.mark.parametrize("empty", [False, True])
@pytest.mark.parametrize("stitching", [False, True])
def test_pausing_during_inference_discards_the_response(
    resume, empty, stitching
):
    node = _node(
        None if empty else dict(CHUNK),
        0 if empty else 30,
        0,
        {"actions": [[9.0]] * 100},
    )
    node._stitcher.enabled = stitching

    def request_inference(observations):
        node._disable_inference_callback(None, SimpleNamespace())
        if resume:
            node._enable_inference_callback(None, SimpleNamespace())
        return {"actions": [[9.0]] * 100}

    node.model_inferencer.request_inference = request_inference
    node._model_infer_timer_callback()

    assert node.current_actions is None
    assert node.current_action_idx == 0
    assert node._inference_generation == (2 if resume else 1)
    assert node._pending_actions is None
    assert node._stitcher.calls == []
    assert node._logger.debug_messages == [
        "Discarding stale inference response."
    ]


@pytest.mark.parametrize("resume", [False, True])
@pytest.mark.parametrize("empty", [False, True])
def test_pausing_during_a_solve_does_not_queue_an_obsolete_switch(
    resume, empty
):
    node = _node(
        None if empty else dict(CHUNK),
        0 if empty else 30,
        0,
        {"actions": [[9.0]] * 100},
    )
    node._stitch_lead_steps = 4
    solve = node._stitcher.stitch

    def slow(actions, install_idx, prev_ran, **kwargs):
        result = solve(actions, install_idx, prev_ran, **kwargs)
        node._disable_inference_callback(None, SimpleNamespace())
        if resume:
            node._enable_inference_callback(None, SimpleNamespace())
        return result

    node._stitcher.stitch = slow
    node._model_infer_timer_callback()

    assert node.current_actions is None
    assert node._pending_actions is None
    assert node._stitcher.commits == 0
    if resume:
        node._stitcher.stitch = solve
        node._model_infer_timer_callback()
        assert node.current_actions == {"actions": [[9.0]] * 100}
        assert node._stitcher.commits == 1


@pytest.mark.parametrize("resume", [False, True])
@pytest.mark.parametrize("empty", [False, True])
@pytest.mark.parametrize("phase", ["before", "during", "after"])
def test_restart_around_real_solve_discards_all_stale_state(
    monkeypatch, resume, empty, phase
):
    actions = {"actions": [[0.2]] * 100}
    node = _node(
        None if empty else {"actions": [[0.0]] * 100},
        0 if empty else 20,
        0,
        actions,
    )
    stitcher = TrajectoryStitcher(
        TrajectoryStitchConfig(
            max_velocity=3.0, max_acceleration=10.0, max_jerk=500.0
        ),
        200.0,
        ["actions"],
    )
    if not stitcher.enabled:
        pytest.skip("osqp is not installed")
    node._stitcher = stitcher
    original_stitch = stitcher.stitch
    original_solve = _PiecewiseJerkQP.solve

    def pause():
        assert node._disable_inference_callback(
            None, SimpleNamespace()
        ).success
        if resume:
            assert node._enable_inference_callback(
                None, SimpleNamespace()
            ).success

    def stitch_across_pause(*args, **kwargs):
        if phase == "before":
            pause()
        result = original_stitch(*args, **kwargs)
        if phase == "after":
            pause()
        return result

    def solve_across_pause(solver, state, reference):
        pause()
        return original_solve(solver, state, reference)

    if phase == "during":
        monkeypatch.setattr(_PiecewiseJerkQP, "solve", solve_across_pause)
    else:
        monkeypatch.setattr(stitcher, "stitch", stitch_across_pause)
    node._model_infer_timer_callback()
    node._action_timer_callback()

    assert stitcher.n_solved == 1
    assert node.action_executor.sent == []
    assert node.current_actions is None
    assert node._pending_actions is None
    assert stitcher._pending is None
    assert stitcher._live is None
    assert node._stitch_lead_steps == 1

    monkeypatch.setattr(stitcher, "stitch", original_stitch)
    monkeypatch.setattr(_PiecewiseJerkQP, "solve", original_solve)
    if resume:
        node._model_infer_timer_callback()
        node._action_timer_callback()
        assert len(node.action_executor.sent) == 1
        assert stitcher._live is not None


def test_disable_waits_for_publication_of_selected_action():
    node = _node({"actions": [[0.2]]}, 0, 0, None)
    selected = threading.Event()
    release_publish = threading.Event()
    disable_started = threading.Event()
    disabled = threading.Event()
    operations = []
    original_send = node.action_executor.send_action

    def delayed_send(actions, action_index):
        selected.set()
        assert release_publish.wait(timeout=5)
        original_send(actions, action_index)
        operations.append("publish")

    def disable():
        disable_started.set()
        response = node._disable_inference_callback(None, SimpleNamespace())
        assert response.success
        operations.append("disable_returned")
        disabled.set()

    node.action_executor.send_action = delayed_send
    with ThreadPoolExecutor(max_workers=2) as executor:
        action_future = executor.submit(node._action_timer_callback)
        try:
            assert selected.wait(timeout=5)
            disable_future = executor.submit(disable)
            assert disable_started.wait(timeout=5)
            assert not disabled.wait(timeout=0.05)
        finally:
            release_publish.set()
        action_future.result(timeout=5)
        disable_future.result(timeout=5)

    assert node.state == NodeState.PAUSED
    assert node.current_actions is None
    assert operations == ["publish", "disable_returned"]
    node._action_timer_callback()
    assert node.action_executor.sent == [[0.2]]


def test_handover_does_not_require_exporting_remaining_actions():
    node = _node(dict(CHUNK), 30, 0, {"actions": [[9.0]] * 100}, elapsed=12)
    node.action_executor.remaining_actions = lambda actions, index: {}
    node._stitch_lead_steps = 4
    solve = node._stitcher.stitch

    def slow(actions, install_idx, prev_ran, **kwargs):
        result = solve(actions, install_idx, prev_ran, **kwargs)
        node._action_timer_callback()
        node._action_timer_callback()
        return result

    node._stitcher.stitch = slow
    node._model_infer_timer_callback()

    assert node._stitcher.calls == [(4, 46)]
    assert node._stitcher.held_actions is None
    for _ in range(3):
        node._action_timer_callback()
    assert node.current_action_idx == 5
    assert node._chunk_start_idx == 4
    assert node._stitcher.commits == 1


def test_real_solver_keeps_an_exhausted_chunk_stationary():
    config = TrajectoryStitchConfig(
        max_velocity=3.0, max_acceleration=10.0, max_jerk=500.0
    )
    stitcher = TrajectoryStitcher(config, 200.0, ["actions"])
    if not stitcher.enabled:
        pytest.skip("osqp is not installed")
    previous = stitcher.stitch(
        {"actions": np.linspace(0.0, 0.1, 100)[:, None].tolist()}, 10
    )
    stitcher.commit()
    held_position = previous["actions"][-1]
    hold = {"actions": [held_position] * 100}
    node = _node(previous, 100, 10, hold)
    node._stitcher = stitcher

    node._model_infer_timer_callback()

    np.testing.assert_allclose(
        node.current_actions["actions"], [held_position] * 100, atol=1e-4
    )
    initial_state = _eval_cubic(*stitcher._live[0][0], config.solver_dt, 0.0)
    np.testing.assert_allclose(initial_state[1:], 0.0, atol=1e-4)
    assert node.current_action_idx == 0
    assert node._chunk_start_idx == 0


def test_real_solver_switches_at_index_eight_of_a_ten_step_chunk():
    config = TrajectoryStitchConfig(
        max_velocity=3.0, max_acceleration=10.0, max_jerk=500.0
    )
    stitcher = TrajectoryStitcher(config, 200.0, ["actions"])
    if not stitcher.enabled:
        pytest.skip("osqp is not installed")
    previous = stitcher.stitch(
        {"actions": np.linspace(0.0, 0.1, 100)[:, None].tolist()}
    )
    stitcher.commit()
    expected = _eval_cubic(*stitcher._live[0][0], config.solver_dt, 38 / 200.0)
    node = _node(previous, 30, 0, {"actions": [[0.2]] * 10})
    node._stitcher = stitcher
    node._stitch_lead_steps = 8

    node._model_infer_timer_callback()
    for _ in range(9):
        node._action_timer_callback()

    assert node._chunk_start_idx == 8
    assert node.current_action_idx == 9
    assert node.current_actions["actions"][:8] == [[0.2]] * 8
    np.testing.assert_allclose(
        node.action_executor.sent[-1], [expected[0]], atol=1e-4
    )
    actual = _eval_cubic(*stitcher._live[0][0], config.solver_dt, 0.0)
    np.testing.assert_allclose(actual, expected, atol=1e-4)
