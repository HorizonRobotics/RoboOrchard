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

"""Offline replay harness for the async chunk pipeline.

Reproduces the executed command stream from a ground-truth trajectory plus a
schedule of inference round trips, so a change to the stitcher can be scored
against the seams a real run actually produced. Runs without ROS.

The pieces that matter and that earlier ad-hoc tests kept getting wrong:

* the RTC pin -- the server copies the first ``pin`` samples of the new chunk
  from the tail the client sent, so consecutive chunks already agree there and
  the stitcher is not absorbing a discontinuity that does not exist;
* ``start_idx`` and ``prev_ran`` refer to the same instant but count from
  different origins, so they differ by the gap between install and request;
* a rejected chunk leaves ``_chunk_start_idx`` alone, so ``prev_ran`` keeps
  accumulating across the rejection.
"""

from __future__ import annotations
import sys

import numpy as np

sys.path.insert(0, "ros2_package/robo_orchard_deploy_ros2")
from robo_orchard_deploy_ros2.config import (  # noqa: E402
    TrajectoryStitchConfig,
)
from robo_orchard_deploy_ros2.trajectory_stitcher import (  # noqa: E402
    TrajectoryStitcher,
)

CONTROL_HZ = 200.0
N_JOINTS = 14
CHUNK_LEN = 426


def ground_truth(path: str, n: int, smooth: int = 21) -> np.ndarray:
    """A long, smooth, realistic trajectory to predict against.

    Taken from a recorded run and low-pass filtered: the recording is a
    concatenation of chunk fragments, and its seams would otherwise be read
    as motion the model predicted.
    """
    d = np.load(path)
    raw = d["cmd"].astype(float)
    keep = np.diff(d["t"], prepend=d["t"][0]) < 0.05
    raw = raw[keep]
    k = np.ones(smooth) / smooth
    out = np.stack(
        [np.convolve(raw[:, j], k, mode="valid") for j in range(N_JOINTS)],
        axis=1,
    )
    if len(out) < n:
        out = np.vstack([out, out[::-1]])[: max(n, len(out))]
    return out[:n]


def predict(
    g: np.ndarray,
    q: int,
    pinned: np.ndarray | None,
    rng=None,
    plan_noise: float = 0.0,
) -> dict:
    """One server response: a chunk indexed from its own observation.

    ``pinned`` is written over the head verbatim, which is what
    ``RTCInferencePlugin.forward`` does with the client's remaining actions.

    ``plan_noise`` perturbs the free tail: successive predictions from a real
    model disagree there, and that disagreement is what the solve trades
    against continuity. Without it every chunk agrees with its predecessor
    and the seams come out an order of magnitude cleaner than a real run.
    """
    chunk = g[q : q + CHUNK_LEN].copy()
    if plan_noise and rng is not None:
        n = len(chunk)
        w = np.cumsum(rng.standard_normal((n, N_JOINTS)), axis=0)
        w -= w[0]
        chunk = chunk + plan_noise * w
    if pinned is not None:
        m = min(len(pinned), len(chunk))
        chunk[:m] = pinned[:m]
    return {
        "left_arm_actions": chunk[:, :7].tolist(),
        "right_arm_actions": chunk[:, 7:].tolist(),
    }


def _arr(actions: dict) -> np.ndarray:
    return np.hstack(
        [
            np.asarray(actions["left_arm_actions"]),
            np.asarray(actions["right_arm_actions"]),
        ]
    )


def replay(
    g: np.ndarray,
    round_trips: list[int],
    *,
    pin: int = 80,
    max_delay_horizon: int = 130,
    idx_quant=16,
    request_lag: int = 2,
    solve_lag: int = 2,
    plan_noise: float = 0.0,
    seed: int = 0,
    chunk_origin: str = "solve",
):
    """Run the schedule and return the executed stream and per-install facts.

    ``round_trips`` is one entry per inference, in control steps: the delay
    between the request going out and the chunk being ready to install.

    ``solve_lag`` is the gap between ``start_idx``, read before the solve,
    and ``current_action_idx``, read after it. The node computes them at
    different instants, so the arm resumes that many samples past the point
    the solve pinned.

    ``chunk_origin`` is where the next handover counts ``prev_ran`` from.
    The solved trajectory has its time origin at ``start_idx``, so "solve"
    is correct; "install" reproduces counting from the index the chunk was
    installed at, which is ``solve_lag`` samples further on.
    """
    if chunk_origin not in {"solve", "install"}:
        raise ValueError("chunk_origin must be 'solve' or 'install'")
    rng = np.random.default_rng(seed)
    st = TrajectoryStitcher(
        TrajectoryStitchConfig(
            max_velocity=3.0,
            max_acceleration=10.0,
            max_jerk=500.0,
            horizon_quantum=idx_quant,
        ),
        CONTROL_HZ,
        ["left_arm_actions", "right_arm_actions"],
    )
    assert st.enabled, "osqp is required"

    # Chunk 0: nothing to pin against, installed at index 0.
    chunk = st.stitch(predict(g, 0, None, rng, plan_noise), 0, 0)
    st.commit()
    live_chunk, live_q = _arr(chunk), 0
    install = 0  # global control step of the last install

    executed = [live_chunk[0]]
    stamps = [0]
    events = []
    now = 0

    for rt in round_trips:
        request = now + request_lag
        r = request - live_q  # index in the live chunk at request time
        if r >= len(live_chunk):
            break
        # Advance the arm to the request instant.
        for gstep in range(now + 1, request + 1):
            k = gstep - live_q
            if k >= len(live_chunk):
                break
            executed.append(live_chunk[k])
            stamps.append(gstep)
        pinned = live_chunk[r : r + pin]
        ready = request + rt

        # Advance the arm while the server works.
        for gstep in range(request + 1, ready + 1):
            k = gstep - live_q
            if k >= len(live_chunk):
                break
            executed.append(live_chunk[k])
            stamps.append(gstep)

        new_chunk = predict(g, request, pinned, rng, plan_noise)
        start_idx = ready - request
        prev_ran = ready - install
        before = st.n_failed
        solved = st.stitch(new_chunk, start_idx, prev_ran)
        failed = st.n_failed != before

        rec = dict(
            t=ready,
            start_idx=start_idx,
            prev_ran=prev_ran,
            cur_delay=start_idx,
            failed=failed,
            rejected=False,
            seam=float("nan"),
        )
        if start_idx > max_delay_horizon:
            rec["rejected"] = True  # solve discarded, live chunk untouched
            events.append(rec)
            now = ready
            continue

        st.commit()
        new_arr = _arr(solved)
        # The solve pinned index ``start_idx``; the arm resumes ``solve_lag``
        # samples later, because the node reads the index again after the
        # solve returns.
        resume = min(start_idx + solve_lag, len(new_arr) - 1)
        old_at_resume = min(ready + solve_lag - live_q, len(live_chunk) - 1)
        prev_cmd = live_chunk[old_at_resume]
        next_cmd = new_arr[resume]
        rec["seam"] = float(np.abs(next_cmd - prev_cmd).max())
        events.append(rec)

        live_chunk, live_q = new_arr, request
        install = ready if chunk_origin == "solve" else ready + solve_lag
        now = ready + solve_lag

    return np.array(executed), np.array(stamps), events
