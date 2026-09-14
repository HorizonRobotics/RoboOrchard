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

"""What the handover actually produces, with the real solver in the loop.

The other install tests hand the node a stitcher that only records its
arguments, so they check the arithmetic and nothing downstream of it. This
one runs the schedule through the real solve and measures the command
stream that comes out, which is the only place a wrong index shows up.
"""

from __future__ import annotations

import numpy as np
import pytest
from replay_harness import replay

# The solver is an optional extra, so an install without it skips these
# rather than failing: the stitcher it exercises is off by default.
pytest.importorskip("osqp")
pytest.importorskip("scipy")

CONTROL_HZ = 200.0
N_JOINTS = 14
# A round trip of 26 control steps is 130 ms, which is what the deployed
# model takes.
ROUND_TRIPS = [26] * 120


def _trajectory(n=9000, seed=0):
    """Something smooth enough that a seam is the only discontinuity."""
    rng = np.random.default_rng(seed)
    t = np.arange(n) / CONTROL_HZ
    amp = rng.uniform(0.2, 0.6, N_JOINTS)
    freq = rng.uniform(0.15, 0.5, N_JOINTS)
    phase = rng.uniform(0.0, 2 * np.pi, N_JOINTS)
    return amp * np.sin(2 * np.pi * freq * t[:, None] + phase)


def _seams(**kwargs):
    _, _, events = replay(_trajectory(), ROUND_TRIPS, pin=80, **kwargs)
    seams = [
        e["seam"]
        for e in events
        if not e["rejected"] and not np.isnan(e["seam"])
    ]
    assert len(seams) > 50, "schedule produced too few installs to judge"
    assert not any(e["failed"] for e in events), "a solve failed"
    return np.array(seams)


@pytest.mark.parametrize("solve_lag", [0, 1, 3, 5])
def test_counting_from_the_solve_leaves_no_step_at_the_handover(solve_lag):
    """The solved trajectory's origin is the index the solve was given."""
    seams = _seams(solve_lag=solve_lag, chunk_origin="solve")

    assert seams.max() < 1e-4


@pytest.mark.parametrize("solve_lag", [1, 3, 5])
def test_counting_from_the_install_steps_by_the_solve_duration(solve_lag):
    """The defect this pins: the step scales with how long the solve took.

    Counting the next handover from the index the chunk was installed at
    reads the trajectory ``solve_lag`` samples early, so the new chunk
    starts that far behind the robot at every install.
    """
    seams = _seams(solve_lag=solve_lag, chunk_origin="install")

    assert np.median(seams) > 2e-3 * solve_lag
