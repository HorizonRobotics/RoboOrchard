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

import math

from robo_orchard_piper_ros2.oscillation_guard import OscillationGuard

DT = 0.005  # 200 Hz command rate, same as the deployed teleop stack


def _run(guard, signal, dt=DT, t0=0.0):
    """Feed a 1-joint velocity signal; return the gain trace."""
    gains = []
    for i, v in enumerate(signal):
        gains.append(guard.update([v] + [0.0] * 5, t0 + i * dt))
    return gains


def _sine(freq_hz, amplitude, seconds, dt=DT):
    n = int(seconds / dt)
    return [
        amplitude * math.sin(2.0 * math.pi * freq_hz * i * dt)
        for i in range(n)
    ]


def test_slow_motion_never_trips():
    # 0.5 Hz at 1.2 rad/s: fast, smooth teleop. Below the 1.5 Hz band
    # edge, so the high-pass strips it regardless of amplitude.
    guard = OscillationGuard()
    gains = _run(guard, _sine(0.5, 1.2, 6.0))
    assert min(gains) == 1.0


def test_single_reversal_never_trips():
    # One sharp direction change: at most a couple of band
    # half-cycles, then nothing — a legitimate transient.
    guard = OscillationGuard()
    signal = [1.0] * 200 + [-1.0] * 200 + [0.0] * 400
    gains = _run(guard, signal)
    assert min(gains) == 1.0


def test_small_oscillation_never_trips():
    # Sustained 3.5 Hz but below min_amplitude: tremor-level noise.
    guard = OscillationGuard(min_amplitude=0.15)
    gains = _run(guard, _sine(3.5, 0.05, 5.0))
    assert min(gains) == 1.0


def test_sustained_oscillation_trips_fast():
    # The blow-up signature: 3.5 Hz well above threshold. Five
    # half-cycles is ~0.71 s; the gain must be fully off within 1.5 s.
    guard = OscillationGuard()
    gains = _run(guard, _sine(3.5, 0.5, 3.0))
    trip_index = next(i for i, g in enumerate(gains) if g < 1.0)
    assert trip_index * DT < 1.5
    assert gains[int(1.5 / DT)] == 0.0
    assert guard.trip_joint == 0


def test_recovers_after_oscillation_stops():
    guard = OscillationGuard(release_seconds=1.5)
    _run(guard, _sine(3.5, 0.5, 3.0))
    gains = _run(guard, [0.0] * int(3.0 / DT), t0=3.0)
    assert gains[-1] == 1.0
    # ...but not instantly: still ramping midway through the release.
    assert gains[int(0.8 / DT)] < 1.0


def test_gain_applies_to_worst_joint_anywhere():
    # Oscillation on joint 5 alone must trip the shared gain.
    guard = OscillationGuard()
    n = int(3.0 / DT)
    gains = []
    for i in range(n):
        vels = [0.0] * 6
        vels[4] = 0.5 * math.sin(2.0 * math.pi * 3.5 * i * DT)
        gains.append(guard.update(vels, i * DT))
    assert min(gains) == 0.0
    assert guard.trip_joint == 4


def test_command_gap_resets_detector():
    # 3 half-cycles, a 1 s stream gap, 3 more half-cycles: the train
    # must not be stitched together across the gap.
    guard = OscillationGuard(trip_half_cycles=5)
    burst = _sine(3.5, 0.5, 3.0 / (2 * 3.5))
    gains = _run(guard, burst)
    gains += _run(guard, burst, t0=len(burst) * DT + 1.0)
    assert min(gains) == 1.0
