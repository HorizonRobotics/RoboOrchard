# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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

"""Detect sustained command-stream oscillation during Piper teleoperation.

The guard attenuates follower friction and velocity phase-lead terms when
periodic motion indicates feedback instability. Gravity compensation remains
active. The implementation is ROS-independent for offline replay and testing.
"""


class OscillationGuard:
    """Suppress command-derived compensation during sustained oscillation."""

    def __init__(
        self,
        joint_count: int = 6,
        *,
        min_amplitude: float = 0.15,
        trip_half_cycles: int = 5,
        band_low_hz: float = 1.5,
        half_period_range: tuple[float, float] = (0.07, 0.4),
        attack_seconds: float = 0.15,
        release_seconds: float = 1.5,
    ) -> None:
        self.joint_count = joint_count
        self.min_amplitude = float(min_amplitude)
        self.trip_half_cycles = int(trip_half_cycles)
        self.band_low_hz = float(band_low_hz)
        self.half_period_range = (
            float(half_period_range[0]),
            float(half_period_range[1]),
        )
        self.attack_seconds = float(attack_seconds)
        self.release_seconds = float(release_seconds)
        self.gain = 1.0
        # Most recent joint to trip the guard.
        self.trip_joint: int | None = None
        self.reset()

    def reset(self) -> None:
        """Reset the detector while preserving the current gain."""
        n = self.joint_count
        self._prev_time: float | None = None
        self._hp_prev_in = [0.0] * n
        self._hp_prev_out = [0.0] * n
        self._prev_sign = [0] * n
        self._last_crossing = [None] * n
        self._half_cycle_peak = [0.0] * n
        self._count = [0] * n

    @property
    def tripping(self) -> bool:
        """Return whether any joint has tripped the guard."""
        return any(c >= self.trip_half_cycles for c in self._count)

    def update(self, velocities, now: float) -> float:
        """Process one velocity sample and return the compensation gain."""
        prev_time = self._prev_time
        self._prev_time = now
        if prev_time is None:
            return self.gain
        dt = now - prev_time
        if not 1e-4 < dt < 0.5:
            # Reset timing after a command gap, but preserve the gain.
            self.reset()
            self._prev_time = now
            return self.gain

        hp_tau = 1.0 / (2.0 * 3.141592653589793 * self.band_low_hz)
        hp_coeff = hp_tau / (hp_tau + dt)
        min_half, max_half = self.half_period_range
        for j in range(min(self.joint_count, len(velocities))):
            v = float(velocities[j])
            band = hp_coeff * (self._hp_prev_out[j] + v - self._hp_prev_in[j])
            self._hp_prev_in[j] = v
            self._hp_prev_out[j] = band
            peak = abs(band)
            if peak > self._half_cycle_peak[j]:
                self._half_cycle_peak[j] = peak
            sign = 1 if band > 0.0 else (-1 if band < 0.0 else 0)
            if sign != 0 and self._prev_sign[j] not in (0, sign):
                last = self._last_crossing[j]
                if last is not None:
                    half_period = now - last
                    if (
                        min_half < half_period < max_half
                        and self._half_cycle_peak[j] >= self.min_amplitude
                    ):
                        self._count[j] += 1
                        if self._count[j] >= self.trip_half_cycles:
                            self.trip_joint = j
                    else:
                        self._count[j] = 0
                self._last_crossing[j] = now
                self._half_cycle_peak[j] = 0.0
            elif (
                self._last_crossing[j] is not None
                and now - self._last_crossing[j] > max_half
            ):
                self._count[j] = 0
            if sign != 0:
                self._prev_sign[j] = sign

        if self.tripping:
            self.gain = max(0.0, self.gain - dt / self.attack_seconds)
        elif self.gain < 1.0:
            self.gain = min(1.0, self.gain + dt / self.release_seconds)
        return self.gain
