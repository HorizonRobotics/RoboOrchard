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

"""Bound the step between consecutive joint commands on one channel."""

from __future__ import annotations

import numpy as np


class CommandLimiter:
    """Clamp how far a joint command may move from the one before it.

    The bound is on the published command stream only. The first command
    of a channel has no predecessor, so it is not a step and is not
    bounded; neither is the first command after :meth:`reset`. Bounding
    those would need the robot's measured position, which this class does
    not have.
    """

    def __init__(self, max_velocity, control_frequency, logger=None):
        self._log = logger
        self.max_step = (
            None
            if max_velocity is None
            else float(max_velocity) / float(control_frequency)
        )
        self._last = {}
        self.n_clamped = 0
        self.worst_step = 0.0

    @property
    def enabled(self) -> bool:
        return self.max_step is not None

    def reset(self) -> None:
        """Forget the last command sent on every channel.

        The stored command is a claim about where the robot was heading.
        Call this wherever that claim stops being supported.
        """
        self._last.clear()

    def apply(self, joint_position, key: str):
        """Return the command to send, clamped if it steps too far.

        Clamped rather than dropped: dropping would hold the robot while
        the plan ran away from it.
        """
        if self.max_step is None or joint_position is None:
            return joint_position
        target = np.asarray(joint_position, dtype=float)
        previous = self._last.get(key)
        if previous is not None and len(previous) == len(target):
            step = target - previous
            worst = float(np.abs(step).max())
            if worst > self.max_step:
                self.worst_step = max(self.worst_step, worst)
                self.n_clamped += 1
                target = previous + np.clip(
                    step, -self.max_step, self.max_step
                )
                if self._log is not None:
                    self._log.error(
                        "%s command step %.4f rad over the %.4f rad per-tick "
                        "limit; clamped (%d so far, worst %.4f)"
                        % (
                            key,
                            worst,
                            self.max_step,
                            self.n_clamped,
                            self.worst_step,
                        ),
                        throttle_duration_sec=1.0,
                    )
        self._last[key] = target
        return target.tolist()
