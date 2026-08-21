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

from __future__ import annotations
import math
import time
from typing import Callable

from robo_orchard_pico_msg_ros2.msg import VRState
from robo_orchard_teleop_msg_ros2.msg import TeleopActivationState


class TopicActivationIntent:
    """Adapt activation-state heartbeats to VRTeleOp's intent contract."""

    def __init__(
        self,
        timeout_s: float,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive and finite")
        self.timeout_s = timeout_s
        self._monotonic = monotonic
        self._state = TeleopActivationState.UNAVAILABLE
        self._armed = False
        self._last_message_time_s: float | None = None

    def update(self, message: TeleopActivationState) -> None:
        """Consume one activation heartbeat using local receive time."""
        now_s = self._monotonic()
        if not math.isfinite(now_s):
            raise ValueError("monotonic time must be finite")
        if (
            self._last_message_time_s is None
            or now_s - self._last_message_time_s > self.timeout_s
        ):
            self._armed = False
        self._last_message_time_s = now_s

        state = int(message.state)
        if state not in {
            TeleopActivationState.UNAVAILABLE,
            TeleopActivationState.INACTIVE,
            TeleopActivationState.ACTIVE,
        }:
            state = TeleopActivationState.UNAVAILABLE
        self._state = state
        if state == TeleopActivationState.INACTIVE:
            self._armed = True
        elif state == TeleopActivationState.UNAVAILABLE:
            self._armed = False

    def status(self, now_s: float | None = None) -> tuple[bool, str]:
        """Return whether teleop may run and a diagnostic reason."""
        if now_s is None:
            now_s = self._monotonic()
        if not math.isfinite(now_s):
            raise ValueError("monotonic time must be finite")
        if self._last_message_time_s is None:
            self._armed = False
            return False, "waiting_for_activation"
        if now_s - self._last_message_time_s > self.timeout_s:
            self._armed = False
            return False, "activation_heartbeat_timeout"
        if self._state == TeleopActivationState.UNAVAILABLE:
            self._armed = False
            return False, "activation_unavailable"
        if self._state == TeleopActivationState.INACTIVE:
            return False, "inactive"
        if not self._armed:
            return False, "release_required"
        return True, "active"

    def require_rearm(self) -> None:
        """Reject ACTIVE until a new INACTIVE heartbeat is observed."""
        self._armed = False

    def is_active(self, _message: VRState) -> bool:
        active, _ = self.status()
        return active

    def is_pressed(self, _message: VRState) -> bool:
        """Return whether a fresh heartbeat reports the key as active."""
        if self._last_message_time_s is None:
            return False
        now_s = self._monotonic()
        if now_s - self._last_message_time_s > self.timeout_s:
            return False
        return self._state == TeleopActivationState.ACTIVE

    def reset(self) -> None:
        """Keep keyboard state when only the teleop session resets."""
        # Fault paths call require_rearm() explicitly when release is required.
        return None


class InactiveActivationIntent:
    """Keep an unselected teleop side inactive."""

    def is_active(self, _message: VRState) -> bool:
        return False

    def is_pressed(self, _message: VRState) -> bool:
        return False

    def reset(self) -> None:
        return None
