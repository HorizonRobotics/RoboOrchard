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

from enum import IntEnum


class ActivationState(IntEnum):
    """States published by the teleop activation node."""

    UNAVAILABLE = 0
    INACTIVE = 1
    ACTIVE = 2


class HoldActivationStateMachine:
    """Fail-closed state machine for a hold-to-activate input."""

    def __init__(self, debounce_s: float) -> None:
        if debounce_s < 0.0:
            raise ValueError("debounce_s must be non-negative")

        self._debounce_s = debounce_s
        self.state = ActivationState.UNAVAILABLE
        self.transition_id = 0
        self._connected = False
        self._pressed = False
        self._armed = False
        self._press_started_at: float | None = None

    def connect(self, initially_pressed: bool) -> bool:
        """Mark input available without activating an already-held key."""
        self._connected = True
        self._pressed = initially_pressed
        self._armed = not initially_pressed
        self._press_started_at = None
        return self._set_state(ActivationState.INACTIVE)

    def disconnect(self) -> bool:
        """Fail closed when the input device disappears or cannot be read."""
        self._connected = False
        self._pressed = False
        self._armed = False
        self._press_started_at = None
        return self._set_state(ActivationState.UNAVAILABLE)

    def key_event(self, value: int, now: float) -> bool:
        """Process Linux EV_KEY values: release=0, press=1, repeat=2."""
        if not self._connected or value == 2:
            return False

        if value == 0:
            self._pressed = False
            self._press_started_at = None
            self._armed = True
            return self._set_state(ActivationState.INACTIVE)

        if value != 1 or self._pressed:
            return False

        self._pressed = True
        if self._armed:
            self._press_started_at = now
        return False

    def advance(self, now: float) -> bool:
        """Apply press debounce once a key has remained held long enough."""
        if (
            not self._connected
            or not self._armed
            or not self._pressed
            or self._press_started_at is None
        ):
            return False

        if now - self._press_started_at < self._debounce_s:
            return False

        self._press_started_at = None
        return self._set_state(ActivationState.ACTIVE)

    def _set_state(self, state: ActivationState) -> bool:
        if state == self.state:
            return False
        self.state = state
        self.transition_id += 1
        return True


class HoldActionStateMachine:
    """Emit one action after a key remains held for the configured time."""

    def __init__(self, hold_s: float) -> None:
        if hold_s < 0.0:
            raise ValueError("hold_s must be non-negative")

        self._hold_s = hold_s
        self._connected = False
        self._pressed = False
        self._armed = False
        self._press_started_at: float | None = None

    def connect(self, initially_pressed: bool) -> None:
        """Require a release when the device connects with the key held."""
        self._connected = True
        self._pressed = initially_pressed
        self._armed = not initially_pressed
        self._press_started_at = None

    def disconnect(self) -> None:
        """Discard an in-progress action when the device disappears."""
        self._connected = False
        self._pressed = False
        self._armed = False
        self._press_started_at = None

    def key_event(self, value: int, now: float) -> None:
        """Process Linux EV_KEY values: release=0, press=1, repeat=2."""
        if not self._connected or value == 2:
            return
        if value == 0:
            self._pressed = False
            self._armed = True
            self._press_started_at = None
            return
        if value != 1 or self._pressed:
            return

        self._pressed = True
        if self._armed:
            self._press_started_at = now

    def advance(self, now: float) -> bool:
        """Return True once per press after the hold duration elapses."""
        if (
            not self._connected
            or not self._armed
            or not self._pressed
            or self._press_started_at is None
        ):
            return False
        if now - self._press_started_at < self._hold_s:
            return False

        self._armed = False
        self._press_started_at = None
        return True
