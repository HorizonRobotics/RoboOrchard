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

from dataclasses import dataclass
from typing import Literal

from robo_orchard_pico_msg_ros2.msg import VRState

__all__ = ["ResetIntent"]


@dataclass
class State:
    """Manages the state for long-press trigger activation."""

    start_vr_time_ns: float = 0.0
    is_held: bool = False
    take_action: bool = False


class ResetIntent:
    """Manages the logic for a one-shot reset action on a long-press.

    This class detects if a user holds down a specific button (X/Y for left,
    A/B for right) for a given duration. It returns `True` for a single
    frame when the threshold is met and will not return `True` again until
    the button has been fully released and re-pressed for the duration.
    """

    def __init__(
        self, source_type: Literal["X", "Y", "A", "B"], thresh: float = 2.0
    ):
        """Initializes the ResetIntent detector.

        Args:
            source_type: The button to monitor ("X", "Y", "A", "B").
            thresh: The duration in seconds the button must be held.
        """
        self.source_type = source_type
        self.thresh_ns = thresh * 1e9
        self._state = State()

    def _check_reset_trigger(
        self, is_button_pressed: bool, current_time_ns: int
    ) -> bool:
        """Internal logic to determine if a reset should be triggered now.

        Returns:
            True for the single frame the reset condition is met, False otherwise.
        """  # noqa: E501
        should_reset_now = False

        if is_button_pressed:
            if not self._state.is_held:
                # This is the beginning of a new press-and-hold action.
                self._state.is_held = True
                self._state.take_action = (
                    False  # Allow a new action to be taken.
                )
                self._state.start_vr_time_ns = current_time_ns

            if not self._state.take_action:
                # We are currently holding, and haven't triggered the action yet.  # noqa: E501
                elapsed_duration_ns = (
                    current_time_ns - self._state.start_vr_time_ns
                )
                if elapsed_duration_ns >= self.thresh_ns:
                    # Threshold met. Trigger the reset for this frame only.
                    should_reset_now = True
                    # Mark action as taken to prevent re-triggering until release.  # noqa: E501
                    self._state.take_action = True
        else:
            # Button is not pressed, so reset the held state.
            # This ensures the user must release the button to start a new reset action.  # noqa: E501
            self._state.is_held = False

        return should_reset_now

    def should_reset(self, msg: VRState) -> bool:
        """Checks the VRState message to see if a reset should be performed.

        Args:
            msg: The incoming VRState message.

        Returns:
            True if the reset condition is met for the specified controller.
        """
        is_pressed = False
        if self.source_type == "X":
            is_pressed = msg.left_controller.x_button
        elif self.source_type == "Y":
            is_pressed = msg.left_controller.y_button
        elif self.source_type == "A":
            is_pressed = msg.right_controller.a_button
        elif self.source_type == "B":
            is_pressed = msg.right_controller.b_button
        else:
            raise ValueError(f"Invalid source type: {self.source_type}")

        # The timestamp is used for calculating the hold duration.
        current_time_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

        return self._check_reset_trigger(is_pressed, current_time_ns)
