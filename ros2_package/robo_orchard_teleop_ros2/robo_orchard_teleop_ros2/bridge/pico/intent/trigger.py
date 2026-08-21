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

__all__ = ["TriggerIntent"]


@dataclass
class ActivationState:
    """Manages the state for a hold-to-run activation logic."""

    # The time when a potential activation press started. None if not pressed.
    press_start_time_ns: int | None = None


class LongPressIntent:
    """Manages a "hold-to-run" activation intent with an initial delay."""

    def __init__(
        self,
        button: Literal["trigger", "gripper"],
        source_type: Literal["all", "left", "right"],
        value_thresh: float = 1.0,
        thresh: float = 2.0,
    ):
        self.button = button
        self.source_type = source_type
        self.value_thresh = value_thresh
        self.thresh = thresh
        self.thresh_ns = int(thresh * 1e9)
        self._state = ActivationState()

    def _get_button_values(self, msg: VRState) -> tuple[float, float]:
        if self.button == "trigger":
            left_value = (
                msg.left_controller.trigger * msg.left_controller.status
            )
            right_value = (
                msg.right_controller.trigger * msg.right_controller.status
            )
        elif self.button == "gripper":
            left_value = (
                msg.left_controller.gripper * msg.left_controller.status
            )
            right_value = (
                msg.right_controller.gripper * msg.right_controller.status
            )
        else:
            raise NotImplementedError
        return left_value, right_value

    def _is_pressed(self, left_value: float, right_value: float) -> bool:
        if self.source_type == "all":
            return (
                left_value >= self.value_thresh
                and right_value >= self.value_thresh
            )
        if self.source_type == "left":
            return left_value >= self.value_thresh
        if self.source_type == "right":
            return right_value >= self.value_thresh
        raise ValueError(f"Invalid source type: {self.source_type}")

    def _get_active_state(
        self,
        left_trigger: float,
        right_trigger: float,
        current_vr_time_ns: int,
    ) -> bool:
        """Determines the activation state based on the hold-to-run logic."""
        is_triggered = self._is_pressed(left_trigger, right_trigger)

        if is_triggered:
            # Trigger is currently being held down.
            if self._state.press_start_time_ns is None:
                # This is the first frame the trigger is pressed. Record start time.  # noqa: E501
                self._state.press_start_time_ns = current_vr_time_ns

            # Check if the hold duration has exceeded the activation threshold.
            elapsed_duration_ns = (
                current_vr_time_ns - self._state.press_start_time_ns
            )
            if elapsed_duration_ns >= self.thresh_ns:
                # Held long enough. The system is considered active.
                return True
            else:
                # Still holding, but not for long enough yet. Remain inactive.
                return False
        else:
            # Trigger is released. Instantly deactivate and reset the state.
            self._state.press_start_time_ns = None
            return False

    def is_active(self, msg: VRState) -> bool:
        """Public method to check if the teleoperation should be active.

        Args:
            msg: The incoming VRState message.

        Returns:
            True if the trigger is being held down past the threshold,
            False otherwise.
        """
        left_trigger, right_trigger = self._get_button_values(msg)

        return self._get_active_state(
            left_trigger,
            right_trigger,
            int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec),
        )

    def is_pressed(self, msg: VRState) -> bool:
        """Return whether the configured hold-to-run control is held now."""
        left_value, right_value = self._get_button_values(msg)
        return self._is_pressed(left_value, right_value)

    def reset(self) -> None:
        """Discard any accumulated long-press duration."""
        self._state = ActivationState()


class TriggerIntent(LongPressIntent):
    def __init__(self, *args, **kwargs):
        super().__init__("trigger", *args, **kwargs)


class PicoActivationIntent(LongPressIntent):
    def __init__(self, *args, **kwargs):
        super().__init__("gripper", *args, **kwargs)


class GripperIntent(PicoActivationIntent):
    """Backward-compatible name for the Pico activation control."""
