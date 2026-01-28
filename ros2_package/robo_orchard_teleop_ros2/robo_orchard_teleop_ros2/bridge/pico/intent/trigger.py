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
    press_start_time_ns: float | None = None


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
        self.thresh_ns = thresh * 1e9
        self._state = ActivationState()

    def _get_active_state(
        self,
        left_trigger: float,
        right_trigger: float,
        current_vr_time_ns: int,
    ) -> bool:
        """Determines the activation state based on the hold-to-run logic."""
        if self.source_type == "all":
            is_triggered = (
                left_trigger >= self.value_thresh
                and right_trigger >= self.value_thresh
            )
        elif self.source_type == "left":
            is_triggered = left_trigger >= self.value_thresh
        elif self.source_type == "right":
            is_triggered = right_trigger >= self.value_thresh
        else:
            raise ValueError(f"Invalid source type: {self.source_type}")

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
        if self.button == "trigger":
            left_trigger = (
                msg.left_controller.trigger * msg.left_controller.status
            )
            right_trigger = (
                msg.right_controller.trigger * msg.right_controller.status
            )
        elif self.button == "gripper":
            left_trigger = (
                msg.left_controller.gripper * msg.left_controller.status
            )
            right_trigger = (
                msg.right_controller.gripper * msg.right_controller.status
            )
        else:
            raise NotImplementedError

        return self._get_active_state(
            left_trigger,
            right_trigger,
            msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec,
        )


class TriggerIntent(LongPressIntent):
    def __init__(self, *args, **kwargs):
        super().__init__("trigger", *args, **kwargs)


class GripperIntent(LongPressIntent):
    def __init__(self, *args, **kwargs):
        super().__init__("gripper", *args, **kwargs)
