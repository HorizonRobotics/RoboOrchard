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

import logging
import math
from dataclasses import dataclass
from enum import Enum, unique
from typing import Callable, Literal

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped

from robo_orchard_pico_msg_ros2.msg import (
    VRState,
)
from robo_orchard_teleop_ros2.bridge.pico.intent import (
    LongPressIntent,
    ResetIntent,
)
from robo_orchard_teleop_ros2.ik import IkOptimizer
from robo_orchard_teleop_ros2.msg_adaptor import (
    matrix_to_pose_msg,
    pose_msg_to_matrix,
)

__all__ = ["Action", "TeleOpResult", "VRTeleOp"]


global_logger = logging.getLogger(__name__)


@dataclass
class ControlState:
    trigger_intent: LongPressIntent
    reset_intent: ResetIntent
    is_active: bool = False
    initial_vr_pose: Pose | PoseStamped | None = None
    initial_ee_pose: Pose | PoseStamped | None = None
    current_ee_pose: Pose | PoseStamped | None = None
    current_joint_state: list[float] | None = None
    filtered_target_ee_pose: Pose | PoseStamped | None = None

    def reset(self):
        self.is_active = False
        self.trigger_intent = type(self.trigger_intent)(
            source_type=self.trigger_intent.source_type,
            value_thresh=self.trigger_intent.value_thresh,
            thresh=self.trigger_intent.thresh,
        )
        self.initial_vr_pose = None
        self.initial_ee_pose = None
        self.current_joint_state = None
        self.filtered_target_ee_pose = None


@dataclass
class TeleOpResult:
    target_ee_pose: Pose
    solution: list[float] | None


@unique
class Action(Enum):
    RESET = 0
    ACTIVE_FAILED = 1
    ACTIVE = 2
    DEACTIVE = 3
    INVALID_VR_MSG = 4


class VRTeleOp:
    def __init__(
        self,
        source_type: Literal["left", "right"],
        urdf_path: str,
        base_link_name: str,
        ee_link_name: str,
        trigger_intent: LongPressIntent,
        reset_intent: ResetIntent,
        scale_factor: float = 1.0,
        pose_low_pass_alpha: float = 0.25,
        reset_callback: Callable | None = None,
        logger=None,
    ):
        if logger is None:
            self.logger = global_logger
        else:
            self.logger = logger

        self.source_type = source_type
        self.scale_factor = scale_factor
        self.pose_low_pass_alpha = pose_low_pass_alpha

        self.control_state = ControlState(
            trigger_intent=trigger_intent, reset_intent=reset_intent
        )

        self.ik_solver = IkOptimizer(
            urdf_path=urdf_path,
            base_link=base_link_name,
            ee_link=ee_link_name,
            clip_to_limit=True,
        )

        self.reset_callback = reset_callback

        self.latest_vr_state: VRState | None = None

    def update_vr_state(self, msg: VRState) -> Action:
        self.latest_vr_state = msg

        control_state = self.control_state

        if self.source_type == "left":
            controller_msg = msg.left_controller
        elif self.source_type == "right":
            controller_msg = msg.right_controller
        else:
            raise ValueError(f"Invalid source_type: {self.source_type}")

        if controller_msg.status == 0:
            self.logger.warning(f"Invalid vr message for {self.source_type}")
            control_state.filtered_target_ee_pose = None
            return Action.INVALID_VR_MSG

        if control_state.reset_intent.should_reset(msg):
            if self.reset_callback is not None:
                self.reset_callback()
            control_state.reset()

            return Action.RESET

        is_intent_active = control_state.trigger_intent.is_active(msg)

        if is_intent_active and not control_state.is_active:
            if control_state.current_ee_pose is None:
                self.logger.warning(
                    f"Attempted to activate {self.source_type} teleop, "
                    "but initial end-effector pose is not yet available."
                )
                return Action.ACTIVE_FAILED

            self.logger.info(
                f"{self.source_type} Teleoperation ACTIVATED. "
                "Capturing baseline poses."
            )

            control_state.is_active = True
            control_state.initial_vr_pose = controller_msg.pose
            control_state.initial_ee_pose = control_state.current_ee_pose
            return Action.ACTIVE

        elif not is_intent_active and control_state.is_active:
            control_state.reset()
            self.logger.info(f"{self.source_type} Teleoperation DEACTIVATED.")
            return Action.DEACTIVE

        else:
            if control_state.is_active:
                return Action.ACTIVE
            else:
                return Action.DEACTIVE

    def update_robot_joint_state(self, msg: list[float]):
        self.control_state.current_joint_state = msg

    def update_robot_ee_pose(self, msg: Pose | PoseStamped):
        self.control_state.current_ee_pose = msg

    def recapture_baseline(self) -> bool:
        """Recapture the engage baseline poses to the most recent inputs.

        Sets control_state.initial_vr_pose / initial_ee_pose to the
        current VR controller pose and robot ee_pose. Useful when an
        external state machine wants to align the baseline to the
        actual moment of engage rather than the moment GripperIntent
        first fired (e.g. after a match-before-engage hand-off, or
        any future re-centering gesture).

        Returns False -- with the existing baseline left intact -- when
        the inputs needed for capture are not yet available (no VR
        frame seen, no robot ee_pose received, or the controller
        reports status == 0).
        """
        if self.latest_vr_state is None:
            return False
        if self.control_state.current_ee_pose is None:
            return False
        if self.source_type == "left":
            controller_msg = self.latest_vr_state.left_controller
        elif self.source_type == "right":
            controller_msg = self.latest_vr_state.right_controller
        else:
            raise ValueError(f"Invalid source_type: {self.source_type}")
        if controller_msg.status == 0:
            return False
        self.control_state.initial_vr_pose = controller_msg.pose
        self.control_state.initial_ee_pose = self.control_state.current_ee_pose
        return True

    def _get_target_ee_pose(self) -> Pose | None:
        if self.latest_vr_state is None:
            return None

        if self.source_type == "left":
            controller_msg = self.latest_vr_state.left_controller
        else:
            controller_msg = self.latest_vr_state.right_controller

        control_state = self.control_state

        if (
            control_state.is_active
            and controller_msg.status != 0
            and control_state.initial_vr_pose is not None
            and control_state.initial_ee_pose is not None
        ):
            T_initial_ee = pose_msg_to_matrix(control_state.initial_ee_pose)  # noqa: N806
            T_initial_vr = pose_msg_to_matrix(control_state.initial_vr_pose)  # noqa: N806
            T_current_vr = pose_msg_to_matrix(controller_msg.pose)  # noqa: N806

            T_local_delta = np.linalg.inv(T_initial_vr) @ T_current_vr  # noqa: N806
            local_translation = T_local_delta[:3, 3]
            local_translation *= self.scale_factor

            T_target_ee = T_initial_ee.copy()  # noqa: N806
            T_target_ee[:3, 3] = T_initial_ee[:3, 3] + local_translation
            T_target_ee[:3, :3] = T_local_delta[:3, :3] @ T_initial_ee[:3, :3]

            target_pose = matrix_to_pose_msg(T_target_ee)
            return self._apply_pose_low_pass(target_pose)

        return None

    def _apply_pose_low_pass(self, target_pose: Pose) -> Pose:
        control_state = self.control_state
        previous_pose = control_state.filtered_target_ee_pose

        if previous_pose is None:
            control_state.filtered_target_ee_pose = target_pose
            return target_pose

        alpha = self.pose_low_pass_alpha
        blended_position = type(target_pose.position)(
            x=previous_pose.position.x
            + alpha * (target_pose.position.x - previous_pose.position.x),
            y=previous_pose.position.y
            + alpha * (target_pose.position.y - previous_pose.position.y),
            z=previous_pose.position.z
            + alpha * (target_pose.position.z - previous_pose.position.z),
        )

        previous_quaternion = [
            previous_pose.orientation.x,
            previous_pose.orientation.y,
            previous_pose.orientation.z,
            previous_pose.orientation.w,
        ]
        target_quaternion = [
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ]
        if (
            sum(
                previous_quaternion[i] * target_quaternion[i] for i in range(4)
            )
            < 0.0
        ):
            target_quaternion = [-value for value in target_quaternion]

        blended_quaternion = [
            (1.0 - alpha) * previous_quaternion[i]
            + alpha * target_quaternion[i]
            for i in range(4)
        ]
        quaternion_norm = math.sqrt(
            sum(value * value for value in blended_quaternion)
        )
        if quaternion_norm > 0.0:
            blended_quaternion = [
                value / quaternion_norm for value in blended_quaternion
            ]

        blended_pose = Pose(
            position=blended_position,
            orientation=type(target_pose.orientation)(
                x=blended_quaternion[0],
                y=blended_quaternion[1],
                z=blended_quaternion[2],
                w=blended_quaternion[3],
            ),
        )
        control_state.filtered_target_ee_pose = blended_pose
        return blended_pose

    def __call__(self) -> None | TeleOpResult:
        if self.latest_vr_state is None:
            return

        target_pose: Pose | None = self._get_target_ee_pose()

        if target_pose is None:
            return None

        solution = self.ik_solver.solve(
            target_pose, self.control_state.current_joint_state
        )

        if solution is None:
            fallback_solution = self.ik_solver.solve(
                target_pose,
                self.control_state.current_joint_state,
                orientation_mode=None,
            )
            if fallback_solution is not None:
                self.logger.info(
                    "IK fallback succeeded in position-only mode."
                )
                solution = fallback_solution

        return TeleOpResult(
            target_ee_pose=target_pose,
            solution=solution,
        )
