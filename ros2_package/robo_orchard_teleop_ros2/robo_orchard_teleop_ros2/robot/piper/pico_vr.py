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
import os
from enum import Enum, unique
from typing import Literal

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Header
from std_srvs.srv import Trigger

from robo_orchard_pico_msg_ros2.msg import (
    VRState,
)
from robo_orchard_teleop_msg_ros2.msg import ControlMode, TeleopActivationState
from robo_orchard_teleop_ros2.bridge.pico.intent import (
    DisabledResetIntent,
    InactiveActivationIntent,
    PicoActivationIntent,
    ResetIntent,
    TopicActivationIntent,
)
from robo_orchard_teleop_ros2.bridge.pico.teleop import (
    Action,
    TeleOpResult,
    VRTeleOp,
)

JOINT_NAMES = [f"joint{index}" for index in range(1, 7)] + ["gripper"]


def _joint_names_from_feedback(
    joint_state: JointState | None, expected_names: list[str]
) -> list[str]:
    """Require the configured hardware/IK order before using joint values."""
    if joint_state is None:
        raise ValueError("No joint feedback")
    names = list(joint_state.name)
    if (
        names != expected_names
        or len(joint_state.position) != len(names)
        or not all(math.isfinite(value) for value in joint_state.position)
    ):
        raise ValueError(
            "Joint feedback must match the configured hardware/IK order "
            "and contain finite positions"
        )
    return names


TELEOP_CONTROL_FREQ_HZ = 30.0
TRANSLATION_SCALE_FACTOR = 1.2
POSE_LOW_PASS_ALPHA = 0.25
# Piper gripper total opening upper limit (meters). Matches piper_x_description
# URDF. Used to map the VR controller trigger's [0, 1] range onto the full
# gripper travel. Trigger release maps to open gripper; pressing the trigger
# closes it, matching a natural pinch gesture.
PIPER_MAX_GRIPPER_OPENING_M = 0.1


def _trigger_to_gripper_position(trigger: float) -> float:
    """Map a VR trigger value to Piper gripper opening in metres."""
    return (1.0 - trigger) * PIPER_MAX_GRIPPER_OPENING_M


@unique
class ArmEngageState(Enum):
    """Per-side match-before-engage handoff state machine states."""

    DEACTIVE = "DEACTIVE"
    WAITING_FOR_MATCH = "WAITING_FOR_MATCH"
    ACTIVE = "ACTIVE"
    ARM_RESETTING = "ARM_RESETTING"
    """The Manager reset is in-flight. VR output is suppressed and new
    VRState events are ignored until the chain resolves to DEACTIVE. The
    shared teleop state then requires a gripper release before re-engaging."""


class PiperPicoVRTeleOpNode(Node):
    def __init__(self, **kwargs):
        super().__init__("piper_pico_vr_teleop", **kwargs)

        self.joint_names: dict[str, list[str]] = {}
        for side in ("left", "right"):
            parameter = f"{side}_joint_names"
            self.declare_parameter(parameter, list(JOINT_NAMES))
            names = self.get_parameter(parameter).value
            if (
                not isinstance(names, (list, tuple))
                or len(names) != 7
                or any(
                    not isinstance(name, str) or not name.strip()
                    for name in names
                )
                or len(set(names)) != 7
            ):
                raise ValueError(
                    f"{parameter} must contain seven unique non-empty "
                    "strings in hardware/IK order (six arm joints, "
                    "then gripper)"
                )
            self.joint_names[side] = list(names)

        self.declare_parameter(
            "urdf_path",
            "",
            descriptor=ParameterDescriptor(description="URDF Path"),
        )
        self.urdf_path: str = (
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )
        if not os.path.exists(self.urdf_path):
            raise FileNotFoundError(
                f"urdf file {self.urdf_path} does not exists!"
            )

        self.declare_parameter(
            "ee_link_name",
            "gripper_base",
            descriptor=ParameterDescriptor(
                description="IK end-effector link."
            ),
        )
        self.ee_link_name: str = (
            self.get_parameter("ee_link_name")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter(
            "enable_pose_control",
            True,
            descriptor=ParameterDescriptor(
                description="Enable or not enable pose control."
            ),
        )
        self.enable_pose_control: bool = (
            self.get_parameter("enable_pose_control")
            .get_parameter_value()
            .bool_value
        )

        self.declare_parameter(
            "base_link_name",
            "base_link",
            descriptor=ParameterDescriptor(
                description="Base link name used by the IK solver."
            ),
        )
        self.base_link_name: str = (
            self.get_parameter("base_link_name")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter("operator_input_source", "pico")
        self.declare_parameter("keyboard_control_side", "both")
        self.declare_parameter(
            "keyboard_activation_topic", "/teleop/activation/state"
        )
        self.declare_parameter("keyboard_reset_topic", "/teleop/reset")
        self.declare_parameter("keyboard_activation_timeout_s", 0.2)
        self._operator_input_source = str(
            self.get_parameter("operator_input_source").value
        )
        self._keyboard_control_side = str(
            self.get_parameter("keyboard_control_side").value
        )
        self._keyboard_activation_topic = str(
            self.get_parameter("keyboard_activation_topic").value
        )
        self._keyboard_reset_topic = str(
            self.get_parameter("keyboard_reset_topic").value
        )
        self._keyboard_activation_timeout_s = float(
            self.get_parameter("keyboard_activation_timeout_s").value
        )
        if self._operator_input_source not in {"pico", "keyboard"}:
            raise ValueError(
                "operator_input_source must be 'pico' or 'keyboard'"
            )
        if self._keyboard_control_side not in {"left", "right", "both"}:
            raise ValueError(
                "keyboard_control_side must be 'left', 'right', or 'both'"
            )
        if not self._keyboard_activation_topic:
            raise ValueError("keyboard_activation_topic must not be empty")
        if not self._keyboard_reset_topic:
            raise ValueError("keyboard_reset_topic must not be empty")
        if self._keyboard_activation_timeout_s <= 0.0:
            raise ValueError("keyboard_activation_timeout_s must be positive")

        # --- Global reset intent ---
        self.declare_parameter(
            "reset_service",
            "/robot/control/reset",
            descriptor=ParameterDescriptor(
                description=(
                    "Fully-qualified Control Manager reset service. Both "
                    "Pico reset gestures request this same global reset."
                )
            ),
        )
        self._reset_service: str = (
            self.get_parameter("reset_service")
            .get_parameter_value()
            .string_value
        )

        self._manager_resetting = False
        self._reset_pending = False
        self._reset_client = (
            self.create_client(Trigger, self._reset_service)
            if self._reset_service
            else None
        )

        # Best-effort pre-warm of DDS discovery so the VR-rate callback
        # does not block on it. Runtime falls back to a synthesized failure
        # response if a service stays unavailable.
        if self._reset_client is not None:
            if not self._reset_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning(
                    f"Service '{self._reset_service}' not yet discovered; "
                    "runtime calls may fail until it appears."
                )
        else:
            self.get_logger().warning(
                "No reset_service configured; RESET gestures will be no-op."
            )

        # --- Match-before-engage state machine ---
        self._arm_state: dict[str, ArmEngageState] = {
            "left": ArmEngageState.DEACTIVE,
            "right": ArmEngageState.DEACTIVE,
        }

        self.declare_parameter(
            "match_tolerance",
            0.1,
            descriptor=ParameterDescriptor(
                description=(
                    "Normalised trigger-vs-gripper match tolerance for "
                    "match-before-engage handoff. Default 0.1 (10 %% of "
                    "full range)."
                )
            ),
        )
        self._match_tolerance: float = (
            self.get_parameter("match_tolerance")
            .get_parameter_value()
            .double_value
        )

        self._topic_activation_intent = (
            TopicActivationIntent(self._keyboard_activation_timeout_s)
            if self._operator_input_source == "keyboard"
            else None
        )

        self.left_teleop = VRTeleOp(
            source_type="left",
            urdf_path=self.urdf_path,
            base_link_name=self.base_link_name,
            ee_link_name=self.ee_link_name,
            scale_factor=TRANSLATION_SCALE_FACTOR,
            pose_low_pass_alpha=POSE_LOW_PASS_ALPHA,
            trigger_intent=self._activation_intent_for_side("left"),
            reset_intent=self._reset_intent_for_side("left"),
            reset_callback=None,
            logger=self.get_logger(),
        )
        self.right_teleop = VRTeleOp(
            source_type="right",
            urdf_path=self.urdf_path,
            base_link_name=self.base_link_name,
            ee_link_name=self.ee_link_name,
            scale_factor=TRANSLATION_SCALE_FACTOR,
            pose_low_pass_alpha=POSE_LOW_PASS_ALPHA,
            trigger_intent=self._activation_intent_for_side("right"),
            reset_intent=self._reset_intent_for_side("right"),
            reset_callback=None,
            logger=self.get_logger(),
        )
        self.left_joint_state_msg: JointState | None = None
        self.right_joint_state_msg: JointState | None = None

        # sub
        self.vr_state_sub = self.create_subscription(
            VRState, "vr_state", self.sub_vr_state_callback, 1
        )
        self.create_subscription(
            ControlMode,
            "/robot/control/status",
            self._on_control_status,
            10,
        )
        self.keyboard_activation_sub = None
        self.keyboard_reset_sub = None
        if self._operator_input_source == "keyboard":
            self.keyboard_activation_sub = self.create_subscription(
                TeleopActivationState,
                self._keyboard_activation_topic,
                self._on_keyboard_activation,
                1,
            )
            self.keyboard_reset_sub = self.create_subscription(
                Empty,
                self._keyboard_reset_topic,
                self._on_keyboard_reset,
                1,
            )
        self.left_ee_pose_sub = self.create_subscription(
            PoseStamped,
            "/robot/left/ee_pose",
            self.sub_left_ee_pose_callback,
            1,
        )
        self.left_joint_state_sub = self.create_subscription(
            JointState,
            "/robot/left/joint_state",
            self.sub_left_joint_state_callback,
            1,
        )
        self.right_ee_pose_sub = self.create_subscription(
            PoseStamped,
            "/robot/right/ee_pose",
            self.sub_right_ee_pose_callback,
            1,
        )
        self.right_joint_state_sub = self.create_subscription(
            JointState,
            "/robot/right/joint_state",
            self.sub_right_joint_state_callback,
            1,
        )

        # pub
        self.left_cmd_pub = self.create_publisher(
            JointState,
            "/robot/left/joint_cmd",
            1,
        )
        self.left_target_pub = self.create_publisher(
            PoseStamped, "/robot/left/ee_pose_target", 1
        )

        self.right_cmd_pub = self.create_publisher(
            JointState,
            "/robot/right/joint_cmd",
            1,
        )
        self.right_target_pub = self.create_publisher(
            PoseStamped, "/robot/right/ee_pose_target", 1
        )

        # timer
        self.timer = self.create_timer(
            1.0 / TELEOP_CONTROL_FREQ_HZ, self.timer_callback
        )

        self.get_logger().info(
            "Piper teleop operator input: "
            f"source={self._operator_input_source}, "
            f"keyboard_side={self._keyboard_control_side}"
        )

    def _keyboard_sides(self) -> tuple[Literal["left", "right"], ...]:
        if self._keyboard_control_side == "both":
            return ("left", "right")
        return (self._keyboard_control_side,)

    def _activation_intent_for_side(self, side: Literal["left", "right"]):
        if self._operator_input_source == "pico":
            return PicoActivationIntent(
                source_type=side,
                value_thresh=0.5,
                thresh=1.0,
            )
        if side in self._keyboard_sides():
            return self._topic_activation_intent
        return InactiveActivationIntent()

    def _reset_intent_for_side(self, side: Literal["left", "right"]):
        if self._operator_input_source == "keyboard":
            return DisabledResetIntent()
        return ResetIntent(
            source_type="X" if side == "left" else "A",
            thresh=1.0,
        )

    def _on_keyboard_activation(self, message: TeleopActivationState) -> None:
        if self._topic_activation_intent is not None:
            self._topic_activation_intent.update(message)

    def _on_keyboard_reset(self, _message: Empty) -> None:
        if self._topic_activation_intent is None:
            return
        self._topic_activation_intent.require_rearm()
        side = self._keyboard_sides()[0]
        self._handle_reset(side, self._arm_state[side])

    def sub_vr_state_callback(self, msg: VRState):
        # The state machine itself gates VR processing during ARM_RESETTING
        # (it ignores all non-RESET actions in that state), so no per-side
        # _is_resetting flag is needed here.
        left_action = self.left_teleop.update_vr_state(msg)
        self._on_vr_state_side("left", left_action, msg)

        right_action = self.right_teleop.update_vr_state(msg)
        self._on_vr_state_side("right", right_action, msg)

    def sub_left_ee_pose_callback(self, msg: PoseStamped):
        self.left_teleop.update_robot_ee_pose(msg.pose)

    def sub_left_joint_state_callback(self, msg: JointState):
        try:
            _joint_names_from_feedback(msg, self.joint_names["left"])
        except ValueError as error:
            self.left_joint_state_msg = None
            self.get_logger().error(f"Rejecting left joint feedback: {error}")
            return
        self.left_joint_state_msg = msg
        self.left_teleop.update_robot_joint_state(msg.position[:-1])

    def sub_right_ee_pose_callback(self, msg: PoseStamped):
        self.right_teleop.update_robot_ee_pose(msg.pose)

    def sub_right_joint_state_callback(self, msg: JointState):
        try:
            _joint_names_from_feedback(msg, self.joint_names["right"])
        except ValueError as error:
            self.right_joint_state_msg = None
            self.get_logger().error(f"Rejecting right joint feedback: {error}")
            return
        self.right_joint_state_msg = msg
        self.right_teleop.update_robot_joint_state(msg.position[:-1])

    def _get_robot_gripper_value(
        self, side: Literal["left", "right"]
    ) -> float | None:
        """Return the robot gripper position (metres) for one side.

        Returns None if the joint_state has not yet been received or does not
        have enough joints. The caller must handle None before attempting a
        match check.
        """
        if side == "left":
            joint_state = self.left_joint_state_msg
        elif side == "right":
            joint_state = self.right_joint_state_msg
        else:
            raise ValueError(f"Invalid side: {side!r}")
        if joint_state is None:
            return None
        if len(joint_state.position) <= 6:
            return None
        return float(joint_state.position[6])

    def _call_service_async(
        self,
        client,
        service_name: str,
        done_callback,
    ) -> None:
        """Fire-and-forget async service call with a done callback.

        Checks client readiness before calling. On failure (not ready, empty
        name, or None client), invokes done_callback immediately with a
        failed Trigger.Response so the state machine can transition to a
        safe state.

        The done_callback receives a Trigger.Response; response.success
        indicates success or failure.
        """
        if client is None or not service_name:
            self.get_logger().error(
                f"Service client for '{service_name}' is not configured."
            )
            failed = Trigger.Response()
            failed.success = False
            failed.message = f"Service '{service_name}' is not configured."
            done_callback(failed)
            return

        if not client.service_is_ready():
            self.get_logger().error(
                f"Service '{service_name}' is not available."
            )
            failed = Trigger.Response()
            failed.success = False
            failed.message = f"Service '{service_name}' is not available."
            done_callback(failed)
            return

        future = client.call_async(Trigger.Request())

        def _on_done(f):
            try:
                resp = f.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"Service '{service_name}' call raised: {exc}"
                )
                resp = Trigger.Response()
                resp.success = False
                resp.message = str(exc)
            done_callback(resp)

        future.add_done_callback(_on_done)

    def _on_vr_state_side(
        self,
        side: Literal["left", "right"],
        action: Action,
        vr_state,
    ) -> None:
        """Drive the per-side match-before-engage state machine.

        Called from sub_vr_state_callback for each side independently.
        This state machine only gates Pico VR command output. Control Manager
        mode transitions are requested explicitly by external callers.

        State transitions:
          DEACTIVE --[engage edge]--> WAITING_FOR_MATCH

          WAITING_FOR_MATCH --[release]--> DEACTIVE
          WAITING_FOR_MATCH --[match ok]--> ACTIVE

          ACTIVE --[release]--> DEACTIVE

          {DEACTIVE, WAITING_FOR_MATCH, ACTIVE} --[RESET]--> ARM_RESETTING
            ARM_RESETTING runs Manager reset -> DEACTIVE. See _handle_reset.
        """
        state = self._arm_state[side]

        # RESET is handled identically regardless of underlying engage
        # state -- _handle_reset decides whether to accept, refuse, or
        # treat as no-op based on the current state.
        if action == Action.RESET:
            self._handle_reset(side, state)
            return

        # ARM_RESETTING: ignore every non-RESET VRState event so the
        # in-flight reset chain runs without interference.
        if state == ArmEngageState.ARM_RESETTING:
            return

        # DEACTIVE: watch for the selected operator input's ACTIVE edge.
        # Both operator input sources take this path: keyboard activation
        # replaces only the engage signal, so the gripper still follows the
        # Pico trigger and must match before the arm engages.
        if state == ArmEngageState.DEACTIVE:
            if action == Action.ACTIVE:
                gripper_val = self._get_robot_gripper_value(side)
                if gripper_val is None:
                    self.get_logger().warning(
                        f"[{side}] Engage requested but joint_state not "
                        "yet received -- waiting for robot state."
                    )
                    # Stay DEACTIVE; the operator input keeps returning
                    # ACTIVE until the user releases, so we'll retry on the
                    # next VRState.
                    return
                self._arm_state[side] = ArmEngageState.WAITING_FOR_MATCH
                self.get_logger().info(
                    f"[{side}] {self._operator_input_source.capitalize()} "
                    f"activation active -- waiting for trigger to match "
                    f"robot gripper ({gripper_val:.4f} m)."
                )
            # All other actions while DEACTIVE are ignored.
            return

        # WAITING_FOR_MATCH: check match; cancel on release.
        if state == ArmEngageState.WAITING_FOR_MATCH:
            if action == Action.DEACTIVE:
                # Operator released the selected activation input.
                self._arm_state[side] = ArmEngageState.DEACTIVE
                self.get_logger().info(
                    f"[{side}] Engage cancelled before match (released)."
                )
                return

            # User is still holding -- check match.
            gripper_val = self._get_robot_gripper_value(side)
            if gripper_val is None:
                self.get_logger().warning(
                    f"[{side}] Waiting for match but joint_state not "
                    "available."
                )
                return

            controller = (
                vr_state.left_controller
                if side == "left"
                else vr_state.right_controller
            )
            trigger_as_gripper = _trigger_to_gripper_position(
                controller.trigger
            )
            normalised_delta = (
                abs(trigger_as_gripper - gripper_val)
                / PIPER_MAX_GRIPPER_OPENING_M
            )

            if normalised_delta >= self._match_tolerance:
                # Not matched yet -- stay and wait. Log only at debug to
                # avoid flooding; the caller (sub_vr_state_callback) runs
                # at VR frame rate (~72 Hz).
                self.get_logger().debug(
                    f"[{side}] Waiting for match: "
                    f"trigger={trigger_as_gripper:.4f} m, "
                    f"robot={gripper_val:.4f} m, "
                    f"delta={normalised_delta:.3f} "
                    f"(tol={self._match_tolerance})."
                )
                return

            self.get_logger().info(
                f"[{side}] Match satisfied "
                f"(delta={normalised_delta:.3f}). Engaging VR output."
            )
            self._engage_active(side, source="Engaged")
            return

        # ACTIVE: watch for the DEACTIVE edge.
        if state == ArmEngageState.ACTIVE:
            if action == Action.DEACTIVE:
                self.get_logger().info(
                    f"[{side}] Operator input released -- VR output paused. "
                    "DAgger mode is unchanged."
                )
                self._arm_state[side] = ArmEngageState.DEACTIVE
            return

    def _engage_active(
        self,
        side: Literal["left", "right"],
        *,
        source: str,
    ) -> None:
        """Refresh baseline and transition `side` to ACTIVE.

        Called once match-before-engage succeeds so the first ACTIVE frame
        starts from the current controller pose.
        """
        # Refresh baseline so the first ACTIVE frame mirrors the
        # controller pose NOW, not the stale long-press snapshot.
        teleop = self.left_teleop if side == "left" else self.right_teleop
        if not teleop.recapture_baseline():
            self.get_logger().warning(
                f"[{side}] Could not refresh engage baseline at engage; "
                "first VR frame may carry drift since activation began."
            )
        self._arm_state[side] = ArmEngageState.ACTIVE
        self.get_logger().info(f"[{side}] {source} -- VR teleop is ACTIVE.")

    def _on_control_status(self, message: ControlMode) -> None:
        resetting = message.data == ControlMode.RESETTING
        if resetting == self._manager_resetting:
            return
        self._manager_resetting = resetting
        if resetting:
            if self._topic_activation_intent is not None:
                self._topic_activation_intent.require_rearm()
            self._begin_reset_state()
        else:
            self._finish_reset_state()

    def _begin_reset_state(self) -> None:
        for side, teleop in (
            ("left", self.left_teleop),
            ("right", self.right_teleop),
        ):
            self._arm_state[side] = ArmEngageState.ARM_RESETTING
            teleop.begin_reset()

    def _finish_reset_state(self) -> None:
        if self._manager_resetting or self._reset_pending:
            return
        for side, teleop in (
            ("left", self.left_teleop),
            ("right", self.right_teleop),
        ):
            self._arm_state[side] = ArmEngageState.DEACTIVE
            teleop.finish_reset()

    def _handle_reset(
        self,
        side: Literal["left", "right"],
        state: ArmEngageState,
    ) -> None:
        """Reset both local sessions before dispatching a global reset.

        Refuse-cases (log + no-op):
          ARM_RESETTING             : already in progress (idempotent)
          reset_service unconfigured: RESET wiring missing

        Accept-cases (transition to ARM_RESETTING and dispatch chain):
          DEACTIVE         : Manager reset
          WAITING_FOR_MATCH: Manager reset
          ACTIVE           : Manager reset

        This gesture does not select the post-reset mode. Control Manager
        mode transitions are requested explicitly by external callers.
        """
        if state == ArmEngageState.ARM_RESETTING:
            return  # idempotent

        if not self._reset_service:
            self.get_logger().error(
                f"[{side}] RESET refused: no reset_service configured."
            )
            self._arm_state[side] = ArmEngageState.DEACTIVE
            teleop = self.left_teleop if side == "left" else self.right_teleop
            teleop.finish_reset()
            return

        self._begin_reset_state()

        self.get_logger().info(
            f"[{side}] RESET requested (from {state.value}). "
            "Dispatching the global Control Manager reset."
        )

        self._reset_dispatch_manager(side)

    def _reset_dispatch_manager(
        self,
        side: Literal["left", "right"],
    ) -> None:
        """Ask the Control Manager to reset its complete configured scope."""
        reset_service = self._reset_service
        reset_client = self._reset_client

        self._reset_pending = True
        self.get_logger().warning(
            f"[{side}] RESET dispatched to '{reset_service}' (async)."
        )

        def _on_done(resp, _side=side):
            if not resp.success:
                self.get_logger().error(
                    f"[{_side}] RESET: manager reset failed ({resp.message})."
                )
            else:
                self.get_logger().info(
                    f"[{_side}] RESET: configured hardware is at home."
                )
            self._reset_pending = False
            self._finish_reset_state()
            self.get_logger().info(
                f"[{_side}] RESET response received; re-engagement requires "
                "Manager reset completion and activation release."
            )

        self._call_service_async(reset_client, reset_service, _on_done)

    def _should_drive_side(self, side: Literal["left", "right"]) -> bool:
        """Whether to compute IK and publish joint_cmd for this side this tick.

        Driven only in ACTIVE -- every other state (including the
        ARM_RESETTING window) returns False so the per-side state
        machine fully controls when VR output is emitted.
        """
        return self._arm_state[side] == ArmEngageState.ACTIVE

    def _handle_teleop_result(
        self,
        side: Literal["left", "right"],
        ret: TeleOpResult,
        gripper: float,
        header: Header,
        joint_state_cmd_publisher,
        target_pose_publisher,
        joint_state: JointState | None,
    ):
        if self.enable_pose_control and ret.solution is not None:
            positions = list(ret.solution)
            positions.append(gripper)
            try:
                names = _joint_names_from_feedback(
                    joint_state, self.joint_names[side]
                )
                if len(positions) != len(names) or not all(
                    math.isfinite(value) for value in positions
                ):
                    raise ValueError("Invalid IK joint positions")
            except ValueError as error:
                self.get_logger().error(
                    f"Cannot publish joint command: {error}"
                )
            else:
                joint_state_msg = JointState(
                    header=header,
                    name=names,
                    position=positions,
                )
                joint_state_cmd_publisher.publish(joint_state_msg)

        pose_msg = PoseStamped(header=header, pose=ret.target_ee_pose)
        target_pose_publisher.publish(pose_msg)

    def timer_callback(self):
        current_stamp = self.get_clock().now().to_msg()

        left_ret = (
            self.left_teleop() if self._should_drive_side("left") else None
        )
        right_ret = (
            self.right_teleop() if self._should_drive_side("right") else None
        )

        if left_ret is not None:
            self._handle_teleop_result(
                side="left",
                ret=left_ret,
                gripper=_trigger_to_gripper_position(
                    self.left_teleop.latest_vr_state.left_controller.trigger
                ),
                header=Header(frame_id="/robot/left", stamp=current_stamp),
                joint_state_cmd_publisher=self.left_cmd_pub,
                target_pose_publisher=self.left_target_pub,
                joint_state=self.left_joint_state_msg,
            )

        if right_ret is not None:
            self._handle_teleop_result(
                side="right",
                ret=right_ret,
                gripper=_trigger_to_gripper_position(
                    self.right_teleop.latest_vr_state.right_controller.trigger
                ),
                header=Header(frame_id="/robot/right", stamp=current_stamp),
                joint_state_cmd_publisher=self.right_cmd_pub,
                target_pose_publisher=self.right_target_pub,
                joint_state=self.right_joint_state_msg,
            )


def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    node = PiperPicoVRTeleOpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
