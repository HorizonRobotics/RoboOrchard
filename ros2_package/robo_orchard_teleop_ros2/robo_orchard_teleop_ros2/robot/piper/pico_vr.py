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

import os
from enum import Enum, unique
from typing import Any, Literal

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from robo_orchard_pico_msg_ros2.msg import (
    VRState,
)
from robo_orchard_teleop_ros2.bridge.pico.intent import (
    GripperIntent,
    ResetIntent,
)
from robo_orchard_teleop_ros2.bridge.pico.teleop import (
    Action,
    TeleOpResult,
    VRTeleOp,
)

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
]

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
    """The reset_ctrl chain is in-flight. VR output is suppressed and new
    VRState events are ignored until the chain resolves to DEACTIVE."""


class PiperPicoVRTeleOpNode(Node):
    def __init__(self, **kwargs):
        super().__init__("piper_pico_vr_teleop", **kwargs)

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

        # --- Reset service parameters ---
        # DAgger takeover/auto mode is controlled by the HoloBrain app via
        # vr_orchestrator services. This node only gates local VR command
        # output from the Pico controller.
        self.declare_parameter(
            "left_reset_service",
            "",
            descriptor=ParameterDescriptor(
                description=(
                    "Fully-qualified name of the left-side reset_ctrl "
                    "(std_srvs/Trigger) provided by piper_ros2. Empty "
                    "disables RESET for this side."
                )
            ),
        )
        self.declare_parameter(
            "right_reset_service",
            "",
            descriptor=ParameterDescriptor(
                description=(
                    "Fully-qualified name of the right-side reset_ctrl "
                    "(std_srvs/Trigger) provided by piper_ros2. Empty "
                    "disables RESET for this side."
                )
            ),
        )

        self._left_reset_service: str = (
            self.get_parameter("left_reset_service")
            .get_parameter_value()
            .string_value
        )
        self._right_reset_service: str = (
            self.get_parameter("right_reset_service")
            .get_parameter_value()
            .string_value
        )

        # --- Service clients for reset gesture orchestration ---
        self._left_reset_client = (
            self.create_client(Trigger, self._left_reset_service)
            if self._left_reset_service
            else None
        )
        self._right_reset_client = (
            self.create_client(Trigger, self._right_reset_service)
            if self._right_reset_service
            else None
        )

        # Best-effort pre-warm of DDS discovery so the VR-rate callback
        # does not block on it. Runtime falls back to a synthesized failure
        # response if a service stays unavailable.
        for client, service_name in (
            (self._left_reset_client, self._left_reset_service),
            (self._right_reset_client, self._right_reset_service),
        ):
            if client is None:
                continue
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning(
                    f"Service '{service_name}' not yet discovered; "
                    "runtime calls may fail until it appears."
                )

        # Surface a missing reset_service per side at startup so operators
        # know RESET will be inert until the param is configured.
        for side, reset_service in (
            ("left", self._left_reset_service),
            ("right", self._right_reset_service),
        ):
            if not reset_service:
                self.get_logger().warning(
                    f"[{side}] No reset_service configured; RESET gesture "
                    "will be a no-op for this side."
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

        self.left_teleop = VRTeleOp(
            source_type="left",
            urdf_path=self.urdf_path,
            base_link_name=self.base_link_name,
            ee_link_name=self.ee_link_name,
            scale_factor=TRANSLATION_SCALE_FACTOR,
            pose_low_pass_alpha=POSE_LOW_PASS_ALPHA,
            trigger_intent=GripperIntent(
                source_type="left",
                value_thresh=0.5,
                thresh=1.0,
            ),
            reset_intent=ResetIntent(source_type="X", thresh=1.0),
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
            trigger_intent=GripperIntent(
                source_type="right",
                value_thresh=0.5,
                thresh=1.0,
            ),
            reset_intent=ResetIntent(source_type="A", thresh=1.0),
            reset_callback=None,
            logger=self.get_logger(),
        )
        self.left_joint_state_msg: JointState | None = None
        self.right_joint_state_msg: JointState | None = None

        # sub
        self.vr_state_sub = self.create_subscription(
            VRState, "vr_state", self.sub_vr_state_callback, 1
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
        self.left_joint_state_msg = msg
        self.left_teleop.update_robot_joint_state(msg.position[:-1])

    def sub_right_ee_pose_callback(self, msg: PoseStamped):
        self.right_teleop.update_robot_ee_pose(msg.pose)

    def sub_right_joint_state_callback(self, msg: JointState):
        self.right_joint_state_msg = msg
        self.right_teleop.update_robot_joint_state(msg.position[:-1])

    def _reset_service_pair(
        self, side: Literal["left", "right"]
    ) -> tuple[str, Any]:
        """Return (service_name, client) for the side's reset_ctrl."""
        if side == "left":
            return self._left_reset_service, self._left_reset_client
        return self._right_reset_service, self._right_reset_client

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
        This state machine only gates Pico VR command output. DAgger
        takeover/auto mode is switched by the frontend app through the
        vr_orchestrator services, matching the ALOHA workflow.

        State transitions:
          DEACTIVE --[engage edge]--> WAITING_FOR_MATCH

          WAITING_FOR_MATCH --[release]--> DEACTIVE
          WAITING_FOR_MATCH --[match ok]--> ACTIVE

          ACTIVE --[release]--> DEACTIVE

          {WAITING_FOR_MATCH, ACTIVE} --[RESET]--> ARM_RESETTING
            ARM_RESETTING runs reset_ctrl -> DEACTIVE. See _handle_reset.
          DEACTIVE --[RESET]--> refused (no engage intent yet).
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

        # DEACTIVE: watch for the ACTIVE edge from GripperIntent.
        if state == ArmEngageState.DEACTIVE:
            if action == Action.ACTIVE:
                gripper_val = self._get_robot_gripper_value(side)
                if gripper_val is None:
                    self.get_logger().warning(
                        f"[{side}] Engage requested but joint_state not "
                        "yet received -- waiting for robot state."
                    )
                    # Stay DEACTIVE; GripperIntent keeps returning ACTIVE
                    # until the user releases, so we'll retry on the next
                    # VRState.
                    return
                self._arm_state[side] = ArmEngageState.WAITING_FOR_MATCH
                self.get_logger().info(
                    f"[{side}] Gripper intent active -- waiting for "
                    f"trigger to match robot gripper "
                    f"({gripper_val:.4f} m)."
                )
            # All other actions while DEACTIVE are ignored.
            return

        # WAITING_FOR_MATCH: check match; cancel on release.
        if state == ArmEngageState.WAITING_FOR_MATCH:
            if action == Action.DEACTIVE:
                # User released gripper -- cancel engage.
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
                    f"[{side}] Gripper released -- VR output paused. "
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
                "first VR frame may carry drift since gripper hold began."
            )
        self._arm_state[side] = ArmEngageState.ACTIVE
        self.get_logger().info(f"[{side}] {source} -- VR teleop is ACTIVE.")

    def _handle_reset(
        self,
        side: Literal["left", "right"],
        state: ArmEngageState,
    ) -> None:
        """Validate and kick off the RESET chain for one side.

        Refuse-cases (log + no-op):
          DEACTIVE                  : "engage first" -- nothing to reset
          ARM_RESETTING             : already in progress (idempotent)
          reset_service unconfigured: RESET wiring missing for this side

        Accept-cases (transition to ARM_RESETTING and dispatch chain):
          WAITING_FOR_MATCH: reset_ctrl
          ACTIVE           : reset_ctrl

        This gesture does not switch DAgger mode. Operators should use the
        frontend app's takeover/release services for AUTO/OVERRIDE changes.
        """
        if state == ArmEngageState.ARM_RESETTING:
            return  # idempotent

        if state == ArmEngageState.DEACTIVE:
            self.get_logger().info(
                f"[{side}] RESET ignored: engage gripper first."
            )
            return

        reset_service, _ = self._reset_service_pair(side)
        if not reset_service:
            self.get_logger().error(
                f"[{side}] RESET refused: no reset_service configured."
            )
            return

        # WAITING_FOR_MATCH or ACTIVE: dispatch the reset chain.
        self._arm_state[side] = ArmEngageState.ARM_RESETTING

        self.get_logger().info(
            f"[{side}] RESET requested (from {state.value}). "
            "Plan: reset_ctrl only; DAgger mode unchanged."
        )

        self._reset_dispatch_reset_ctrl(side)

    def _reset_dispatch_reset_ctrl(
        self,
        side: Literal["left", "right"],
    ) -> None:
        """Ask piper_ros2 to drive the arm to its configured home pose.

        reset_ctrl bypasses the muxer entirely (writes through SDK) and
        blocks server-side for ~3 s while running its joint interpolation.
        This node does not switch muxer mode around it.
        """
        reset_service, reset_client = self._reset_service_pair(side)

        # reset_ctrl is dispatched async; the only path back to DEACTIVE is
        # _on_done. If the downstream service hangs (server crash, CAN bus
        # lockup) the future never resolves, _on_done never fires, and this
        # side stays in ARM_RESETTING until the process restarts. Surface
        # that diagnostic up front so the operator knows what to look for.
        self.get_logger().warning(
            f"[{side}] RESET dispatched to '{reset_service}' (async, "
            f"expected ~3 s). If no '[{side}] RESET complete' log follows "
            "within ~10 s, the downstream service is hung -- restart the "
            "teleop node to recover this side."
        )

        def _on_done(resp, _side=side):
            if not resp.success:
                self.get_logger().error(
                    f"[{_side}] RESET: reset_ctrl failed ({resp.message})."
                )
            else:
                self.get_logger().info(f"[{_side}] RESET: arm at home.")
            self._arm_state[_side] = ArmEngageState.DEACTIVE
            self.get_logger().info(f"[{_side}] RESET complete -- DEACTIVE.")

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
        ret: TeleOpResult,
        gripper: float,
        header: Header,
        joint_state_cmd_publisher,
        target_pose_publisher,
    ):
        if self.enable_pose_control and ret.solution is not None:
            positions = list(ret.solution)
            positions.append(gripper)
            joint_state_msg = JointState(
                header=header,
                name=JOINT_NAMES,
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
                ret=left_ret,
                gripper=_trigger_to_gripper_position(
                    self.left_teleop.latest_vr_state.left_controller.trigger
                ),
                header=Header(frame_id="/robot/left", stamp=current_stamp),
                joint_state_cmd_publisher=self.left_cmd_pub,
                target_pose_publisher=self.left_target_pub,
            )

        if right_ret is not None:
            self._handle_teleop_result(
                ret=right_ret,
                gripper=_trigger_to_gripper_position(
                    self.right_teleop.latest_vr_state.right_controller.trigger
                ),
                header=Header(frame_id="/robot/right", stamp=current_stamp),
                joint_state_cmd_publisher=self.right_cmd_pub,
                target_pose_publisher=self.right_target_pub,
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
